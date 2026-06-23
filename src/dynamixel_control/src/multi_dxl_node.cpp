#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int32MultiArray.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float32MultiArray.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk/group_sync_read.h>
#include "dynamixel_control/GetCurrent.h"
#include "dynamixel_control/SetCurrent.h"
#include "dynamixel_control/SetOperatingMode.h"
#include "dynamixel_control/SetPosition.h"
#include "dynamixel_control/GetPosition.h"
#include "dynamixel_control/GetBulkPositions.h"
#include "dynamixel_control/GetBulkCurrents.h"
#include "dynamixel_control/GetPositionTuning.h"
#include "dynamixel_control/SetPositionTuning.h"
#include <algorithm>
#include <cstdlib>
#include <map>
#include <set>
#include <string>
#include <vector>
#include <mutex>
#include <xmlrpcpp/XmlRpcValue.h>

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_PWM_LIMIT 36
#define ADDR_CURRENT_LIMIT 38
#define ADDR_TORQUE_ENABLE 64
#define ADDR_POSITION_D_GAIN 80
#define ADDR_POSITION_I_GAIN 82
#define ADDR_POSITION_P_GAIN 84
#define ADDR_GOAL_CURRENT 102
#define ADDR_PROFILE_ACCELERATION 108
#define ADDR_PROFILE_VELOCITY 112
#define ADDR_GOAL_POSITION 116
#define ADDR_PRESENT_CURRENT 126
#define ADDR_PRESENT_POSITION 132

// Protocol version
#define PROTOCOL_VERSION 2.0

// Default setting
#define BAUDRATE 1000000
#define DEVICE_NAME "/dev/ttyUSB0"

using namespace dynamixel;

PortHandler *portHandler;
PacketHandler *packetHandler;

uint8_t dxl_error = 0;
int dxl_comm_result;
int32_t present_position = 0;
int16_t present_current = 0;

std::mutex serial_mutex;

dynamixel::GroupSyncRead *group_sync_reader = nullptr;
dynamixel::GroupSyncRead *current_group_sync_reader = nullptr;
dynamixel::GroupSyncWrite *group_sync_writer = nullptr;

ros::Publisher joint_state_pub;
ros::Publisher joint_currents_pub;
std::map<uint8_t, int32_t> last_tick_positions;
ros::Time last_telemetry_time;
int telemetry_rate_hz = 50;

std::vector<uint8_t> controlled_ids;
std::set<uint8_t> multi_turn_ids;
std::map<uint8_t, uint8_t> default_operating_modes;
std::map<uint8_t, uint8_t> current_operating_modes;

struct PositionGains
{
    int p;
    int i;
    int d;
};

struct PositionProfile
{
    int velocity;
    int acceleration;
};

std::map<uint8_t, PositionGains> position_gains_by_id;
std::map<uint8_t, PositionProfile> position_profiles_by_id;

bool containsId(const std::vector<uint8_t> &ids, uint8_t id)
{
    return std::find(ids.begin(), ids.end(), id) != ids.end();
}

std::vector<uint8_t> intListToUint8(const std::vector<int> &values)
{
    std::vector<uint8_t> out;
    out.reserve(values.size());
    for (size_t i = 0; i < values.size(); ++i)
    {
        if (values[i] < 0 || values[i] > 252)
        {
            ROS_WARN("Ignore invalid motor id %d", values[i]);
            continue;
        }
        out.push_back(static_cast<uint8_t>(values[i]));
    }
    return out;
}

int clampPositionGain(int value)
{
    return std::max(0, std::min(16383, value));
}

int clampProfileValue(int value)
{
    return std::max(0, value);
}

bool readXmlRpcInt(XmlRpc::XmlRpcValue &value, const std::string &key, int default_value, int *out)
{
    if (value.getType() != XmlRpc::XmlRpcValue::TypeStruct || !value.hasMember(key))
    {
        *out = default_value;
        return true;
    }

    const XmlRpc::XmlRpcValue &entry = value[key];
    if (entry.getType() != XmlRpc::XmlRpcValue::TypeInt)
        return false;

    *out = static_cast<int>(entry);
    return true;
}

void loadPositionGains(const std::string &board_prefix)
{
    XmlRpc::XmlRpcValue gains_param;
    bool found = false;
    if (!board_prefix.empty())
        found = ros::param::get(board_prefix + "position_gains", gains_param);
    else
        found = ros::param::get("position_gains", gains_param);

    if (!found)
        return;

    if (gains_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
        ROS_WARN("position_gains must be a map keyed by motor id");
        return;
    }

    for (XmlRpc::XmlRpcValue::iterator it = gains_param.begin(); it != gains_param.end(); ++it)
    {
        const int id_int = std::atoi(it->first.c_str());
        if (id_int < 0 || id_int > 252)
        {
            ROS_WARN("Ignore invalid position_gains motor id '%s'", it->first.c_str());
            continue;
        }

        int p = 900;
        int i = 0;
        int d = 0;
        if (!readXmlRpcInt(it->second, "p", p, &p) ||
            !readXmlRpcInt(it->second, "i", i, &i) ||
            !readXmlRpcInt(it->second, "d", d, &d))
        {
            ROS_WARN("Ignore position_gains for ID %d: expected integer p/i/d fields", id_int);
            continue;
        }

        PositionGains gains;
        gains.p = clampPositionGain(p);
        gains.i = clampPositionGain(i);
        gains.d = clampPositionGain(d);
        position_gains_by_id[static_cast<uint8_t>(id_int)] = gains;
    }

    ROS_INFO("Loaded position gain overrides for %zu motors", position_gains_by_id.size());
}

void loadPositionProfiles(const std::string &board_prefix)
{
    XmlRpc::XmlRpcValue profiles_param;
    bool found = false;
    if (!board_prefix.empty())
        found = ros::param::get(board_prefix + "position_profiles", profiles_param);
    else
        found = ros::param::get("position_profiles", profiles_param);

    if (!found)
        return;
    if (profiles_param.getType() != XmlRpc::XmlRpcValue::TypeStruct)
    {
        ROS_WARN("position_profiles must be a map keyed by motor id");
        return;
    }

    for (XmlRpc::XmlRpcValue::iterator it = profiles_param.begin(); it != profiles_param.end(); ++it)
    {
        const int id_int = std::atoi(it->first.c_str());
        if (id_int < 0 || id_int > 252)
        {
            ROS_WARN("Ignore invalid position_profiles motor id '%s'", it->first.c_str());
            continue;
        }

        int velocity = 200;
        int acceleration = 100;
        if (!readXmlRpcInt(it->second, "velocity", velocity, &velocity) ||
            !readXmlRpcInt(it->second, "acceleration", acceleration, &acceleration))
        {
            ROS_WARN("Ignore position_profiles for ID %d: expected integer velocity/acceleration fields", id_int);
            continue;
        }

        PositionProfile profile;
        profile.velocity = clampProfileValue(velocity);
        profile.acceleration = clampProfileValue(acceleration);
        position_profiles_by_id[static_cast<uint8_t>(id_int)] = profile;
    }

    ROS_INFO("Loaded position profile overrides for %zu motors", position_profiles_by_id.size());
}

bool writePositionGains(uint8_t id, const PositionGains &gains, std::string *error)
{
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_POSITION_D_GAIN,
                                                    static_cast<uint16_t>(gains.d), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        if (error != nullptr)
            *error = packetHandler->getTxRxResult(dxl_comm_result);
        return false;
    }
    if (dxl_error != 0)
    {
        if (error != nullptr)
            *error = packetHandler->getRxPacketError(dxl_error);
        return false;
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_POSITION_I_GAIN,
                                                    static_cast<uint16_t>(gains.i), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        if (error != nullptr)
            *error = packetHandler->getTxRxResult(dxl_comm_result);
        return false;
    }
    if (dxl_error != 0)
    {
        if (error != nullptr)
            *error = packetHandler->getRxPacketError(dxl_error);
        return false;
    }

    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_POSITION_P_GAIN,
                                                    static_cast<uint16_t>(gains.p), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        if (error != nullptr)
            *error = packetHandler->getTxRxResult(dxl_comm_result);
        return false;
    }
    if (dxl_error != 0)
    {
        if (error != nullptr)
            *error = packetHandler->getRxPacketError(dxl_error);
        return false;
    }
    return true;
}

bool writePositionProfile(uint8_t id, const PositionProfile &profile, std::string *error)
{
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_ACCELERATION,
                                                    static_cast<uint32_t>(profile.acceleration), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        if (error != nullptr)
            *error = packetHandler->getTxRxResult(dxl_comm_result);
        return false;
    }
    if (dxl_error != 0)
    {
        if (error != nullptr)
            *error = packetHandler->getRxPacketError(dxl_error);
        return false;
    }

    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY,
                                                    static_cast<uint32_t>(profile.velocity), &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        if (error != nullptr)
            *error = packetHandler->getTxRxResult(dxl_comm_result);
        return false;
    }
    if (dxl_error != 0)
    {
        if (error != nullptr)
            *error = packetHandler->getRxPacketError(dxl_error);
        return false;
    }
    return true;
}

void applyPositionGains(uint8_t id)
{
    std::map<uint8_t, PositionGains>::const_iterator it = position_gains_by_id.find(id);
    if (it == position_gains_by_id.end())
        return;
    std::string error;
    if (!writePositionGains(id, it->second, &error))
        ROS_ERROR("[ID %d] Failed write Position P/I/D gains: %s", id, error.c_str());
}

void applyPositionProfile(uint8_t id)
{
    PositionProfile profile;
    profile.velocity = 200;
    profile.acceleration = 100;
    std::map<uint8_t, PositionProfile>::const_iterator it = position_profiles_by_id.find(id);
    if (it != position_profiles_by_id.end())
        profile = it->second;

    std::string error;
    if (!writePositionProfile(id, profile, &error))
        ROS_ERROR("[ID %d] Failed write Position Profile: %s", id, error.c_str());
}

bool setOperatingModeInternal(uint8_t id, uint8_t mode)
{
    std::map<uint8_t, uint8_t>::const_iterator mode_it = current_operating_modes.find(id);
    if (mode_it != current_operating_modes.end() && mode_it->second == mode)
        return true;

    std::lock_guard<std::mutex> lock(serial_mutex);
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_ERROR("[ID %d] Failed disable torque before mode set: %s", id, packetHandler->getTxRxResult(dxl_comm_result));

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_OPERATING_MODE, mode, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("[ID %d] Failed set mode: %s", id, packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else
        ROS_INFO("[ID %d] Operating mode set to %d", id, static_cast<int>(mode));

    applyPositionGains(id);
    applyPositionProfile(id);

    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 1, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("[ID %d] Failed enable torque: %s", id, packetHandler->getTxRxResult(dxl_comm_result));
        return false;
    }
    else
        ROS_INFO("[ID %d] Torque enabled", id);

    current_operating_modes[id] = mode;
    return true;
}

void setupDxl(uint8_t id)
{
    const uint8_t mode = multi_turn_ids.count(id) ? 4 : 3;
    default_operating_modes[id] = mode;
    setOperatingModeInternal(id, mode);
}

void setPositionCallback(const dynamixel_control::SetPosition::ConstPtr &msg)
{
    uint8_t id = msg->id;
    if (!containsId(controlled_ids, id))
    {
        // ROS_WARN("ID %d is not in the controlled list", id);
        return;
    }
    std::lock_guard<std::mutex> lock(serial_mutex);
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, id, ADDR_GOAL_POSITION, msg->position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_ERROR("Failed write pos: %s", packetHandler->getTxRxResult(dxl_comm_result));
}

void setBulkPositionsCallback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    if (msg->data.size() != controlled_ids.size())
    {
        ROS_WARN("set_bulk_positions size %zu != controlled_ids size %zu", msg->data.size(), controlled_ids.size());
        return;
    }
    std::lock_guard<std::mutex> lock(serial_mutex);
    for (size_t i = 0; i < controlled_ids.size(); ++i)
    {
        uint8_t id = controlled_ids[i];
        int32_t position = msg->data[i];
        uint8_t param[4];
        param[0] = DXL_LOBYTE(DXL_LOWORD(position));
        param[1] = DXL_HIBYTE(DXL_LOWORD(position));
        param[2] = DXL_LOBYTE(DXL_HIWORD(position));
        param[3] = DXL_HIBYTE(DXL_HIWORD(position));
        if (!group_sync_writer->addParam(id, param))
        {
            ROS_WARN("Failed to add ID %d to GroupSyncWrite", id);
        }
    }
    int tx_result = group_sync_writer->txPacket();
    if (tx_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed bulk write pos: %s", packetHandler->getTxRxResult(tx_result));
    }
    group_sync_writer->clearParam();
}

void setCurrentCallback(const dynamixel_control::SetCurrent::ConstPtr &msg)
{
    uint8_t id = msg->id;
    if (!containsId(controlled_ids, id))
        return;

    std::lock_guard<std::mutex> lock(serial_mutex);
    uint16_t goal_current_raw = static_cast<uint16_t>(static_cast<int16_t>(msg->current));
    dxl_comm_result = packetHandler->write2ByteTxRx(portHandler, id, ADDR_GOAL_CURRENT, goal_current_raw, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
        ROS_ERROR("Failed write current: %s", packetHandler->getTxRxResult(dxl_comm_result));
}

void setOperatingModeCallback(const dynamixel_control::SetOperatingMode::ConstPtr &msg)
{
    uint8_t id = msg->id;
    if (!containsId(controlled_ids, id))
        return;

    uint8_t requested_mode = static_cast<uint8_t>(msg->mode);
    if (requested_mode == 255)
    {
        std::map<uint8_t, uint8_t>::const_iterator default_it = default_operating_modes.find(id);
        if (default_it == default_operating_modes.end())
            return;
        requested_mode = default_it->second;
    }
    setOperatingModeInternal(id, requested_mode);
}

bool getPositionService(dynamixel_control::GetPosition::Request &req,
                        dynamixel_control::GetPosition::Response &res)
{
    uint8_t id = req.id;
    if (!containsId(controlled_ids, id))
    {
        ROS_WARN("ID %d is not in the controlled list", id);
        res.position = -1;
        return true;
    }
    std::lock_guard<std::mutex> lock(serial_mutex);
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, ADDR_PRESENT_POSITION,
                                                   (uint32_t *)&present_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed read pos: %s", packetHandler->getTxRxResult(dxl_comm_result));
    }
    res.position = present_position;
    return true;
}

bool getCurrentService(dynamixel_control::GetCurrent::Request &req,
                       dynamixel_control::GetCurrent::Response &res)
{
    uint8_t id = req.id;
    if (!containsId(controlled_ids, id))
    {
        ROS_WARN("ID %d is not in the controlled list", id);
        res.current = 0;
        return true;
    }
    std::lock_guard<std::mutex> lock(serial_mutex);
    uint16_t present_current_raw = 0;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, ADDR_PRESENT_CURRENT,
                                                   &present_current_raw, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        ROS_ERROR("Failed read current: %s", packetHandler->getTxRxResult(dxl_comm_result));
        res.current = 0;
        return true;
    }

    present_current = static_cast<int16_t>(present_current_raw);
    res.current = static_cast<int32_t>(present_current);
    return true;
}

bool read1Byte(uint8_t id, uint16_t address, uint8_t *value, std::string *error)
{
    uint8_t raw = 0;
    dxl_comm_result = packetHandler->read1ByteTxRx(portHandler, id, address, &raw, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        if (error != nullptr)
            *error = packetHandler->getTxRxResult(dxl_comm_result);
        return false;
    }
    if (dxl_error != 0)
    {
        if (error != nullptr)
            *error = packetHandler->getRxPacketError(dxl_error);
        return false;
    }
    *value = raw;
    return true;
}

bool read2Byte(uint8_t id, uint16_t address, uint16_t *value, std::string *error)
{
    uint16_t raw = 0;
    dxl_comm_result = packetHandler->read2ByteTxRx(portHandler, id, address, &raw, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        if (error != nullptr)
            *error = packetHandler->getTxRxResult(dxl_comm_result);
        return false;
    }
    if (dxl_error != 0)
    {
        if (error != nullptr)
            *error = packetHandler->getRxPacketError(dxl_error);
        return false;
    }
    *value = raw;
    return true;
}

bool read4Byte(uint8_t id, uint16_t address, uint32_t *value, std::string *error)
{
    uint32_t raw = 0;
    dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, id, address, &raw, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        if (error != nullptr)
            *error = packetHandler->getTxRxResult(dxl_comm_result);
        return false;
    }
    if (dxl_error != 0)
    {
        if (error != nullptr)
            *error = packetHandler->getRxPacketError(dxl_error);
        return false;
    }
    *value = raw;
    return true;
}

bool readPositionTuningLocked(
    uint8_t id,
    dynamixel_control::GetPositionTuning::Response *res,
    std::string *error)
{
    uint8_t operating_mode = 0;
    uint16_t pwm_limit = 0;
    uint16_t current_limit = 0;
    uint16_t p_gain = 0;
    uint16_t i_gain = 0;
    uint16_t d_gain = 0;
    uint32_t profile_velocity = 0;
    uint32_t profile_acceleration = 0;

    if (!read1Byte(id, ADDR_OPERATING_MODE, &operating_mode, error) ||
        !read2Byte(id, ADDR_PWM_LIMIT, &pwm_limit, error) ||
        !read2Byte(id, ADDR_CURRENT_LIMIT, &current_limit, error) ||
        !read2Byte(id, ADDR_POSITION_P_GAIN, &p_gain, error) ||
        !read2Byte(id, ADDR_POSITION_I_GAIN, &i_gain, error) ||
        !read2Byte(id, ADDR_POSITION_D_GAIN, &d_gain, error) ||
        !read4Byte(id, ADDR_PROFILE_VELOCITY, &profile_velocity, error) ||
        !read4Byte(id, ADDR_PROFILE_ACCELERATION, &profile_acceleration, error))
        return false;

    res->operating_mode = static_cast<int32_t>(operating_mode);
    res->pwm_limit = static_cast<int32_t>(pwm_limit);
    res->current_limit = static_cast<int32_t>(current_limit);
    res->position_p_gain = static_cast<int32_t>(p_gain);
    res->position_i_gain = static_cast<int32_t>(i_gain);
    res->position_d_gain = static_cast<int32_t>(d_gain);
    res->profile_velocity = static_cast<int32_t>(profile_velocity);
    res->profile_acceleration = static_cast<int32_t>(profile_acceleration);
    return true;
}

bool getPositionTuningService(dynamixel_control::GetPositionTuning::Request &req,
                              dynamixel_control::GetPositionTuning::Response &res)
{
    const uint8_t id = static_cast<uint8_t>(req.id);
    if (req.id < 0 || req.id > 252 || !containsId(controlled_ids, id))
    {
        res.success = false;
        res.message = "motor id is not controlled by this board";
        return true;
    }

    std::lock_guard<std::mutex> lock(serial_mutex);
    std::string error;
    res.success = readPositionTuningLocked(id, &res, &error);
    res.message = res.success ? "ok" : error;
    return true;
}

bool setPositionTuningService(dynamixel_control::SetPositionTuning::Request &req,
                              dynamixel_control::SetPositionTuning::Response &res)
{
    const uint8_t id = static_cast<uint8_t>(req.id);
    if (req.id < 0 || req.id > 252 || !containsId(controlled_ids, id))
    {
        res.success = false;
        res.message = "motor id is not controlled by this board";
        return true;
    }
    if (req.position_p_gain < 0 || req.position_p_gain > 16383 ||
        req.position_i_gain < 0 || req.position_i_gain > 16383 ||
        req.position_d_gain < 0 || req.position_d_gain > 16383 ||
        req.profile_velocity < 0 || req.profile_acceleration < 0)
    {
        res.success = false;
        res.message = "requested tuning values are outside the allowed range";
        return true;
    }

    std::lock_guard<std::mutex> lock(serial_mutex);
    dynamixel_control::GetPositionTuning::Response before;
    std::string error;
    if (!readPositionTuningLocked(id, &before, &error))
    {
        res.success = false;
        res.message = "failed to read current tuning: " + error;
        return true;
    }

    PositionGains requested_gains;
    requested_gains.p = req.position_p_gain;
    requested_gains.i = req.position_i_gain;
    requested_gains.d = req.position_d_gain;
    PositionProfile requested_profile;
    requested_profile.velocity = req.profile_velocity;
    requested_profile.acceleration = req.profile_acceleration;

    if (!writePositionGains(id, requested_gains, &error) ||
        !writePositionProfile(id, requested_profile, &error))
    {
        PositionGains previous_gains;
        previous_gains.p = before.position_p_gain;
        previous_gains.i = before.position_i_gain;
        previous_gains.d = before.position_d_gain;
        PositionProfile previous_profile;
        previous_profile.velocity = before.profile_velocity;
        previous_profile.acceleration = before.profile_acceleration;
        std::string rollback_error;
        const bool rollback_ok = writePositionGains(id, previous_gains, &rollback_error) &&
                                 writePositionProfile(id, previous_profile, &rollback_error);
        res.success = false;
        res.message = "write failed: " + error + (rollback_ok ? "; restored previous values" : "; rollback failed: " + rollback_error);
        return true;
    }

    dynamixel_control::GetPositionTuning::Response after;
    if (!readPositionTuningLocked(id, &after, &error) ||
        after.position_p_gain != req.position_p_gain ||
        after.position_i_gain != req.position_i_gain ||
        after.position_d_gain != req.position_d_gain ||
        after.profile_velocity != req.profile_velocity ||
        after.profile_acceleration != req.profile_acceleration)
    {
        PositionGains previous_gains;
        previous_gains.p = before.position_p_gain;
        previous_gains.i = before.position_i_gain;
        previous_gains.d = before.position_d_gain;
        PositionProfile previous_profile;
        previous_profile.velocity = before.profile_velocity;
        previous_profile.acceleration = before.profile_acceleration;
        std::string rollback_error;
        writePositionGains(id, previous_gains, &rollback_error);
        writePositionProfile(id, previous_profile, &rollback_error);
        res.success = false;
        res.message = "readback did not match the requested tuning";
        return true;
    }

    position_gains_by_id[id] = requested_gains;
    position_profiles_by_id[id] = requested_profile;
    res.success = true;
    res.message = "ok";
    return true;
}

bool getBulkPositionsService(dynamixel_control::GetBulkPositions::Request &req,
                             dynamixel_control::GetBulkPositions::Response &res)
{
    std::lock_guard<std::mutex> lock(serial_mutex);

    if (group_sync_reader == nullptr)
    {
        ROS_ERROR("GroupSyncRead not initialized");
        return false;
    }

    if (req.motor_ids.empty())
    {
        // Use all controlled IDs
        int tx_result = group_sync_reader->txRxPacket();
        if (tx_result != COMM_SUCCESS)
        {
            ROS_ERROR("Failed bulk read: %s", packetHandler->getTxRxResult(tx_result));
            return false;
        }
        for (size_t i = 0; i < controlled_ids.size(); ++i)
        {
            uint8_t id = controlled_ids[i];
            if (group_sync_reader->isAvailable(id, ADDR_PRESENT_POSITION, 4))
            {
                res.positions.push_back(static_cast<int32_t>(group_sync_reader->getData(id, ADDR_PRESENT_POSITION, 4)));
            }
            else
            {
                ROS_WARN("Bulk read data not available for ID %d", id);
                res.positions.push_back(0);
            }
        }
    }
    else
    {
        // Use requested motor IDs
        for (size_t i = 0; i < req.motor_ids.size(); ++i)
        {
            uint8_t id = static_cast<uint8_t>(req.motor_ids[i]);
            if (!group_sync_reader->addParam(id))
            {
                ROS_WARN("Failed to add ID %d to bulk read", id);
            }
        }
        int tx_result = group_sync_reader->txRxPacket();
        if (tx_result != COMM_SUCCESS)
        {
            ROS_ERROR("Failed bulk read: %s", packetHandler->getTxRxResult(tx_result));
            return false;
        }
        for (size_t i = 0; i < req.motor_ids.size(); ++i)
        {
            uint8_t id = static_cast<uint8_t>(req.motor_ids[i]);
            if (group_sync_reader->isAvailable(id, ADDR_PRESENT_POSITION, 4))
            {
                res.positions.push_back(static_cast<int32_t>(group_sync_reader->getData(id, ADDR_PRESENT_POSITION, 4)));
            }
            else
            {
                ROS_WARN("Bulk read data not available for ID %d", id);
                res.positions.push_back(0);
            }
        }
        group_sync_reader->clearParam();
    }
    return true;
}

bool getBulkCurrentsService(dynamixel_control::GetBulkCurrents::Request &req,
                            dynamixel_control::GetBulkCurrents::Response &res)
{
    std::lock_guard<std::mutex> lock(serial_mutex);

    if (current_group_sync_reader == nullptr)
    {
        ROS_ERROR("Current GroupSyncRead not initialized");
        return false;
    }

    if (req.motor_ids.empty())
    {
        int tx_result = current_group_sync_reader->txRxPacket();
        if (tx_result != COMM_SUCCESS)
        {
            ROS_ERROR("Failed bulk current read: %s", packetHandler->getTxRxResult(tx_result));
            return false;
        }
        for (size_t i = 0; i < controlled_ids.size(); ++i)
        {
            uint8_t id = controlled_ids[i];
            if (current_group_sync_reader->isAvailable(id, ADDR_PRESENT_CURRENT, 2))
            {
                uint16_t raw = static_cast<uint16_t>(current_group_sync_reader->getData(id, ADDR_PRESENT_CURRENT, 2));
                res.currents.push_back(static_cast<int32_t>(static_cast<int16_t>(raw)));
            }
            else
            {
                ROS_WARN("Bulk current data not available for ID %d", id);
                res.currents.push_back(0);
            }
        }
    }
    else
    {
        for (size_t i = 0; i < req.motor_ids.size(); ++i)
        {
            uint8_t id = static_cast<uint8_t>(req.motor_ids[i]);
            if (!current_group_sync_reader->addParam(id))
            {
                ROS_WARN("Failed to add ID %d to bulk current read", id);
            }
        }
        int tx_result = current_group_sync_reader->txRxPacket();
        if (tx_result != COMM_SUCCESS)
        {
            ROS_ERROR("Failed bulk current read: %s", packetHandler->getTxRxResult(tx_result));
            return false;
        }
        for (size_t i = 0; i < req.motor_ids.size(); ++i)
        {
            uint8_t id = static_cast<uint8_t>(req.motor_ids[i]);
            if (current_group_sync_reader->isAvailable(id, ADDR_PRESENT_CURRENT, 2))
            {
                uint16_t raw = static_cast<uint16_t>(current_group_sync_reader->getData(id, ADDR_PRESENT_CURRENT, 2));
                res.currents.push_back(static_cast<int32_t>(static_cast<int16_t>(raw)));
            }
            else
            {
                ROS_WARN("Bulk current data not available for ID %d", id);
                res.currents.push_back(0);
            }
        }
        current_group_sync_reader->clearParam();
    }
    return true;
}

void publishTelemetry(const ros::TimerEvent &event)
{
    ros::Time now = ros::Time::now();

    // SyncRead positions
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        int tx_result = group_sync_reader->txRxPacket();
        if (tx_result != COMM_SUCCESS)
        {
            ROS_ERROR_THROTTLE(1.0, "Failed bulk pos read: %s", packetHandler->getTxRxResult(tx_result));
        }
    }

    // SyncRead currents
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        int tx_result = current_group_sync_reader->txRxPacket();
        if (tx_result != COMM_SUCCESS)
        {
            ROS_ERROR_THROTTLE(1.0, "Failed bulk current read: %s", packetHandler->getTxRxResult(tx_result));
        }
    }

    // Build JointState with raw ticks, velocities, currents
    sensor_msgs::JointState js;
    js.header.stamp = now;
    for (auto id : controlled_ids)
    {
        js.name.push_back(std::to_string(static_cast<int>(id)));
        if (group_sync_reader->isAvailable(id, ADDR_PRESENT_POSITION, 4))
        {
            int32_t tick = static_cast<int32_t>(group_sync_reader->getData(id, ADDR_PRESENT_POSITION, 4));
            js.position.push_back(static_cast<double>(tick));

            double dt = last_telemetry_time.isZero() ? 0.001 : (now - last_telemetry_time).toSec();
            double prev_tick = last_tick_positions.count(id) ? last_tick_positions[id] : tick;
            js.velocity.push_back((tick - prev_tick) / (dt > 0.0 ? dt : 0.001));
            last_tick_positions[id] = tick;
        }
        else
        {
            js.position.push_back(0);
            js.velocity.push_back(0);
        }

        if (current_group_sync_reader->isAvailable(id, ADDR_PRESENT_CURRENT, 2))
        {
            uint16_t raw = static_cast<uint16_t>(current_group_sync_reader->getData(id, ADDR_PRESENT_CURRENT, 2));
            js.effort.push_back(static_cast<double>(static_cast<int16_t>(raw)));
        }
        else
        {
            js.effort.push_back(0);
        }
    }
    last_telemetry_time = now;

    joint_state_pub.publish(js);

    // Publish raw currents separately
    std_msgs::Float32MultiArray currents_msg;
    for (auto id : controlled_ids)
    {
        if (current_group_sync_reader->isAvailable(id, ADDR_PRESENT_CURRENT, 2))
        {
            uint16_t raw = static_cast<uint16_t>(current_group_sync_reader->getData(id, ADDR_PRESENT_CURRENT, 2));
            currents_msg.data.push_back(static_cast<float>(static_cast<int16_t>(raw)));
        }
        else
        {
            currents_msg.data.push_back(0.0f);
        }
    }
    joint_currents_pub.publish(currents_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "multi_dxl_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");
    ros::NodeHandle ns_nh;
    std::string board_name;

    pnh.param<std::string>("board_name", board_name, std::string(""));

    const std::string board_prefix = board_name.empty() ? std::string("") : std::string("/") + board_name + std::string("/");

    std::string device_name = DEVICE_NAME;
    int baudrate = BAUDRATE;
    pnh.param<std::string>("device_name", device_name, std::string(""));
    if (device_name.empty())
    {
        if (!board_prefix.empty())
            ros::param::param<std::string>(board_prefix + "device_name", device_name, std::string(DEVICE_NAME));
        else
            ns_nh.param<std::string>("device_name", device_name, std::string(DEVICE_NAME));
    }

    pnh.param("baudrate", baudrate, -1);
    if (baudrate < 0)
    {
        if (!board_prefix.empty())
            ros::param::param(board_prefix + "baudrate", baudrate, BAUDRATE);
        else
            ns_nh.param("baudrate", baudrate, BAUDRATE);
    }

    std::vector<int> default_motor_ids;
    default_motor_ids.push_back(11);
    default_motor_ids.push_back(1);
    default_motor_ids.push_back(2);
    default_motor_ids.push_back(12);
    default_motor_ids.push_back(3);
    default_motor_ids.push_back(4);
    default_motor_ids.push_back(13);
    default_motor_ids.push_back(5);
    default_motor_ids.push_back(6);
    default_motor_ids.push_back(14);
    default_motor_ids.push_back(7);
    default_motor_ids.push_back(8);

    std::vector<int> default_multi_turn_ids;
    for (int i = 1; i <= 8; ++i)
        default_multi_turn_ids.push_back(i);

    std::vector<int> motor_ids_param;
    std::vector<int> multi_turn_ids_param;
    pnh.param("motor_ids", motor_ids_param, std::vector<int>());
    if (motor_ids_param.empty())
    {
        if (!board_prefix.empty())
            ros::param::param(board_prefix + "motor_ids", motor_ids_param, default_motor_ids);
        else
            ns_nh.param("motor_ids", motor_ids_param, default_motor_ids);
    }

    pnh.param("multi_turn_ids", multi_turn_ids_param, std::vector<int>());
    if (multi_turn_ids_param.empty())
    {
        if (!board_prefix.empty())
            ros::param::param(board_prefix + "multi_turn_ids", multi_turn_ids_param, default_multi_turn_ids);
        else
            ns_nh.param("multi_turn_ids", multi_turn_ids_param, default_multi_turn_ids);
    }

    controlled_ids = intListToUint8(motor_ids_param);
    std::vector<uint8_t> multi_turn_list = intListToUint8(multi_turn_ids_param);
    multi_turn_ids = std::set<uint8_t>(multi_turn_list.begin(), multi_turn_list.end());
    loadPositionGains(board_prefix);
    loadPositionProfiles(board_prefix);

    if (controlled_ids.empty())
    {
        ROS_ERROR("No valid motor IDs configured.");
        return 1;
    }

    ROS_INFO("Configured %zu motors", controlled_ids.size());

    portHandler = PortHandler::getPortHandler(device_name.c_str());
    packetHandler = PacketHandler::getPacketHandler(PROTOCOL_VERSION);

    if (!portHandler->openPort())
    {
        ROS_ERROR("Failed to open port %s", device_name.c_str());
        return 1;
    }
    if (!portHandler->setBaudRate(baudrate))
    {
        ROS_ERROR("Failed to set baudrate %d", baudrate);
        return 1;
    }

    // initialize each motor
    for (auto id : controlled_ids)
        setupDxl(id);

    // Initialize GroupSyncRead for bulk position reads (4 bytes = 32-bit position)
    group_sync_reader = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_POSITION, 4);
    for (auto id : controlled_ids)
    {
        if (!group_sync_reader->addParam(id))
        {
            ROS_WARN("Failed to add ID %d to GroupSyncRead", id);
        }
    }

    // Initialize GroupSyncRead for bulk current reads (2 bytes = 16-bit current)
    current_group_sync_reader = new dynamixel::GroupSyncRead(portHandler, packetHandler, ADDR_PRESENT_CURRENT, 2);
    for (auto id : controlled_ids)
    {
        if (!current_group_sync_reader->addParam(id))
        {
            ROS_WARN("Failed to add ID %d to current GroupSyncRead", id);
        }
    }

    // Initialize GroupSyncWrite for bulk position writes (4 bytes = 32-bit position)
    group_sync_writer = new dynamixel::GroupSyncWrite(portHandler, packetHandler, ADDR_GOAL_POSITION, 4);

    ros::Subscriber sub = nh.subscribe("set_position", 100, setPositionCallback);
    ros::Subscriber bulk_sub = nh.subscribe("set_bulk_positions", 100, setBulkPositionsCallback);
    ros::Subscriber current_sub = nh.subscribe("set_current", 100, setCurrentCallback);
    ros::Subscriber mode_sub = nh.subscribe("set_operating_mode", 100, setOperatingModeCallback);
    ros::ServiceServer srv = nh.advertiseService("get_position", getPositionService);
    ros::ServiceServer current_srv = nh.advertiseService("get_current", getCurrentService);
    ros::ServiceServer bulk_srv = nh.advertiseService("get_bulk_positions", getBulkPositionsService);
    ros::ServiceServer bulk_current_srv = nh.advertiseService("get_bulk_currents", getBulkCurrentsService);
    ros::ServiceServer get_tuning_srv = nh.advertiseService("get_position_tuning", getPositionTuningService);
    ros::ServiceServer set_tuning_srv = nh.advertiseService("set_position_tuning", setPositionTuningService);

    // Direct telemetry publisher (in C++, bypassing ROS service round-trips)
    pnh.param("telemetry_rate_hz", telemetry_rate_hz, 50);
    joint_state_pub = nh.advertise<sensor_msgs::JointState>("joint_state", 20);
    joint_currents_pub = nh.advertise<std_msgs::Float32MultiArray>("joint_currents", 20);
    ros::Timer telemetry_timer = nh.createTimer(ros::Duration(1.0 / telemetry_rate_hz), publishTelemetry);

    ros::AsyncSpinner spinner(2);  // 2 threads: one for serial writes, one for service reads
    spinner.start();
    ros::waitForShutdown();

    // disable torque on exit
    {
        std::lock_guard<std::mutex> lock(serial_mutex);
        for (auto id : controlled_ids)
        {
            packetHandler->write1ByteTxRx(portHandler, id, ADDR_TORQUE_ENABLE, 0, &dxl_error);
        }
    }
    portHandler->closePort();
    delete group_sync_reader;
    delete current_group_sync_reader;
    delete group_sync_writer;

    return 0;
}
