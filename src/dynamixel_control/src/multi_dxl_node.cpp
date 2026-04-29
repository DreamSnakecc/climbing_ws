#include <ros/ros.h>
#include <std_msgs/Int32.h>
#include <dynamixel_sdk/dynamixel_sdk.h>
#include <dynamixel_sdk/group_sync_read.h>
#include "dynamixel_control/GetCurrent.h"
#include "dynamixel_control/SetCurrent.h"
#include "dynamixel_control/SetOperatingMode.h"
#include "dynamixel_control/SetPosition.h"
#include "dynamixel_control/GetPosition.h"
#include "dynamixel_control/GetBulkPositions.h"
#include <algorithm>
#include <map>
#include <set>
#include <vector>
#include <mutex>

// Control table address for X series (except XL-320)
#define ADDR_OPERATING_MODE 11
#define ADDR_TORQUE_ENABLE 64
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

std::vector<uint8_t> controlled_ids;
std::set<uint8_t> multi_turn_ids;
std::map<uint8_t, uint8_t> default_operating_modes;
std::map<uint8_t, uint8_t> current_operating_modes;

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

    // Set smooth motion profile: velocity=200 (~24rpm), acceleration=50
    // Non-zero values enable internal trajectory interpolation, making leg
    // motion smooth despite irregular command arrival or serial bus contention.
    packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_ACCELERATION, 50, &dxl_error);
    packetHandler->write4ByteTxRx(portHandler, id, ADDR_PROFILE_VELOCITY, 200, &dxl_error);

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

    ros::Subscriber sub = nh.subscribe("set_position", 100, setPositionCallback);
    ros::Subscriber current_sub = nh.subscribe("set_current", 100, setCurrentCallback);
    ros::Subscriber mode_sub = nh.subscribe("set_operating_mode", 100, setOperatingModeCallback);
    ros::ServiceServer srv = nh.advertiseService("get_position", getPositionService);
    ros::ServiceServer current_srv = nh.advertiseService("get_current", getCurrentService);
    ros::ServiceServer bulk_srv = nh.advertiseService("get_bulk_positions", getBulkPositionsService);

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

    return 0;
}
