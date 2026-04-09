#!/usr/bin/env python

import binascii
import math
import struct

import rospy
from sensor_msgs.msg import Imu

try:
    import serial
except ImportError:
    serial = None


FDILINK_FRAME_HEAD = 0xFC
FDILINK_FRAME_END = 0xFD
FDILINK_TYPE_IMU = 0x40
FDILINK_TYPE_AHRS = 0x41
FDILINK_TYPE_INSGPS = 0x42
FDILINK_TYPE_GEODETIC_POS = 0x5C
FDILINK_TYPE_GROUND = 0xF0

FDILINK_IMU_LEN = 0x38
FDILINK_AHRS_LEN = 0x30
FDILINK_INSGPS_LEN = 0x48
FDILINK_GEODETIC_POS_LEN = 0x20

CRC8_TABLE = [
    0, 94, 188, 226, 97, 63, 221, 131, 194, 156, 126, 32, 163, 253, 31, 65,
    157, 195, 33, 127, 252, 162, 64, 30, 95, 1, 227, 189, 62, 96, 130, 220,
    35, 125, 159, 193, 66, 28, 254, 160, 225, 191, 93, 3, 128, 222, 60, 98,
    190, 224, 2, 92, 223, 129, 99, 61, 124, 34, 192, 158, 29, 67, 161, 255,
    70, 24, 250, 164, 39, 121, 155, 197, 132, 218, 56, 102, 229, 187, 89, 7,
    219, 133, 103, 57, 186, 228, 6, 88, 25, 71, 165, 251, 120, 38, 196, 154,
    101, 59, 217, 135, 4, 90, 184, 230, 167, 249, 27, 69, 198, 152, 122, 36,
    248, 166, 68, 26, 153, 199, 37, 123, 58, 100, 134, 216, 91, 5, 231, 185,
    140, 210, 48, 110, 237, 179, 81, 15, 78, 16, 242, 172, 47, 113, 147, 205,
    17, 79, 173, 243, 112, 46, 204, 146, 211, 141, 111, 49, 178, 236, 14, 80,
    175, 241, 19, 77, 206, 144, 114, 44, 109, 51, 209, 143, 12, 82, 176, 238,
    50, 108, 142, 208, 83, 13, 239, 177, 240, 174, 76, 18, 145, 207, 45, 115,
    202, 148, 118, 40, 171, 245, 23, 73, 8, 86, 180, 234, 105, 55, 213, 139,
    87, 9, 235, 181, 54, 104, 138, 212, 149, 203, 41, 119, 244, 170, 72, 22,
    233, 183, 85, 11, 136, 214, 52, 106, 43, 117, 151, 201, 74, 20, 246, 168,
    116, 42, 200, 150, 21, 75, 169, 247, 182, 232, 10, 84, 215, 137, 107, 53,
]


def quaternion_from_euler(roll, pitch, yaw):
    half_roll = 0.5 * roll
    half_pitch = 0.5 * pitch
    half_yaw = 0.5 * yaw

    cr = math.cos(half_roll)
    sr = math.sin(half_roll)
    cp = math.cos(half_pitch)
    sp = math.sin(half_pitch)
    cy = math.cos(half_yaw)
    sy = math.sin(half_yaw)

    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    qw = cr * cp * cy + sr * sp * sy
    return qx, qy, qz, qw


def quaternion_from_axis_angle(axis_x, axis_y, axis_z, angle):
    half_angle = 0.5 * angle
    sin_half = math.sin(half_angle)
    return axis_x * sin_half, axis_y * sin_half, axis_z * sin_half, math.cos(half_angle)


def quaternion_multiply(lhs, rhs):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return (
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    )


def normalize_quaternion(quaternion):
    qx, qy, qz, qw = quaternion
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if norm <= 1e-9:
        return 0.0, 0.0, 0.0, 1.0
    return qx / norm, qy / norm, qz / norm, qw / norm


def quaternion_conjugate(quaternion):
    qx, qy, qz, qw = quaternion
    return -qx, -qy, -qz, qw


def rotate_vector(quaternion, vector):
    normalized = normalize_quaternion(quaternion)
    pure = (float(vector[0]), float(vector[1]), float(vector[2]), 0.0)
    rotated = quaternion_multiply(quaternion_multiply(normalized, pure), quaternion_conjugate(normalized))
    return rotated[:3]


def crc8_fdilink(data):
    crc8 = 0
    for value in data:
        crc8 = CRC8_TABLE[crc8 ^ int(value)]
    return crc8


class ImuSerialBridge(object):
    def __init__(self):
        rospy.init_node("imu_serial_bridge", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/imu/" + name, default))

        self.frame_id = str(get_cfg("frame_id", "imu_link"))
        self.publish_rate_hz = float(get_cfg("publish_rate_hz", 100.0))
        self.use_mock_data = bool(get_cfg("use_mock_data", False))
        self.protocol = str(get_cfg("protocol", "generic")).lower()

        self.port_name = str(get_cfg("port", "/dev/ttyUSB_imu"))
        self.baudrate = int(get_cfg("baudrate", 115200))
        self.timeout_s = float(get_cfg("timeout_s", 0.02))
        self.read_chunk_size = int(get_cfg("read_chunk_size", 512))
        self.reconnect_interval_s = float(get_cfg("reconnect_interval_s", 1.0))

        self.device_type = int(get_cfg("device_type", 1))
        self.fdilink_packet_lengths = {
            FDILINK_TYPE_IMU: FDILINK_IMU_LEN,
            FDILINK_TYPE_AHRS: FDILINK_AHRS_LEN,
            FDILINK_TYPE_INSGPS: FDILINK_INSGPS_LEN,
            FDILINK_TYPE_GEODETIC_POS: FDILINK_GEODETIC_POS_LEN,
        }
        self.latest_imu_packet = None
        self.latest_ahrs_packet = None
        opposite_direction_parallel_q = quaternion_from_axis_angle(math.sqrt(0.5), -math.sqrt(0.5), 0.0, math.pi)
        self.fdilink_orientation_pre = self._parse_quaternion(
            get_cfg(
                "fdilink_orientation_pre_xyzw",
                opposite_direction_parallel_q,
            ),
            opposite_direction_parallel_q,
        )
        self.fdilink_orientation_post = self._parse_quaternion(
            get_cfg("fdilink_orientation_post_xyzw", opposite_direction_parallel_q),
            opposite_direction_parallel_q,
        )
        self.sensor_vector_to_body = self._parse_quaternion(
            get_cfg("sensor_vector_to_body_xyzw", opposite_direction_parallel_q),
            opposite_direction_parallel_q,
        )

        self.frame_header = bytearray(self._parse_byte_list(get_cfg("frame_header", [])))
        self.payload_format = str(get_cfg("payload_format", "<9f"))
        self.field_names = [str(value) for value in get_cfg(
            "fields",
            ["roll_deg", "pitch_deg", "yaw_deg", "ax", "ay", "az", "gx", "gy", "gz"],
        )]
        self.field_scale = get_cfg("field_scale", {})
        self.checksum_type = str(get_cfg("checksum_type", "none")).lower()
        self.checksum_include_header = bool(get_cfg("checksum_include_header", False))

        self.orientation_source = str(get_cfg("orientation_source", "euler_deg")).lower()
        self.orientation_fields = [str(value) for value in get_cfg(
            "orientation_fields",
            self._default_orientation_fields(self.orientation_source),
        )]
        self.angular_velocity_fields = [str(value) for value in get_cfg(
            "angular_velocity_fields",
            ["gx", "gy", "gz"],
        )]
        self.linear_acceleration_fields = [str(value) for value in get_cfg(
            "linear_acceleration_fields",
            ["ax", "ay", "az"],
        )]
        self.angular_velocity_unit = str(get_cfg("angular_velocity_unit", "radps")).lower()
        self.linear_acceleration_unit = str(get_cfg("linear_acceleration_unit", "mps2")).lower()

        self.orientation_covariance = self._diag_covariance(
            get_cfg("orientation_covariance_diagonal", [0.02, 0.02, 0.04])
        )
        self.angular_velocity_covariance = self._diag_covariance(
            get_cfg("angular_velocity_covariance_diagonal", [0.01, 0.01, 0.01])
        )
        self.linear_acceleration_covariance = self._diag_covariance(
            get_cfg("linear_acceleration_covariance_diagonal", [0.04, 0.04, 0.04])
        )

        self.payload_size = struct.calcsize(self.payload_format)
        unpacked_length = len(struct.unpack(self.payload_format, bytearray(self.payload_size)))
        self.field_names = self._normalize_field_names(self.field_names, unpacked_length)
        self.checksum_size = 0 if self.checksum_type == "none" else 1
        self.packet_size = len(self.frame_header) + self.payload_size + self.checksum_size

        self.port = None
        self.buffer = bytearray()
        self.last_message = None
        self.last_open_attempt = rospy.Time(0)

        self.pub = rospy.Publisher("~/imu", Imu, queue_size=50)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.publish_rate_hz, 1.0)), self.update)

        if self.use_mock_data:
            rospy.logwarn("imu_serial_bridge is running in mock mode")
        elif serial is None:
            rospy.logwarn("pyserial is not installed; imu_serial_bridge cannot access serial hardware")
        else:
            self.try_open_port(force=True)

    @staticmethod
    def _default_orientation_fields(orientation_source):
        if orientation_source == "quaternion":
            return ["qx", "qy", "qz", "qw"]
        return ["roll_deg", "pitch_deg", "yaw_deg"]

    @staticmethod
    def _normalize_field_names(field_names, expected_length):
        normalized = list(field_names[:expected_length])
        while len(normalized) < expected_length:
            normalized.append("field_%d" % len(normalized))
        return normalized

    @staticmethod
    def _diag_covariance(diagonal):
        values = [float(value) for value in diagonal]
        if len(values) != 3:
            values = [0.0, 0.0, 0.0]
        covariance = [0.0] * 9
        covariance[0] = values[0]
        covariance[4] = values[1]
        covariance[8] = values[2]
        return covariance

    @staticmethod
    def _parse_byte_list(values):
        parsed = []
        for value in values:
            if isinstance(value, str):
                parsed.append(int(value, 0))
            else:
                parsed.append(int(value))
        return parsed

    @staticmethod
    def _covariance_unknown():
        covariance = [0.0] * 9
        covariance[0] = -1.0
        return covariance

    @staticmethod
    def _parse_quaternion(values, fallback):
        try:
            parsed = tuple([float(value) for value in values])
        except (TypeError, ValueError):
            parsed = tuple(fallback)
        if len(parsed) != 4:
            parsed = tuple(fallback)
        return normalize_quaternion(parsed)

    def try_open_port(self, force=False):
        if self.use_mock_data or serial is None:
            return

        now = rospy.Time.now()
        if not force and (now - self.last_open_attempt).to_sec() < self.reconnect_interval_s:
            return
        self.last_open_attempt = now

        try:
            self.port = serial.Serial(self.port_name, self.baudrate, timeout=self.timeout_s)
            self.buffer = bytearray()
            rospy.loginfo("Opened IMU serial port %s at %d baud", self.port_name, self.baudrate)
        except Exception as exc:
            self.port = None
            rospy.logwarn_throttle(2.0, "Failed to open IMU serial port %s: %s", self.port_name, exc)

    def close_port(self):
        if self.port is not None:
            try:
                self.port.close()
            except Exception:
                pass
        self.port = None

    def update(self, _event):
        if self.use_mock_data:
            self.pub.publish(self.build_mock_message())
            return

        if self.port is None:
            self.try_open_port()
        if self.port is not None:
            if self.protocol == "fdilink":
                self.read_fdilink_frames()
            else:
                self.read_generic_serial_frames()

        if self.last_message is not None:
            self.pub.publish(self.last_message)

    def read_generic_serial_frames(self):
        try:
            available = self._bytes_waiting()
            if available <= 0:
                return

            chunk = self.port.read(max(1, min(available, self.read_chunk_size)))
            if not chunk:
                return

            self.buffer.extend(bytearray(chunk))
            while True:
                frame = self.extract_generic_measurement()
                if frame is None:
                    break
                self.last_message = self.build_imu_message(frame)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "IMU serial read failed: %s", exc)
            self.close_port()

    def read_fdilink_frames(self):
        try:
            available = self._bytes_waiting()
            if available <= 0:
                return

            chunk = self.port.read(max(1, min(available, self.read_chunk_size)))
            if not chunk:
                return

            self.buffer.extend(bytearray(chunk))
            while True:
                packet = self.extract_fdilink_packet()
                if packet is None:
                    break
                self.handle_fdilink_packet(packet)
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "FDILink IMU serial read failed: %s", exc)
            self.close_port()

    def _bytes_waiting(self):
        if hasattr(self.port, "in_waiting"):
            return int(self.port.in_waiting)
        return int(self.port.inWaiting())

    def extract_generic_measurement(self):
        while True:
            if self.frame_header:
                if len(self.buffer) < len(self.frame_header):
                    return None

                start = self.buffer.find(self.frame_header)
                if start < 0:
                    keep = max(len(self.frame_header) - 1, 0)
                    if keep > 0:
                        self.buffer = self.buffer[-keep:]
                    else:
                        self.buffer = bytearray()
                    return None

                if start > 0:
                    del self.buffer[:start]

            if len(self.buffer) < self.packet_size:
                return None

            payload_start = len(self.frame_header)
            payload_end = payload_start + self.payload_size
            payload = self.buffer[payload_start:payload_end]
            checksum = self.buffer[payload_end:payload_end + self.checksum_size]

            if not self._checksum_valid(payload, checksum):
                del self.buffer[0]
                continue

            try:
                unpacked = struct.unpack(self.payload_format, payload)
            except struct.error:
                del self.buffer[0]
                continue

            del self.buffer[:self.packet_size]
            return self._map_fields(unpacked)

    def extract_fdilink_packet(self):
        while True:
            if len(self.buffer) < 1:
                return None

            start = self.buffer.find(bytearray([FDILINK_FRAME_HEAD]))
            if start < 0:
                self.buffer = bytearray()
                return None
            if start > 0:
                del self.buffer[:start]

            if len(self.buffer) < 7:
                return None

            packet_type = int(self.buffer[1])
            packet_length = int(self.buffer[2])

            if packet_type in [FDILINK_TYPE_GROUND, 0x50, 0xFF]:
                del self.buffer[0]
                continue

            expected_length = self.fdilink_packet_lengths.get(packet_type)
            if expected_length is None or packet_length != expected_length:
                del self.buffer[0]
                continue

            header_crc8 = int(self.buffer[4])
            if header_crc8 != crc8_fdilink(self.buffer[:4]):
                del self.buffer[0]
                continue

            packet_size = 7 + packet_length + 1
            if len(self.buffer) < packet_size:
                return None

            payload = bytes(self.buffer[7:7 + packet_length])
            expected_crc16 = (int(self.buffer[5]) << 8) | int(self.buffer[6])
            if expected_crc16 != binascii.crc_hqx(payload, 0):
                del self.buffer[0]
                continue

            frame_end = int(self.buffer[7 + packet_length])
            if frame_end != FDILINK_FRAME_END:
                del self.buffer[0]
                continue

            packet = {
                "type": packet_type,
                "length": packet_length,
                "serial_num": int(self.buffer[3]),
                "payload": payload,
            }
            del self.buffer[:packet_size]
            return packet

    def handle_fdilink_packet(self, packet):
        packet_type = packet["type"]
        if packet_type == FDILINK_TYPE_AHRS:
            self.latest_ahrs_packet = self.parse_fdilink_ahrs(packet["payload"])
        elif packet_type == FDILINK_TYPE_IMU:
            self.latest_imu_packet = self.parse_fdilink_imu(packet["payload"])
            self.last_message = self.build_fdilink_imu_message(self.latest_imu_packet, self.latest_ahrs_packet)

    @staticmethod
    def parse_fdilink_imu(payload):
        values = struct.unpack("<12fq", payload)
        return {
            "gx": float(values[0]),
            "gy": float(values[1]),
            "gz": float(values[2]),
            "ax": float(values[3]),
            "ay": float(values[4]),
            "az": float(values[5]),
            "mx": float(values[6]),
            "my": float(values[7]),
            "mz": float(values[8]),
            "temperature_c": float(values[9]),
            "pressure_pa": float(values[10]),
            "pressure_temperature_c": float(values[11]),
            "timestamp_us": int(values[12]),
        }

    @staticmethod
    def parse_fdilink_ahrs(payload):
        values = struct.unpack("<10fq", payload)
        return {
            "roll_speed": float(values[0]),
            "pitch_speed": float(values[1]),
            "heading_speed": float(values[2]),
            "roll": float(values[3]),
            "pitch": float(values[4]),
            "heading": float(values[5]),
            "qw": float(values[6]),
            "qx": float(values[7]),
            "qy": float(values[8]),
            "qz": float(values[9]),
            "timestamp_us": int(values[10]),
        }

    def transform_fdilink_quaternion(self, raw_quaternion):
        transformed = quaternion_multiply(
            quaternion_multiply(self.fdilink_orientation_pre, raw_quaternion),
            self.fdilink_orientation_post,
        )
        return normalize_quaternion(transformed)

    def transform_sensor_vector(self, vector):
        return rotate_vector(self.sensor_vector_to_body, vector)

    def build_fdilink_imu_message(self, imu_packet, ahrs_packet):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        msg.orientation_covariance = list(self.orientation_covariance)
        msg.angular_velocity_covariance = list(self.angular_velocity_covariance)
        msg.linear_acceleration_covariance = list(self.linear_acceleration_covariance)

        if imu_packet is None:
            msg.orientation.w = 1.0
            msg.orientation_covariance = self._covariance_unknown()
            msg.angular_velocity_covariance = self._covariance_unknown()
            msg.linear_acceleration_covariance = self._covariance_unknown()
            return msg

        if ahrs_packet is None:
            msg.orientation.w = 1.0
            msg.orientation_covariance = self._covariance_unknown()
        else:
            raw_quaternion = normalize_quaternion(
                (ahrs_packet["qx"], ahrs_packet["qy"], ahrs_packet["qz"], ahrs_packet["qw"])
            )
            if self.device_type == 1:
                qx, qy, qz, qw = self.transform_fdilink_quaternion(raw_quaternion)
            else:
                qx, qy, qz, qw = raw_quaternion
            msg.orientation.x = qx
            msg.orientation.y = qy
            msg.orientation.z = qz
            msg.orientation.w = qw

        if self.device_type == 1:
            angular_velocity = self.transform_sensor_vector((imu_packet["gx"], imu_packet["gy"], imu_packet["gz"]))
            linear_acceleration = self.transform_sensor_vector((imu_packet["ax"], imu_packet["ay"], imu_packet["az"]))
            msg.angular_velocity.x = angular_velocity[0]
            msg.angular_velocity.y = angular_velocity[1]
            msg.angular_velocity.z = angular_velocity[2]
            msg.linear_acceleration.x = linear_acceleration[0]
            msg.linear_acceleration.y = linear_acceleration[1]
            msg.linear_acceleration.z = linear_acceleration[2]
        else:
            msg.angular_velocity.x = imu_packet["gx"]
            msg.angular_velocity.y = imu_packet["gy"]
            msg.angular_velocity.z = imu_packet["gz"]
            msg.linear_acceleration.x = imu_packet["ax"]
            msg.linear_acceleration.y = imu_packet["ay"]
            msg.linear_acceleration.z = imu_packet["az"]
        return msg

    def _checksum_valid(self, payload, checksum_bytes):
        if self.checksum_type == "none":
            return True

        if len(checksum_bytes) != 1:
            return False

        data = bytearray()
        if self.checksum_include_header:
            data.extend(self.frame_header)
        data.extend(payload)

        if self.checksum_type == "xor8":
            expected = 0
            for value in data:
                expected ^= int(value)
        elif self.checksum_type == "sum8":
            expected = sum([int(value) for value in data]) & 0xFF
        else:
            rospy.logwarn_throttle(2.0, "Unsupported IMU checksum_type: %s", self.checksum_type)
            return False

        return expected == int(checksum_bytes[0])

    def _map_fields(self, unpacked_values):
        mapped = {}
        for name, value in zip(self.field_names, unpacked_values):
            scale = float(self.field_scale.get(name, 1.0))
            mapped[name] = float(value) * scale
        return mapped

    def _extract_vector(self, frame, field_names):
        if len(field_names) != 3:
            return None
        if not all([name in frame for name in field_names]):
            return None
        return [float(frame[field_names[0]]), float(frame[field_names[1]]), float(frame[field_names[2]])]

    def _normalize_angular_velocity(self, vector):
        if vector is None:
            return None
        if self.angular_velocity_unit == "degps":
            return [math.radians(value) for value in vector]
        return vector

    def _normalize_linear_acceleration(self, vector):
        if vector is None:
            return None
        if self.linear_acceleration_unit == "g":
            return [value * 9.80665 for value in vector]
        return vector

    def build_imu_message(self, frame):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id

        if self.orientation_source == "quaternion":
            if all([name in frame for name in self.orientation_fields[:4]]):
                quaternion = normalize_quaternion((
                    float(frame[self.orientation_fields[0]]),
                    float(frame[self.orientation_fields[1]]),
                    float(frame[self.orientation_fields[2]]),
                    float(frame[self.orientation_fields[3]]),
                ))
                msg.orientation.x = quaternion[0]
                msg.orientation.y = quaternion[1]
                msg.orientation.z = quaternion[2]
                msg.orientation.w = quaternion[3]
                msg.orientation_covariance = list(self.orientation_covariance)
            else:
                msg.orientation.w = 1.0
                msg.orientation_covariance = self._covariance_unknown()
        elif self.orientation_source in ["euler_deg", "euler_rad"]:
            euler = self._extract_vector(frame, self.orientation_fields)
            if euler is None:
                msg.orientation.w = 1.0
                msg.orientation_covariance = self._covariance_unknown()
            else:
                if self.orientation_source == "euler_deg":
                    euler = [math.radians(value) for value in euler]
                qx, qy, qz, qw = normalize_quaternion(quaternion_from_euler(euler[0], euler[1], euler[2]))
                msg.orientation.x = qx
                msg.orientation.y = qy
                msg.orientation.z = qz
                msg.orientation.w = qw
                msg.orientation_covariance = list(self.orientation_covariance)
        else:
            msg.orientation.w = 1.0
            msg.orientation_covariance = self._covariance_unknown()

        angular_velocity = self._normalize_angular_velocity(
            self._extract_vector(frame, self.angular_velocity_fields)
        )
        if angular_velocity is None:
            msg.angular_velocity_covariance = self._covariance_unknown()
        else:
            angular_velocity = self.transform_sensor_vector(angular_velocity)
            msg.angular_velocity.x = angular_velocity[0]
            msg.angular_velocity.y = angular_velocity[1]
            msg.angular_velocity.z = angular_velocity[2]
            msg.angular_velocity_covariance = list(self.angular_velocity_covariance)

        linear_acceleration = self._normalize_linear_acceleration(
            self._extract_vector(frame, self.linear_acceleration_fields)
        )
        if linear_acceleration is None:
            msg.linear_acceleration_covariance = self._covariance_unknown()
        else:
            linear_acceleration = self.transform_sensor_vector(linear_acceleration)
            msg.linear_acceleration.x = linear_acceleration[0]
            msg.linear_acceleration.y = linear_acceleration[1]
            msg.linear_acceleration.z = linear_acceleration[2]
            msg.linear_acceleration_covariance = list(self.linear_acceleration_covariance)
        return msg

    def build_mock_message(self):
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.orientation.w = 1.0
        msg.orientation_covariance = self._covariance_unknown()
        msg.angular_velocity_covariance = self._covariance_unknown()
        msg.linear_acceleration_covariance = self._covariance_unknown()
        return msg


if __name__ == "__main__":
    ImuSerialBridge()
    rospy.spin()