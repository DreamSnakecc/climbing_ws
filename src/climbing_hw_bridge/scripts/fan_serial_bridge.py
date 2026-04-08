#!/usr/bin/env python

import struct

import rospy
from climbing_msgs.msg import AdhesionCommand
from climbing_msgs.srv import SetFanSpeed, SetFanSpeedResponse
from std_msgs.msg import Float32, Float32MultiArray

try:
    import serial
except ImportError:
    serial = None


class FanSerialBridge(object):
    def __init__(self):
        rospy.init_node("fan_serial_bridge", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/fan_controller/" + name, default))

        self.port_name = get_cfg("port", "/dev/ttyUSB_fan")
        self.baudrate = int(get_cfg("baudrate", 115200))
        self.float_endianness = get_cfg("float_endianness", ">")
        self.protocol = str(get_cfg("protocol", "total_rpm")).lower()
        self.leg_count = int(get_cfg("leg_count", 4))
        self.telemetry_rate_hz = float(get_cfg("telemetry_rate_hz", 50.0))
        self.total_rpm_header = bytes([0x01, 0x02])
        self.indexed_leg_header = bytes([0x02, 0x03])
        self.current_frame_header = bytes(self._parse_byte_list(get_cfg("current_frame_header", [0x03, 0x04])))
        self.current_payload_format = self.float_endianness + str(get_cfg("current_payload_format", "4f"))
        self.command_leg_order = [str(name) for name in get_cfg("command_leg_order", ["lf", "rf", "rr", "lr"])]
        self.msg_leg_order = [str(name) for name in get_cfg("msg_leg_order", ["lf", "rf", "lr", "rr"])]
        self.current_leg_order = [str(name) for name in get_cfg("current_leg_order", list(self.msg_leg_order))]
        self.current_scale = float(get_cfg("current_scale", 1.0))
        self.command_fan_ids = dict((leg_name, index + 1) for index, leg_name in enumerate(self.command_leg_order))
        self.msg_leg_index_map = dict((leg_name, index) for index, leg_name in enumerate(self.msg_leg_order))
        self.rx_index_to_leg_name = dict((index, leg_name) for leg_name, index in self.msg_leg_index_map.items())
        self.current_leg_index_map = dict((leg_name, index) for index, leg_name in enumerate(self.current_leg_order))
        self.port = None
        self.rx_buffer = bytearray()
        self.leg_targets = dict(
            (leg_name, {"mode": 0, "target_rpm": 0.0, "normal_force_limit": 0.0, "required_adhesion_force": 0.0})
            for leg_name in self.command_leg_order
        )
        self.last_total_payload = None
        self.last_fan_currents = dict((leg_name, 0.0) for leg_name in self.msg_leg_order)
        self.current_payload_size = struct.calcsize(self.current_payload_format)
        self.current_packet_size = len(self.current_frame_header) + self.current_payload_size

        self.echo_pub = rospy.Publisher("~/last_command", Float32, queue_size=10)
        self.leg_echo_pub = rospy.Publisher("~/leg_rpm", Float32MultiArray, queue_size=10)
        self.current_pub = rospy.Publisher("~/fan_currents", Float32MultiArray, queue_size=10)
        rospy.Subscriber("~/adhesion_command", AdhesionCommand, self.adhesion_callback, queue_size=20)
        rospy.Service("~/set_fan_speed_once", SetFanSpeed, self.handle_set_fan_speed)

        if serial is None:
            rospy.logwarn("pyserial is not installed; fan_serial_bridge will not access serial hardware")
        else:
            self.try_open_port()
        self.telemetry_timer = rospy.Timer(rospy.Duration(1.0 / max(self.telemetry_rate_hz, 1.0)), self.update_telemetry)

    @staticmethod
    def _parse_byte_list(values):
        parsed = []
        for value in values:
            if isinstance(value, str):
                parsed.append(int(value, 0))
            else:
                parsed.append(int(value))
        return parsed

    def try_open_port(self):
        try:
            self.port = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
            rospy.loginfo("Opened fan serial port %s", self.port_name)
        except Exception as exc:
            self.port = None
            rospy.logerr("Failed to open fan serial port %s: %s", self.port_name, exc)

    def _pack_total_rpm_command(self):
        ordered_targets = [float(self.leg_targets[leg_name]["target_rpm"]) for leg_name in self.command_leg_order]
        return self.total_rpm_header + struct.pack(self.float_endianness + "4f", *ordered_targets)

    def _pack_indexed_leg_command(self, fan_id, target_rpm):
        payload = struct.pack(self.float_endianness + "Bf", int(fan_id), float(target_rpm))
        return self.indexed_leg_header + payload

    def _send_bytes(self, payload, log_value):
        if self.port is None:
            rospy.logwarn_throttle(2.0, "fan serial port is not available; dropping command %.3f", log_value)
            return False, "serial port unavailable"
        try:
            self.port.write(payload)
            self.echo_pub.publish(Float32(data=float(log_value)))
            return True, "command sent"
        except Exception as exc:
            rospy.logerr("Failed to send fan command: %s", exc)
            return False, str(exc)

    def _publish_leg_echo(self):
        msg = Float32MultiArray()
        msg.data = [float(self.leg_targets[leg_name]["target_rpm"]) for leg_name in self.command_leg_order]
        self.leg_echo_pub.publish(msg)

    def _publish_fan_currents(self):
        msg = Float32MultiArray()
        msg.data = [float(self.last_fan_currents.get(leg_name, 0.0)) for leg_name in self.msg_leg_order]
        self.current_pub.publish(msg)

    def _bytes_waiting(self):
        if self.port is None:
            return 0
        if hasattr(self.port, "in_waiting"):
            return int(self.port.in_waiting)
        return int(self.port.inWaiting())

    def update_telemetry(self, _event):
        if self.port is None:
            return
        try:
            available = self._bytes_waiting()
            if available <= 0:
                return
            chunk = self.port.read(max(1, available))
            if not chunk:
                return
            self.rx_buffer.extend(bytearray(chunk))
            while True:
                currents = self._extract_current_frame()
                if currents is None:
                    break
                for leg_name, value in currents.items():
                    self.last_fan_currents[leg_name] = float(value)
                self._publish_fan_currents()
        except Exception as exc:
            rospy.logwarn_throttle(2.0, "fan serial telemetry read failed: %s", exc)

    def _extract_current_frame(self):
        while True:
            if len(self.rx_buffer) < len(self.current_frame_header):
                return None
            start = self.rx_buffer.find(self.current_frame_header)
            if start < 0:
                keep = max(len(self.current_frame_header) - 1, 0)
                self.rx_buffer = self.rx_buffer[-keep:] if keep > 0 else bytearray()
                return None
            if start > 0:
                del self.rx_buffer[:start]
            if len(self.rx_buffer) < self.current_packet_size:
                return None
            payload_start = len(self.current_frame_header)
            payload_end = payload_start + self.current_payload_size
            payload = bytes(self.rx_buffer[payload_start:payload_end])
            try:
                unpacked = struct.unpack(self.current_payload_format, payload)
            except struct.error:
                del self.rx_buffer[0]
                continue
            del self.rx_buffer[:self.current_packet_size]
            currents = {}
            for index, leg_name in enumerate(self.current_leg_order):
                if index >= len(unpacked):
                    break
                currents[leg_name] = float(unpacked[index]) * self.current_scale
            return currents

    def send_rpm(self, target_rpm):
        for leg_name in self.command_leg_order:
            self.leg_targets[leg_name]["target_rpm"] = float(target_rpm)
            self.leg_targets[leg_name]["mode"] = 1 if float(target_rpm) > 0.0 else 0
        payload = self._pack_total_rpm_command()
        accepted, message = self._send_bytes(payload, target_rpm)
        if accepted:
            self.last_total_payload = payload
        return accepted, message

    def send_leg_command(self, fan_id, target_rpm):
        payload = self._pack_indexed_leg_command(fan_id, target_rpm)
        return self._send_bytes(payload, target_rpm)

    def adhesion_callback(self, msg):
        leg_index = int(msg.leg_index)
        if leg_index not in self.rx_index_to_leg_name:
            rospy.logwarn_throttle(2.0, "fan_serial_bridge received invalid leg index %d", leg_index)
            return

        leg_name = self.rx_index_to_leg_name[leg_index]
        if leg_name not in self.leg_targets:
            rospy.logwarn_throttle(2.0, "fan_serial_bridge leg %s is not present in command order", leg_name)
            return

        self.leg_targets[leg_name] = {
            "mode": int(msg.mode),
            "target_rpm": float(msg.target_rpm),
            "normal_force_limit": float(msg.normal_force_limit),
            "required_adhesion_force": float(msg.required_adhesion_force),
        }
        self._publish_leg_echo()

        if self.protocol == "indexed_leg_rpm":
            fan_id = self.command_fan_ids[leg_name]
            self.send_leg_command(fan_id, msg.target_rpm)
            return

        payload = self._pack_total_rpm_command()
        if payload != self.last_total_payload:
            accepted, _message = self._send_bytes(payload, msg.target_rpm)
            if accepted:
                self.last_total_payload = payload

    def handle_set_fan_speed(self, req):
        accepted, message = self.send_rpm(req.target_rpm)
        return SetFanSpeedResponse(accepted=accepted, message=message)


if __name__ == "__main__":
    FanSerialBridge()
    rospy.spin()