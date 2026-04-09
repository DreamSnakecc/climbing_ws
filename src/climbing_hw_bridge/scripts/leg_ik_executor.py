#!/usr/bin/env python

import math

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from climbing_msgs.msg import LegCenterCommand


class LegKinematics(object):
    def __init__(self, name, motor_ids, hip_yaw_deg):
        self.name = name
        self.motor_ids = [int(v) for v in motor_ids]
        self.hip_yaw = math.radians(float(hip_yaw_deg))


class LegIkExecutor(object):
    def __init__(self):
        rospy.init_node("leg_ik_executor", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/gait_controller/" + name, default))

        self.command_topic = rospy.get_param("~command_topic", "/jetson/joint_position_ticks_cmd")
        self.command_pub = rospy.Publisher(self.command_topic, JointState, queue_size=20)
        self.rate_hz = float(rospy.get_param("~rate_hz", rospy.get_param("/jetson/command_rate_hz", 100.0)))
        self.input_mode = rospy.get_param("~input_mode", "absolute_center_m")  # or "delta_from_nominal_m"

        self.nominal_x = float(get_cfg("nominal_x", 120.0))
        self.nominal_y = float(get_cfg("nominal_y", 0.0))
        self.legacy_nominal_z = float(get_cfg("nominal_z", -299.2))

        self.l_coxa = float(get_cfg("link_coxa", 44.75))
        self.l_femur = float(get_cfg("link_femur", 74.0))
        self.l_tibia = float(get_cfg("link_tibia", 150.0))
        self.l_a3 = float(get_cfg("link_a3", 41.5))
        self.l_d6 = float(get_cfg("link_d6", -13.5))
        self.l_d7 = float(get_cfg("link_d7", -106.7))
        self.nominal_universal_joint_center_z = float(
            get_cfg("nominal_universal_joint_center_z", self.legacy_nominal_z + abs(self.l_d6 + self.l_d7))
        )
        self.gear_ratio_joint2 = float(get_cfg("gear_ratio_joint2", 1.0))
        self.gear_ratio_joint3 = float(get_cfg("gear_ratio_joint3", 4.421))
        self.joint_limit_deg = get_cfg(
            "joint_limit_deg",
            {"j1": [-90.0, 90.0], "j2": [-10.0, 190.0], "j3": [-100.0, 100.0]},
        )
        self.joint_zero_deg = get_cfg(
            "joint_zero_deg",
            {"j1": 0.0, "j2": 90.0, "j3": 0.0},
        )
        self.motor_home = get_cfg("motor_home_ticks", {})
        self.motor_dir = get_cfg("motor_dir", {})
        self.position_mode_ids = set([int(v) for v in get_cfg("position_mode_ids", [])])
        self.direct_drive_ids = set([int(v) for v in get_cfg("single_turn_ids", [])])

        default_leg_cfg = {
            "lf": {"motor_ids": [11, 1, 2], "hip_yaw_deg": 45.0},
            "rf": {"motor_ids": [12, 3, 4], "hip_yaw_deg": -45.0},
            "lr": {"motor_ids": [14, 7, 8], "hip_yaw_deg": 135.0},
            "rr": {"motor_ids": [13, 5, 6], "hip_yaw_deg": -135.0},
        }
        leg_cfg = get_cfg("legs", default_leg_cfg)
        self.leg_order = ["lf", "rf", "lr", "rr"]
        self.legs = {}
        self.motor_index = []
        for name in self.leg_order:
            leg = LegKinematics(name, leg_cfg[name]["motor_ids"], leg_cfg[name]["hip_yaw_deg"])
            self.legs[name] = leg
            self.motor_index.extend(leg.motor_ids)

        self.targets_m = {name: Point(0.0, 0.0, self.nominal_universal_joint_center_z / 1000.0) for name in self.leg_order}
        self.last_ticks = self.compute_all_ticks()

        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self.command_callback, queue_size=50)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.publish_ticks)
        rospy.loginfo("leg_ik_executor started with real IK/tick mapping from quadruped_core formulas")

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    def _base_delta_to_leg_delta(self, dx_base_mm, dy_base_mm, leg):
        cos_yaw = math.cos(leg.hip_yaw)
        sin_yaw = math.sin(leg.hip_yaw)
        dx_leg = cos_yaw * dx_base_mm + sin_yaw * dy_base_mm
        dy_leg = -sin_yaw * dx_base_mm + cos_yaw * dy_base_mm
        return dx_leg, dy_leg

    def _ik_transform_matrix_solve(self, x_mm, y_mm, z_mm):
        p_z = z_mm

        q1 = math.atan2(y_mm, x_mm)
        r_total = math.hypot(x_mm, y_mm)
        r_prime = max(r_total - self.l_femur, 1.0)

        l1 = self.l_tibia
        l2 = self.l_a3
        d_sq = r_prime ** 2 + p_z ** 2
        cos_theta3 = (d_sq - l1 ** 2 - l2 ** 2) / (2.0 * l1 * l2)
        cos_theta3 = self._clamp(cos_theta3, -1.0, 1.0)
        theta3 = math.atan2(-math.sqrt(max(0.0, 1.0 - cos_theta3 ** 2)), cos_theta3)

        q3 = theta3
        q2 = math.atan2(p_z, r_prime) - math.atan2(l2 * math.sin(theta3), l1 + l2 * math.cos(theta3)) + math.radians(90.0)

        q1_deg = self._clamp(math.degrees(q1), self.joint_limit_deg["j1"][0], self.joint_limit_deg["j1"][1])
        q2_deg = self._clamp(math.degrees(q2), self.joint_limit_deg["j2"][0], self.joint_limit_deg["j2"][1])
        q3_deg = self._clamp(math.degrees(q3), self.joint_limit_deg["j3"][0], self.joint_limit_deg["j3"][1])
        return q1_deg, q2_deg, q3_deg

    def _joint_gear_ratio(self, joint_index):
        if joint_index == 1:
            return self.gear_ratio_joint2
        if joint_index == 2:
            return self.gear_ratio_joint3
        return 1.0

    def _is_position_mode(self, motor_id):
        return int(motor_id) in self.position_mode_ids

    def _normalize_target_for_mode(self, motor_id, target_ticks):
        if self._is_position_mode(motor_id):
            normalized = int(round(float(target_ticks))) % 4096
            return self._clamp(normalized, 0, 4095)
        return self._clamp(int(round(float(target_ticks))), -1048575, 1048575)

    def _output_deg_to_ticks(self, motor_id, output_deg, joint_index):
        unit = 4096.0 / 360.0
        key = str(int(motor_id))
        home = float(self.motor_home.get(key, 0.0))
        direction = float(self.motor_dir.get(key, 1.0))
        ratio = self._joint_gear_ratio(joint_index)

        if int(motor_id) in self.direct_drive_ids:
            ratio = 1.0
        if self._is_position_mode(motor_id):
            home = home % 4096.0

        ticks = home + direction * (output_deg * ratio) * unit
        return self._normalize_target_for_mode(motor_id, ticks)

    def _target_to_leg_deltas_mm(self, point_m):
        if self.input_mode == "delta_from_nominal_m":
            return point_m.x * 1000.0, point_m.y * 1000.0, point_m.z * 1000.0
        return point_m.x * 1000.0, point_m.y * 1000.0, point_m.z * 1000.0 - self.nominal_universal_joint_center_z

    def compute_leg_ticks(self, leg_name, point_m):
        leg = self.legs[leg_name]
        dx_base_mm, dy_base_mm, dz_delta_base_mm = self._target_to_leg_deltas_mm(point_m)
        dx_leg_mm, dy_leg_mm = self._base_delta_to_leg_delta(dx_base_mm, dy_base_mm, leg)

        tx_mm = self.nominal_x + dx_leg_mm
        ty_mm = self.nominal_y + dy_leg_mm
        tz_mm = self.nominal_universal_joint_center_z + dz_delta_base_mm
        q1_deg, q2_deg, q3_deg = self._ik_transform_matrix_solve(tx_mm, ty_mm, tz_mm)

        cmd_deg = [
            q1_deg + float(self.joint_zero_deg["j1"]),
            q2_deg + float(self.joint_zero_deg["j2"]),
            q3_deg + float(self.joint_zero_deg["j3"]),
        ]
        ticks = []
        for joint_index, motor_id in enumerate(leg.motor_ids):
            ticks.append(self._output_deg_to_ticks(motor_id, cmd_deg[joint_index], joint_index))
        return ticks

    def compute_all_ticks(self):
        ordered_ticks = []
        for leg_name in self.leg_order:
            ordered_ticks.extend(self.compute_leg_ticks(leg_name, self.targets_m[leg_name]))
        return ordered_ticks

    def command_callback(self, msg):
        if msg.leg_name not in self.targets_m:
            rospy.logwarn_throttle(2.0, "Unknown leg name for IK executor: %s", msg.leg_name)
            return
        self.targets_m[msg.leg_name] = Point(msg.center.x, msg.center.y, msg.center.z)
        self.last_ticks = self.compute_all_ticks()

    def publish_ticks(self, _event):
        ticks_msg = JointState()
        ticks_msg.header.stamp = rospy.Time.now()
        ticks_msg.name = [str(motor_id) for motor_id in self.motor_index]
        ticks_msg.position = [float(tick) for tick in self.last_ticks]
        self.command_pub.publish(ticks_msg)


if __name__ == "__main__":
    LegIkExecutor()
    rospy.spin()