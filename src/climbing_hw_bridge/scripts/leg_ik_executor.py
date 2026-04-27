#!/usr/bin/env python

import math
import os

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState

from climbing_msgs.msg import LegCenterCommand
from dynamixel_control.srv import GetPosition


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
        self.robot_config_path = rospy.get_param("~robot_config_path", "")
        self.command_pub = rospy.Publisher(self.command_topic, JointState, queue_size=20)
        self.enable_auto_position_commands = bool(rospy.get_param("~enable_auto_position_commands", True))
        self.rate_hz = float(rospy.get_param("~rate_hz", rospy.get_param("/jetson/command_rate_hz", 100.0)))
        self.input_mode = rospy.get_param("~input_mode", "absolute_center_m")  # or "delta_from_nominal_m"
        self.startup_pose_mode = str(get_cfg("startup_pose_mode", "use_motor_home_ticks"))

        self.nominal_x = float(get_cfg("nominal_x", 118.75))
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
        self.operating_universal_joint_center_x = float(
            get_cfg("operating_universal_joint_center_x", self.nominal_x)
        )
        self.operating_universal_joint_center_y = float(
            get_cfg("operating_universal_joint_center_y", self.nominal_y)
        )
        self.operating_universal_joint_center_z = float(
            get_cfg(
                "operating_universal_joint_center_z",
                self.nominal_universal_joint_center_z,
            )
        )
        self.startup_target_enabled = bool(get_cfg("startup_target_enabled", False))
        self.startup_target_x = float(
            get_cfg("startup_target_x", self.operating_universal_joint_center_x)
        )
        self.startup_target_y = float(
            get_cfg("startup_target_y", self.operating_universal_joint_center_y)
        )
        self.startup_target_universal_joint_center_z = float(
            get_cfg(
                "startup_target_universal_joint_center_z",
                self.operating_universal_joint_center_z,
            )
        )
        self.startup_target_hold_s = max(float(get_cfg("startup_target_hold_s", 0.0)), 0.0)
        self.startup_target_move_duration_s = max(
            float(get_cfg("startup_target_move_duration_s", 0.0)),
            0.0,
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
        self.leg_order = ["lf", "rf", "rr", "lr"]
        self.legs = {}
        self.motor_index = []
        for name in self.leg_order:
            leg = LegKinematics(name, leg_cfg[name]["motor_ids"], leg_cfg[name]["hip_yaw_deg"])
            self.legs[name] = leg
            self.motor_index.extend(leg.motor_ids)

        self.board_configs = self._load_board_configs()
        self.position_clients = {}
        self._init_position_service_clients()
        self.nominal_joint_deg = self._nominal_joint_deg_vector()
        self.nominal_output_deg = [
            self.nominal_joint_deg[0] + float(self.joint_zero_deg["j1"]),
            self.nominal_joint_deg[1] + float(self.joint_zero_deg["j2"]),
            self.nominal_joint_deg[2] + float(self.joint_zero_deg["j3"]),
        ]
        self.last_joint_deg_by_leg = {name: list(self.nominal_joint_deg) for name in self.leg_order}
        self._initialize_startup_pose_mode()
        self.startup_time_sec = rospy.Time.now().to_sec()
        self.external_command_received = False
        self.startup_move_started = False
        self.startup_move_completed = False

        self.targets_m = {
            name: self._operating_center_command(name)
            for name in self.leg_order
        }
        self.last_ticks = self.compute_all_ticks()

        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self.command_callback, queue_size=50)
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.publish_ticks)
        rospy.loginfo("leg_ik_executor started with real IK/tick mapping from quadruped_core formulas")

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    @staticmethod
    def _smoothstep5(phase):
        phase = max(0.0, min(1.0, float(phase)))
        return phase * phase * phase * (10.0 + phase * (-15.0 + 6.0 * phase))

    def _load_board_configs(self):
        board_configs = {}
        for board_name in ["left_board", "right_board"]:
            board_param = rospy.get_param("/" + board_name, None)
            if isinstance(board_param, dict):
                board_configs[board_name] = {"motor_ids": [int(value) for value in board_param.get("motor_ids", [])]}
        return board_configs

    def _qualify_service(self, board_name, service_basename):
        namespace = rospy.get_namespace().rstrip("/")
        base_name = service_basename.strip("/")
        if namespace:
            return namespace + "/" + board_name + "/" + base_name
        return "/" + board_name + "/" + base_name

    def _init_position_service_clients(self):
        for board_name in sorted(self.board_configs.keys()):
            service_name = self._qualify_service(board_name, "get_position")
            rospy.wait_for_service(service_name)
            self.position_clients[board_name] = rospy.ServiceProxy(service_name, GetPosition)

    def _board_name_for_motor(self, motor_id):
        for board_name, board_cfg in self.board_configs.items():
            if int(motor_id) in board_cfg.get("motor_ids", []):
                return board_name
        return None

    def _read_motor_position(self, motor_id):
        board_name = self._board_name_for_motor(motor_id)
        if board_name is None:
            return None
        try:
            return int(self.position_clients[board_name](int(motor_id)).position)
        except rospy.ServiceException as exc:
            rospy.logwarn("leg_ik_executor failed to read startup position for ID %d: %s", motor_id, exc)
            return None

    def _nominal_joint_deg_vector(self):
        q1_deg, q2_deg, q3_deg = self._ik_transform_matrix_solve(
            self.nominal_x,
            self.nominal_y,
            self.nominal_universal_joint_center_z,
        )
        return [q1_deg, q2_deg, q3_deg]

    def _capture_motor_home_from_current_pose(self):
        captured_home = {}
        for motor_id in self.motor_index:
            raw_position = self._read_motor_position(motor_id)
            if raw_position is None:
                return None
            if self._is_position_mode(motor_id):
                raw_position %= 4096
            captured_home[str(int(motor_id))] = int(raw_position)
        return captured_home

    def _persist_motor_home_ticks(self):
        rospy.set_param("/gait_controller/motor_home_ticks", self.motor_home)
        if not self.robot_config_path:
            rospy.logwarn("leg_ik_executor has no robot_config_path; captured motor_home_ticks were not written to disk")
            return
        if not os.path.isfile(self.robot_config_path):
            rospy.logwarn("leg_ik_executor could not find robot config file: %s", self.robot_config_path)
            return

        with open(self.robot_config_path, "r") as stream:
            lines = stream.readlines()

        start_index = None
        end_index = None
        for index, line in enumerate(lines):
            if line.strip() == "motor_home_ticks:":
                start_index = index
                continue
            if start_index is not None and line.startswith("  ") and not line.startswith("    "):
                end_index = index
                break
        if start_index is None:
            rospy.logwarn("leg_ik_executor could not locate motor_home_ticks block in %s", self.robot_config_path)
            return
        if end_index is None:
            end_index = len(lines)

        replacement = ["  motor_home_ticks:\n"]
        for motor_id in sorted([int(key) for key in self.motor_home.keys()]):
            replacement.append("    \"%d\": %d\n" % (motor_id, int(self.motor_home[str(motor_id)])))
        lines[start_index:end_index] = replacement

        with open(self.robot_config_path, "w") as stream:
            stream.writelines(lines)
        rospy.loginfo("leg_ik_executor updated motor_home_ticks in %s", self.robot_config_path)

    def _initialize_startup_pose_mode(self):
        if self.startup_pose_mode == "capture_current_as_home":
            captured_home = self._capture_motor_home_from_current_pose()
            if captured_home is None:
                rospy.logwarn("leg_ik_executor could not capture startup motor positions; falling back to configured motor_home_ticks")
                return
            self.motor_home = captured_home
            self._persist_motor_home_ticks()
            rospy.loginfo("leg_ik_executor captured current motor positions as motor_home_ticks")
            return
        if self.startup_pose_mode != "use_motor_home_ticks":
            rospy.logwarn("Unknown startup_pose_mode '%s'; defaulting to use_motor_home_ticks", self.startup_pose_mode)
            self.startup_pose_mode = "use_motor_home_ticks"

    def _base_delta_to_leg_delta(self, dx_base_mm, dy_base_mm, leg):
        cos_yaw = math.cos(leg.hip_yaw)
        sin_yaw = math.sin(leg.hip_yaw)
        dx_leg = cos_yaw * dx_base_mm + sin_yaw * dy_base_mm
        dy_leg = -sin_yaw * dx_base_mm + cos_yaw * dy_base_mm
        return dx_leg, dy_leg

    def _leg_delta_to_base_delta(self, dx_leg_mm, dy_leg_mm, leg):
        cos_yaw = math.cos(leg.hip_yaw)
        sin_yaw = math.sin(leg.hip_yaw)
        dx_base = cos_yaw * dx_leg_mm - sin_yaw * dy_leg_mm
        dy_base = sin_yaw * dx_leg_mm + cos_yaw * dy_leg_mm
        return dx_base, dy_base

    def _operating_center_command(self, leg_name):
        """LegCenterCommand.center that maps to the operating UJC in leg frame 0."""
        leg = self.legs[leg_name]
        dx_leg_mm = self.operating_universal_joint_center_x - self.nominal_x
        dy_leg_mm = self.operating_universal_joint_center_y - self.nominal_y
        dx_base_mm, dy_base_mm = self._leg_delta_to_base_delta(dx_leg_mm, dy_leg_mm, leg)
        return Point(
            dx_base_mm / 1000.0,
            dy_base_mm / 1000.0,
            self.operating_universal_joint_center_z / 1000.0,
        )

    def _clamp_joint_solution_deg(self, joint_solution_deg):
        return [
            self._clamp(float(joint_solution_deg[0]), self.joint_limit_deg["j1"][0], self.joint_limit_deg["j1"][1]),
            self._clamp(float(joint_solution_deg[1]), self.joint_limit_deg["j2"][0], self.joint_limit_deg["j2"][1]),
            self._clamp(float(joint_solution_deg[2]), self.joint_limit_deg["j3"][0], self.joint_limit_deg["j3"][1]),
        ]

    def _ik_solution_cost(self, candidate_deg, reference_deg):
        return sum((float(candidate_deg[index]) - float(reference_deg[index])) ** 2 for index in [0, 1, 2])

    def _ik_candidates_deg(self, x_mm, y_mm, z_mm):
        x_prime = float(x_mm) - self.l_coxa
        p_z = float(z_mm)

        q1 = math.atan2(float(y_mm), x_prime)
        r_total = math.hypot(x_prime, float(y_mm))
        r_prime = max(r_total - self.l_femur, 1e-6)

        l1 = self.l_tibia
        l2 = self.l_a3
        d_sq = r_prime ** 2 + p_z ** 2
        cos_theta3 = (d_sq - l1 ** 2 - l2 ** 2) / (2.0 * l1 * l2)
        cos_theta3 = self._clamp(cos_theta3, -1.0, 1.0)
        sin_theta3_mag = math.sqrt(max(0.0, 1.0 - cos_theta3 ** 2))

        candidates = []
        for branch_sign in [-1.0, 1.0]:
            theta3 = math.atan2(branch_sign * sin_theta3_mag, cos_theta3)
            q2 = math.atan2(p_z, r_prime) - math.atan2(l2 * math.sin(theta3), l1 + l2 * math.cos(theta3)) + math.radians(90.0)
            candidates.append([math.degrees(q1), math.degrees(q2), math.degrees(theta3)])
        return candidates

    def _ik_transform_matrix_solve(self, x_mm, y_mm, z_mm, reference_deg=None):
        candidates = [self._clamp_joint_solution_deg(candidate) for candidate in self._ik_candidates_deg(x_mm, y_mm, z_mm)]
        if reference_deg is None:
            reference_deg = [0.0, 0.0, 0.0]
        return tuple(min(candidates, key=lambda candidate: self._ik_solution_cost(candidate, reference_deg)))

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
        nominal_output_deg = float(self.nominal_output_deg[joint_index])

        if int(motor_id) in self.direct_drive_ids:
            ratio = 1.0
        if self._is_position_mode(motor_id):
            home = home % 4096.0

        ticks = home + direction * ((output_deg - nominal_output_deg) * ratio) * unit
        return self._normalize_target_for_mode(motor_id, ticks)

    def _compute_leg_ticks_from_leg_frame_mm(self, leg_name, tx_mm, ty_mm, tz_mm):
        leg = self.legs[leg_name]
        reference_deg = self.last_joint_deg_by_leg.get(leg_name, self.nominal_joint_deg)
        q1_deg, q2_deg, q3_deg = self._ik_transform_matrix_solve(tx_mm, ty_mm, tz_mm, reference_deg)
        self.last_joint_deg_by_leg[leg_name] = [q1_deg, q2_deg, q3_deg]

        cmd_deg = [
            q1_deg + float(self.joint_zero_deg["j1"]),
            q2_deg + float(self.joint_zero_deg["j2"]),
            q3_deg + float(self.joint_zero_deg["j3"]),
        ]
        ticks = []
        for joint_index, motor_id in enumerate(leg.motor_ids):
            ticks.append(self._output_deg_to_ticks(motor_id, cmd_deg[joint_index], joint_index))
        return ticks

    def _target_to_leg_deltas_mm(self, point_m):
        if self.input_mode == "delta_from_nominal_m":
            return point_m.x * 1000.0, point_m.y * 1000.0, point_m.z * 1000.0
        return point_m.x * 1000.0, point_m.y * 1000.0, point_m.z * 1000.0 - self.nominal_universal_joint_center_z

    def compute_leg_ticks(self, leg_name, point_m):
        dx_base_mm, dy_base_mm, dz_delta_base_mm = self._target_to_leg_deltas_mm(point_m)
        leg = self.legs[leg_name]
        dx_leg_mm, dy_leg_mm = self._base_delta_to_leg_delta(dx_base_mm, dy_base_mm, leg)

        tx_mm = self.nominal_x + dx_leg_mm
        ty_mm = self.nominal_y + dy_leg_mm
        tz_mm = self.nominal_universal_joint_center_z + dz_delta_base_mm
        return self._compute_leg_ticks_from_leg_frame_mm(leg_name, tx_mm, ty_mm, tz_mm)

    def compute_all_ticks(self):
        ordered_ticks = []
        for leg_name in self.leg_order:
            ordered_ticks.extend(self.compute_leg_ticks(leg_name, self.targets_m[leg_name]))
        return ordered_ticks

    def _startup_move_progress(self, now_sec):
        if self.external_command_received:
            return None
        if not self.startup_target_enabled:
            return 1.0  # stay at nominal -- treated the same as "move complete"
        elapsed = max(0.0, float(now_sec) - self.startup_time_sec)
        if elapsed <= self.startup_target_hold_s:
            return 0.0
        if self.startup_target_move_duration_s <= 1e-6:
            return 1.0
        return self._smoothstep5(
            (elapsed - self.startup_target_hold_s) / self.startup_target_move_duration_s
        )

    def _compute_startup_move_ticks(self, now_sec):
        progress = self._startup_move_progress(now_sec)
        if progress is None:
            return None
        if self.startup_target_enabled and progress > 0.0 and not self.startup_move_started:
            rospy.loginfo(
                "leg_ik_executor startup move: nominal UJC -> target (%.2f, %.2f, %.2f) mm in leg frame 0",
                self.startup_target_x,
                self.startup_target_y,
                self.startup_target_universal_joint_center_z,
            )
            self.startup_move_started = True
        if not self.startup_target_enabled and not self.startup_move_completed:
            rospy.loginfo("leg_ik_executor startup: staying at nominal UJC (startup_target_enabled=false)")
            self.startup_move_completed = True
        if self.startup_target_enabled and progress >= 1.0 and not self.startup_move_completed:
            rospy.loginfo("leg_ik_executor startup move completed")
            self.startup_move_completed = True

        if self.startup_target_enabled:
            tx_mm = self.nominal_x + progress * (self.startup_target_x - self.nominal_x)
            ty_mm = self.nominal_y + progress * (self.startup_target_y - self.nominal_y)
            tz_mm = self.nominal_universal_joint_center_z + progress * (
                self.startup_target_universal_joint_center_z - self.nominal_universal_joint_center_z
            )
        else:
            tx_mm = self.nominal_x
            ty_mm = self.nominal_y
            tz_mm = self.nominal_universal_joint_center_z

        ordered_ticks = []
        for leg_name in self.leg_order:
            ordered_ticks.extend(
                self._compute_leg_ticks_from_leg_frame_mm(leg_name, tx_mm, ty_mm, tz_mm)
            )
        return ordered_ticks

    def command_callback(self, msg):
        if msg.leg_name not in self.targets_m:
            rospy.logwarn_throttle(2.0, "Unknown leg name for IK executor: %s", msg.leg_name)
            return
        self.external_command_received = True
        self.targets_m[msg.leg_name] = Point(msg.center.x, msg.center.y, msg.center.z)
        self.last_ticks = self.compute_all_ticks()

    def publish_ticks(self, _event):
        if not self.enable_auto_position_commands:
            return
        startup_ticks = self._compute_startup_move_ticks(rospy.Time.now().to_sec())
        if startup_ticks is not None:
            self.last_ticks = startup_ticks
        ticks_msg = JointState()
        ticks_msg.header.stamp = rospy.Time.now()
        ticks_msg.name = [str(motor_id) for motor_id in self.motor_index]
        ticks_msg.position = [float(tick) for tick in self.last_ticks]
        self.command_pub.publish(ticks_msg)


if __name__ == "__main__":
    LegIkExecutor()
    rospy.spin()