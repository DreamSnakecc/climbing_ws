#!/usr/bin/env python

import math

import rospy
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray

from climbing_msgs.msg import LegCenterCommand, StanceWrenchCommand
from dynamixel_control.msg import SetCurrent, SetOperatingMode, SetPosition
from dynamixel_control.srv import GetCurrent, GetPosition, GetBulkPositions, GetBulkCurrents
from std_msgs.msg import Int32MultiArray, MultiArrayLayout, MultiArrayDimension


class DynamixelBridge(object):
    def __init__(self):
        rospy.init_node("dynamixel_bridge", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/dynamixel_telemetry/" + name, default))

        def get_gait_cfg(name, default):
            return rospy.get_param("/gait_controller/" + name, default)

        self.motor_ids = [int(value) for value in rospy.get_param("~ordered_motor_ids", [])]
        self.command_topic = rospy.get_param("~command_topic", "/jetson/joint_position_ticks_cmd")
        self.enable_auto_mode_switching = bool(rospy.get_param("~enable_auto_mode_switching", True))
        self.enable_auto_current_control = bool(rospy.get_param("~enable_auto_current_control", True))
        self.telemetry_rate_hz = float(get_cfg("telemetry_rate_hz", 20.0))
        self.current_command_rate_hz = float(get_cfg("current_command_rate_hz", 50.0))
        self.position_service_name = str(get_cfg("position_service", "/get_position"))
        self.current_service_name = str(get_cfg("current_service", "/get_current"))
        self.current_lsb_ma = float(get_cfg("current_lsb_ma", 2.69))
        self.support_operating_mode = int(get_cfg("support_operating_mode", 5))
        self.default_operating_mode_marker = int(get_cfg("default_operating_mode_marker", 255))
        self.jacobian_delta_rad = float(get_cfg("jacobian_delta_rad", 1e-3))
        self.support_current_margin_a = float(get_cfg("support_current_margin_a", 0.08))
        self.max_goal_current_a = float(get_cfg("max_goal_current_a", 4.8))
        self.min_goal_current_a = float(get_cfg("min_goal_current_a", 0.0))
        self.joint_current_scale = [float(value) for value in get_cfg("joint_current_scale", [0.35, 1.0, 1.0])]
        self.torque_constant_nm_per_a = get_cfg("torque_constant_nm_per_a", {"default": 1.0})
        self.torque_bias_nm = get_cfg("torque_bias_nm", {"default": 0.0})
        self.torque_models = get_cfg("torque_models", {})
        self.torque_model_by_motor = get_cfg("torque_model_by_motor", {})
        self.runtime_namespace = rospy.get_namespace().rstrip("/")

        # Diagnostic counters
        self.command_recv_count = 0
        self.position_pub_count = 0
        self.last_diag_log_time = rospy.Time.now()
        self.board_configs = self._load_board_configs()

        self.gear_ratio_joint2 = float(get_gait_cfg("gear_ratio_joint2", 1.0))
        self.gear_ratio_joint3 = float(get_gait_cfg("gear_ratio_joint3", 4.421))
        self.nominal_x = float(get_gait_cfg("nominal_x", 118.75))
        self.nominal_y = float(get_gait_cfg("nominal_y", 0.0))
        self.legacy_nominal_z = float(get_gait_cfg("nominal_z", -299.2))
        self.l_coxa = float(get_gait_cfg("link_coxa", 44.75)) / 1000.0
        self.l_femur = float(get_gait_cfg("link_femur", 74.0)) / 1000.0
        self.l_tibia = float(get_gait_cfg("link_tibia", 150.0)) / 1000.0
        self.l_a3 = float(get_gait_cfg("link_a3", 41.5)) / 1000.0
        self.l_d6 = float(get_gait_cfg("link_d6", -13.5)) / 1000.0
        self.l_d7 = float(get_gait_cfg("link_d7", -106.7)) / 1000.0
        self.joint_zero_deg = get_gait_cfg("joint_zero_deg", {"j1": 0.0, "j2": 90.0, "j3": 0.0})
        self.nominal_universal_joint_center_z = float(
            get_gait_cfg("nominal_universal_joint_center_z", self.legacy_nominal_z + abs(self.l_d6 * 1000.0 + self.l_d7 * 1000.0))
        )
        self.startup_pose_mode = str(get_gait_cfg("startup_pose_mode", "use_motor_home_ticks"))
        self.motor_home = get_gait_cfg("motor_home_ticks", {})
        self.motor_dir = get_gait_cfg("motor_dir", {})
        self.position_mode_ids = set([int(value) for value in get_gait_cfg("position_mode_ids", [])])
        self.direct_drive_ids = set([int(value) for value in get_gait_cfg("single_turn_ids", [])])

        self.unit_ticks_per_degree = 4096.0 / 360.0
        self.motor_to_joint = self._build_motor_joint_index()
        self.leg_to_motors = self._build_leg_to_motor_map()
        self.leg_hip_yaw = self._build_leg_yaw_map()
        self.nominal_output_deg_by_motor = self._compute_nominal_output_deg_by_motor()

        self.last_command_ticks = {motor_id: 0 for motor_id in self.motor_ids}
        self.last_position_rad = {motor_id: 0.0 for motor_id in self.motor_ids}
        self.last_sample_time = None
        self.left_board_ticks = None  # JointState from left_board/joint_state (C++ telemetry)
        self.right_board_ticks = None  # JointState from right_board/joint_state
        self.leg_support_mode = {leg_name: False for leg_name in self.leg_to_motors.keys()}
        self.current_operating_mode = {motor_id: self.default_operating_mode_marker for motor_id in self.motor_ids}
        self.leg_wrench = {
            leg_name: {"active": False, "force": [0.0, 0.0, 0.0]}
            for leg_name in self.leg_to_motors.keys()
        }

        self.cmd_pub = rospy.Publisher("/set_position", SetPosition, queue_size=200)
        self.current_cmd_pub = rospy.Publisher("/set_current", SetCurrent, queue_size=200)
        self.mode_cmd_pub = rospy.Publisher("/set_operating_mode", SetOperatingMode, queue_size=100)
        self.telemetry_pub = rospy.Publisher("~joint_state", JointState, queue_size=20)
        self.ticks_pub = rospy.Publisher("~joint_ticks", Int32MultiArray, queue_size=20)
        self.current_pub = rospy.Publisher("~joint_currents", Float32MultiArray, queue_size=20)

        self.get_position = None
        self.get_current = None
        self.position_clients = {}
        self.current_clients = {}
        self.bulk_clients = {}
        self.bulk_current_clients = {}
        self._init_service_clients()
        self._initialize_startup_pose_mode()

        rospy.Subscriber(self.command_topic, JointState, self.command_callback, queue_size=20)
        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self.swing_target_callback, queue_size=50)
        # Subscribe to C++ telemetry from multi_dxl_node (direct SyncRead, no service overhead)
        rospy.Subscriber("/jetson/left_board/joint_state", JointState, self._left_board_ticks_cb, queue_size=10)
        rospy.Subscriber("/jetson/right_board/joint_state", JointState, self._right_board_ticks_cb, queue_size=10)
        for leg_name in sorted(self.leg_to_motors.keys()):
            rospy.Subscriber(
                "/control/stance_wrench/" + leg_name,
                StanceWrenchCommand,
                self.stance_wrench_callback,
                callback_args=leg_name,
                queue_size=20,
            )

        self.timer = rospy.Timer(rospy.Duration(1.0 / max(self.telemetry_rate_hz, 1.0)), self.publish_telemetry)
        self.current_timer = rospy.Timer(rospy.Duration(1.0 / max(self.current_command_rate_hz, 1.0)), self.publish_current_commands)
        self.diag_timer = rospy.Timer(rospy.Duration(5.0), self._log_diagnostics)

    def _build_motor_joint_index(self):
        legs = rospy.get_param("/legs", {})
        mapping = {}
        for leg_name in ["lf", "rf", "rr", "lr"]:
            motor_ids = [int(value) for value in legs.get(leg_name, {}).get("motor_ids", [])]
            for joint_index, motor_id in enumerate(motor_ids):
                mapping[motor_id] = {"leg_name": leg_name, "joint_index": joint_index}
        return mapping

    def _build_leg_to_motor_map(self):
        legs = rospy.get_param("/legs", {})
        return {str(leg_name): [int(value) for value in cfg.get("motor_ids", [])] for leg_name, cfg in legs.items()}

    def _build_leg_yaw_map(self):
        legs = rospy.get_param("/legs", {})
        return {str(leg_name): math.radians(float(cfg.get("hip_yaw_deg", 0.0))) for leg_name, cfg in legs.items()}

    def _load_board_configs(self):
        board_configs = {}
        for board_name in ["left_board", "right_board"]:
            board_param = rospy.get_param("/" + board_name, None)
            if isinstance(board_param, dict):
                board_configs[board_name] = {"motor_ids": [int(value) for value in board_param.get("motor_ids", [])]}
        return board_configs

    def _qualify_service(self, board_name, service_basename):
        base_name = service_basename.strip("/")
        if self.runtime_namespace:
            return self.runtime_namespace + "/" + board_name + "/" + base_name
        return "/" + board_name + "/" + base_name

    def _init_service_clients(self):
        if self.board_configs:
            for board_name in sorted(self.board_configs.keys()):
                position_service = self._qualify_service(board_name, "get_position")
                rospy.wait_for_service(position_service)
                self.position_clients[board_name] = rospy.ServiceProxy(position_service, GetPosition)

                current_service = self._qualify_service(board_name, "get_current")
                try:
                    rospy.wait_for_service(current_service, timeout=3.0)
                    self.current_clients[board_name] = rospy.ServiceProxy(current_service, GetCurrent)
                except (rospy.ROSException, rospy.ROSInterruptException):
                    rospy.logwarn("dynamixel_bridge could not find %s; current telemetry will stay zero on %s", current_service, board_name)

                bulk_service = self._qualify_service(board_name, "get_bulk_positions")
                try:
                    rospy.wait_for_service(bulk_service, timeout=3.0)
                    self.bulk_clients[board_name] = rospy.ServiceProxy(bulk_service, GetBulkPositions)
                except (rospy.ROSException, rospy.ROSInterruptException):
                    rospy.logwarn("dynamixel_bridge could not find %s; will use per-motor reads", bulk_service)

                bulk_current_service = self._qualify_service(board_name, "get_bulk_currents")
                try:
                    rospy.wait_for_service(bulk_current_service, timeout=3.0)
                    self.bulk_current_clients[board_name] = rospy.ServiceProxy(bulk_current_service, GetBulkCurrents)
                except (rospy.ROSException, rospy.ROSInterruptException):
                    rospy.logwarn("dynamixel_bridge could not find %s; will use per-motor current reads", bulk_current_service)
            return

        rospy.wait_for_service(self.position_service_name)
        self.get_position = rospy.ServiceProxy(self.position_service_name, GetPosition)
        try:
            rospy.wait_for_service(self.current_service_name, timeout=3.0)
            self.get_current = rospy.ServiceProxy(self.current_service_name, GetCurrent)
        except (rospy.ROSException, rospy.ROSInterruptException):
            rospy.logwarn("dynamixel_bridge could not find %s; current/torque telemetry will stay zero", self.current_service_name)

    def _board_name_for_motor(self, motor_id):
        for board_name, board_cfg in self.board_configs.items():
            if int(motor_id) in board_cfg.get("motor_ids", []):
                return board_name
        return None

    def _joint_ratio(self, motor_id, joint_index):
        if motor_id in self.direct_drive_ids:
            return 1.0
        if joint_index == 1:
            return self.gear_ratio_joint2
        if joint_index == 2:
            return self.gear_ratio_joint3
        return 1.0

    def _joint_zero_deg_for_index(self, joint_index):
        return float(self.joint_zero_deg.get("j%d" % (joint_index + 1), 0.0))

    @staticmethod
    def _clamp(value, lo, hi):
        return max(lo, min(hi, value))

    @staticmethod
    def _solution_cost(candidate_deg, reference_deg):
        return sum((float(candidate_deg[index]) - float(reference_deg[index])) ** 2 for index in [0, 1, 2])

    def _ik_candidates_deg(self, x_mm, y_mm, z_mm):
        x_prime = float(x_mm) - self.l_coxa * 1000.0
        q1 = math.atan2(float(y_mm), x_prime)
        r_total = math.hypot(x_prime, float(y_mm))
        r_prime = max(r_total - self.l_femur * 1000.0, 1.0)

        l1 = self.l_tibia * 1000.0
        l2 = self.l_a3 * 1000.0
        d_sq = r_prime ** 2 + float(z_mm) ** 2
        cos_theta3 = (d_sq - l1 ** 2 - l2 ** 2) / (2.0 * l1 * l2)
        cos_theta3 = self._clamp(cos_theta3, -1.0, 1.0)
        sin_theta3_mag = math.sqrt(max(0.0, 1.0 - cos_theta3 ** 2))

        candidates = []
        for branch_sign in [-1.0, 1.0]:
            theta3 = math.atan2(branch_sign * sin_theta3_mag, cos_theta3)
            q2 = math.atan2(float(z_mm), r_prime) - math.atan2(l2 * math.sin(theta3), l1 + l2 * math.cos(theta3)) + math.radians(90.0)
            candidates.append([math.degrees(q1), math.degrees(q2), math.degrees(theta3)])
        return candidates

    def _ik_transform_matrix_solve_deg(self, x_mm, y_mm, z_mm):
        candidates = self._ik_candidates_deg(x_mm, y_mm, z_mm)
        return min(candidates, key=lambda candidate: self._solution_cost(candidate, [0.0, 0.0, 0.0]))

    def _compute_nominal_output_deg_by_motor(self):
        nominal_joint_deg = self._ik_transform_matrix_solve_deg(
            self.nominal_x,
            self.nominal_y,
            self.nominal_universal_joint_center_z,
        )
        nominal_output_deg = [nominal_joint_deg[index] + self._joint_zero_deg_for_index(index) for index in [0, 1, 2]]
        mapping = {}
        for leg_name, motor_ids in self.leg_to_motors.items():
            for joint_index, motor_id in enumerate(motor_ids):
                mapping[int(motor_id)] = float(nominal_output_deg[joint_index])
        return mapping

    def _capture_motor_home_from_current_pose(self):
        captured_home = {}
        for motor_id in self.motor_ids:
            raw_position = self._read_motor_position(motor_id)
            if raw_position is None:
                return None
            if int(motor_id) in self.position_mode_ids:
                raw_position %= 4096
            captured_home[str(int(motor_id))] = int(raw_position)
        return captured_home

    def _initialize_startup_pose_mode(self):
        if self.startup_pose_mode == "capture_current_as_home":
            captured_home = self._capture_motor_home_from_current_pose()
            if captured_home is None:
                rospy.logwarn("dynamixel_bridge could not capture startup motor positions; using configured motor_home_ticks")
                return
            self.motor_home = captured_home
            rospy.set_param("/gait_controller/motor_home_ticks", self.motor_home)
            rospy.loginfo("dynamixel_bridge captured current motor positions as motor_home_ticks")
            return
        if self.startup_pose_mode != "use_motor_home_ticks":
            rospy.logwarn("Unknown startup_pose_mode '%s'; defaulting to use_motor_home_ticks", self.startup_pose_mode)
            self.startup_pose_mode = "use_motor_home_ticks"

    def _output_deg_from_ticks(self, motor_id, raw_ticks):
        home = float(self.motor_home.get(str(int(motor_id)), 0.0))
        direction = float(self.motor_dir.get(str(int(motor_id)), 1.0))
        if abs(direction) < 1e-6:
            direction = 1.0

        delta_ticks = float(raw_ticks) - home
        if int(motor_id) in self.position_mode_ids:
            while delta_ticks > 2048.0:
                delta_ticks -= 4096.0
            while delta_ticks < -2048.0:
                delta_ticks += 4096.0
        joint_meta = self.motor_to_joint.get(int(motor_id))
        if joint_meta is None:
            return 0.0
        ratio = self._joint_ratio(int(motor_id), int(joint_meta["joint_index"]))
        if abs(ratio) < 1e-9:
            ratio = 1.0
        nominal_output_deg = float(self.nominal_output_deg_by_motor.get(int(motor_id), 0.0))
        return nominal_output_deg + delta_ticks / (direction * self.unit_ticks_per_degree * ratio)

    def _ticks_to_joint_rad(self, motor_id, raw_ticks):
        joint_meta = self.motor_to_joint.get(int(motor_id))
        if joint_meta is None:
            return 0.0
        joint_index = int(joint_meta["joint_index"])
        cmd_deg = self._output_deg_from_ticks(int(motor_id), raw_ticks)
        joint_deg = cmd_deg - self._joint_zero_deg_for_index(joint_index)
        return math.radians(joint_deg)

    def _current_raw_to_amp(self, raw_current):
        return float(raw_current) * self.current_lsb_ma / 1000.0

    def _amp_to_current_raw(self, current_amp):
        return int(round(float(current_amp) * 1000.0 / max(self.current_lsb_ma, 1e-6)))

    @staticmethod
    def _piecewise_interpolate(abs_current_amp, points):
        if not points:
            return None
        normalized_points = []
        for point in points:
            normalized_points.append((abs(float(point.get("current_a", point.get("abs_current_a", 0.0)))), float(point.get("torque_nm", 0.0))))
        normalized_points.sort(key=lambda item: item[0])
        if len(normalized_points) == 1:
            return normalized_points[0][1]
        if abs_current_amp <= normalized_points[0][0]:
            x0, y0 = normalized_points[0]
            x1, y1 = normalized_points[1]
        elif abs_current_amp >= normalized_points[-1][0]:
            x0, y0 = normalized_points[-2]
            x1, y1 = normalized_points[-1]
        else:
            x0, y0 = normalized_points[0]
            x1, y1 = normalized_points[1]
            for index in range(1, len(normalized_points)):
                if abs_current_amp <= normalized_points[index][0]:
                    x0, y0 = normalized_points[index - 1]
                    x1, y1 = normalized_points[index]
                    break
        if abs(x1 - x0) < 1e-9:
            return y0
        ratio = (abs_current_amp - x0) / (x1 - x0)
        return y0 + ratio * (y1 - y0)

    def _estimate_torque_nm(self, motor_id, current_amp):
        key = str(int(motor_id))
        model_name = self.torque_model_by_motor.get(key, "")
        model_cfg = self.torque_models.get(model_name, {}) if model_name else {}
        piecewise_torque = self._piecewise_interpolate(abs(float(current_amp)), model_cfg.get("current_to_torque_points", []))
        if piecewise_torque is not None:
            return -piecewise_torque if current_amp < 0.0 else piecewise_torque
        gain = float(self.torque_constant_nm_per_a.get(key, model_cfg.get("torque_constant_nm_per_a", self.torque_constant_nm_per_a.get("default", 1.0))))
        bias = float(self.torque_bias_nm.get(key, model_cfg.get("torque_bias_nm", self.torque_bias_nm.get("default", 0.0))))
        return gain * float(current_amp) + bias

    def _estimate_current_from_torque_nm(self, motor_id, torque_nm):
        key = str(int(motor_id))
        model_name = self.torque_model_by_motor.get(key, "")
        model_cfg = self.torque_models.get(model_name, {}) if model_name else {}
        points = model_cfg.get("current_to_torque_points", [])
        abs_torque = abs(float(torque_nm))
        if points:
            lookup = sorted([(float(point.get("torque_nm", 0.0)), abs(float(point.get("current_a", point.get("abs_current_a", 0.0))))) for point in points], key=lambda item: item[0])
            if len(lookup) == 1:
                current_amp = lookup[0][1]
            elif abs_torque <= lookup[0][0]:
                x0, y0 = lookup[0]
                x1, y1 = lookup[1]
                current_amp = y0 if abs(x1 - x0) < 1e-9 else y0 + (abs_torque - x0) * (y1 - y0) / (x1 - x0)
            elif abs_torque >= lookup[-1][0]:
                x0, y0 = lookup[-2]
                x1, y1 = lookup[-1]
                current_amp = y0 if abs(x1 - x0) < 1e-9 else y0 + (abs_torque - x0) * (y1 - y0) / (x1 - x0)
            else:
                current_amp = 0.0
                for index in range(1, len(lookup)):
                    if abs_torque <= lookup[index][0]:
                        x0, y0 = lookup[index - 1]
                        x1, y1 = lookup[index]
                        current_amp = y0 if abs(x1 - x0) < 1e-9 else y0 + (abs_torque - x0) * (y1 - y0) / (x1 - x0)
                        break
            return current_amp if torque_nm >= 0.0 else -current_amp
        gain = float(self.torque_constant_nm_per_a.get(key, model_cfg.get("torque_constant_nm_per_a", self.torque_constant_nm_per_a.get("default", 1.0))))
        bias = float(self.torque_bias_nm.get(key, model_cfg.get("torque_bias_nm", self.torque_bias_nm.get("default", 0.0))))
        if abs(gain) < 1e-9:
            return 0.0
        return (float(torque_nm) - bias) / gain

    def _base_force_to_leg_force(self, leg_name, force_vector):
        hip_yaw = self.leg_hip_yaw.get(leg_name, 0.0)
        cos_yaw = math.cos(hip_yaw)
        sin_yaw = math.sin(hip_yaw)
        return [cos_yaw * force_vector[0] + sin_yaw * force_vector[1], -sin_yaw * force_vector[0] + cos_yaw * force_vector[1], force_vector[2]]

    def _forward_kinematics_leg(self, joint_vector):
        q1 = float(joint_vector[0])
        q2 = float(joint_vector[1])
        q3 = float(joint_vector[2])
        alpha = q2 - math.radians(90.0)
        radial_prime = self.l_tibia * math.cos(alpha) + self.l_a3 * math.cos(alpha + q3)
        p_z = self.l_tibia * math.sin(alpha) + self.l_a3 * math.sin(alpha + q3)
        radial_total = self.l_femur + radial_prime
        return [self.l_coxa + radial_total * math.cos(q1), radial_total * math.sin(q1), p_z]

    def _leg_joint_vector(self, leg_name):
        motor_ids = self.leg_to_motors.get(leg_name, [])
        if len(motor_ids) != 3:
            return None
        return [float(self.last_position_rad.get(int(motor_id), 0.0)) for motor_id in motor_ids]

    def _leg_jacobian(self, leg_name):
        joint_vector = self._leg_joint_vector(leg_name)
        if joint_vector is None:
            return None
        base_position = self._forward_kinematics_leg(joint_vector)
        jacobian = [[0.0, 0.0, 0.0] for _ in range(3)]
        for column in [0, 1, 2]:
            perturbed = list(joint_vector)
            perturbed[column] += self.jacobian_delta_rad
            next_position = self._forward_kinematics_leg(perturbed)
            for row in [0, 1, 2]:
                jacobian[row][column] = (next_position[row] - base_position[row]) / max(self.jacobian_delta_rad, 1e-9)
        return jacobian

    @staticmethod
    def _transpose_multiply(matrix, vector):
        result = [0.0 for _ in range(len(matrix[0]))]
        for row in range(len(matrix)):
            for column in range(len(matrix[row])):
                result[column] += matrix[row][column] * vector[row]
        return result

    def _set_leg_mode(self, leg_name, support_mode):
        requested_mode = self.support_operating_mode if support_mode else self.default_operating_mode_marker
        for motor_id in self.leg_to_motors.get(leg_name, []):
            if self.current_operating_mode.get(int(motor_id)) == requested_mode:
                continue
            packet = SetOperatingMode()
            packet.id = int(motor_id)
            packet.mode = int(requested_mode)
            self.mode_cmd_pub.publish(packet)
            self.current_operating_mode[int(motor_id)] = int(requested_mode)

    def swing_target_callback(self, msg):
        if not self.enable_auto_mode_switching:
            return
        if msg.leg_name not in self.leg_support_mode:
            return
        support_mode = bool(msg.support_leg)
        if self.leg_support_mode[msg.leg_name] != support_mode:
            self.leg_support_mode[msg.leg_name] = support_mode
            self._set_leg_mode(msg.leg_name, support_mode)

    def _left_board_ticks_cb(self, msg):
        self.left_board_ticks = msg

    def _right_board_ticks_cb(self, msg):
        self.right_board_ticks = msg

    def stance_wrench_callback(self, msg, leg_name):
        if not self.enable_auto_current_control:
            return
        if leg_name not in self.leg_wrench:
            return
        self.leg_wrench[leg_name]["active"] = bool(msg.active)
        self.leg_wrench[leg_name]["force"] = [float(msg.wrench.force.x), float(msg.wrench.force.y), float(msg.wrench.force.z)]

    def command_callback(self, msg):
        if len(msg.position) != len(self.motor_ids):
            rospy.logwarn_throttle(2.0, "joint_position_ticks_cmd length does not match ordered_motor_ids")
            return
        self.command_recv_count += 1
        for motor_id, tick in zip(self.motor_ids, msg.position):
            packet = SetPosition()
            packet.id = int(motor_id)
            packet.position = int(round(tick))
            self.last_command_ticks[int(motor_id)] = packet.position
            self.cmd_pub.publish(packet)
            self.position_pub_count += 1

    def _read_motor_position(self, motor_id):
        board_name = self._board_name_for_motor(motor_id)
        try:
            if board_name is not None:
                return int(self.position_clients[board_name](int(motor_id)).position)
            if self.get_position is None:
                return None
            return int(self.get_position(int(motor_id)).position)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "Failed to read position for ID %d: %s", motor_id, exc)
            return None

    def _read_motor_current(self, motor_id):
        board_name = self._board_name_for_motor(motor_id)
        try:
            if board_name is not None:
                client = self.current_clients.get(board_name)
                if client is None:
                    return None
                return int(client(int(motor_id)).current)
            if self.get_current is None:
                return None
            return int(self.get_current(int(motor_id)).current)
        except rospy.ServiceException as exc:
            rospy.logwarn_throttle(2.0, "Failed to read current for ID %d: %s", motor_id, exc)
            return None

    def _count_failed_telemetry_reads(self):
        """Return (failed_pos, failed_cur) counts since last call (sampled, not exact)."""
        return 0, 0

    def _log_diagnostics(self, _event):
        now = rospy.Time.now()
        elapsed = (now - self.last_diag_log_time).to_sec()
        self.last_diag_log_time = now

        cmd_rate = self.command_recv_count / max(elapsed, 1e-6)
        pos_rate = self.position_pub_count / max(elapsed, 1e-6)

        rospy.loginfo(
            "DIAG: commands_rx=%d (%.1f Hz) | SetPosition_tx=%d (%.1f Hz) | "
            "telemetry_rate=%.1f Hz (target=%.0f Hz)",
            self.command_recv_count, cmd_rate,
            self.position_pub_count, pos_rate,
            1.0 / max(elapsed, 1e-6), self.telemetry_rate_hz,
        )
        self.command_recv_count = 0
        self.position_pub_count = 0

    def _bulk_read_positions(self):
        """Read positions of all motors on each board in bulk. Returns dict motor_id->tick (or None)."""
        result = {}
        if self.board_configs:
            for board_name in sorted(self.board_configs.keys()):
                bulk_client = self.bulk_clients.get(board_name)
                if bulk_client is None:
                    # Fallback to per-motor reads
                    for motor_id in self.board_configs[board_name].get("motor_ids", []):
                        result[motor_id] = self._read_motor_position(motor_id)
                    continue
                try:
                    # Bulk read: pass empty list to read all controlled IDs on this board
                    resp = bulk_client(motor_ids=[])
                    motor_ids = self.board_configs[board_name].get("motor_ids", [])
                    for motor_id, pos in zip(motor_ids, resp.positions):
                        if pos == 0:
                            result[motor_id] = None
                        else:
                            result[motor_id] = pos
                except (rospy.ServiceException, rospy.ROSException) as exc:
                    rospy.logwarn_throttle(5.0, "Bulk read failed on %s: %s", board_name, exc)
                    for motor_id in self.board_configs[board_name].get("motor_ids", []):
                        result[motor_id] = self._read_motor_position(motor_id)
        else:
            for motor_id in self.motor_ids:
                result[motor_id] = self._read_motor_position(motor_id)
        return result

    def _bulk_read_currents(self):
        """Read currents of all motors on each board in bulk. Returns dict motor_id->raw_current (or None)."""
        result = {}
        if self.board_configs:
            for board_name in sorted(self.board_configs.keys()):
                bulk_client = self.bulk_current_clients.get(board_name)
                if bulk_client is None:
                    for motor_id in self.board_configs[board_name].get("motor_ids", []):
                        result[motor_id] = self._read_motor_current(motor_id)
                    continue
                try:
                    resp = bulk_client(motor_ids=[])
                    motor_ids = self.board_configs[board_name].get("motor_ids", [])
                    for motor_id, raw_current in zip(motor_ids, resp.currents):
                        if raw_current == 0:
                            result[motor_id] = None
                        else:
                            result[motor_id] = raw_current
                except (rospy.ServiceException, rospy.ROSException) as exc:
                    rospy.logwarn_throttle(5.0, "Bulk current read failed on %s: %s", board_name, exc)
                    for motor_id in self.board_configs[board_name].get("motor_ids", []):
                        result[motor_id] = self._read_motor_current(motor_id)
        else:
            for motor_id in self.motor_ids:
                result[motor_id] = self._read_motor_current(motor_id)
        return result

    def publish_telemetry(self, _event):
        stamp = rospy.Time.now()

        # Merge left + right board telemetry into motor_id -> value dicts
        ticks_by_id = {}
        currents_by_id = {}
        for js in [self.left_board_ticks, self.right_board_ticks]:
            if js is None:
                continue
            for i, motor_id_str in enumerate(js.name):
                mid = int(motor_id_str)
                ticks_by_id[mid] = int(round(js.position[i]))
                if i < len(js.effort):
                    currents_by_id[mid] = int(round(js.effort[i]))

        dt = None
        if self.last_sample_time is not None:
            dt = max((stamp - self.last_sample_time).to_sec(), 1e-6)
        self.last_sample_time = stamp

        positions = []
        velocities = []
        torques = []
        currents = []

        for motor_id in self.motor_ids:
            raw_position = ticks_by_id.get(motor_id)
            if raw_position is None:
                raw_position = self.last_command_ticks.get(motor_id, 0)
            joint_position = self._ticks_to_joint_rad(motor_id, raw_position)
            previous_position = self.last_position_rad[motor_id]
            joint_velocity = 0.0 if dt is None else (joint_position - previous_position) / dt
            self.last_position_rad[motor_id] = joint_position

            raw_current = currents_by_id.get(motor_id)
            current_amp = 0.0 if raw_current is None else self._current_raw_to_amp(raw_current)
            torque_nm = self._estimate_torque_nm(motor_id, current_amp)

            positions.append(joint_position)
            velocities.append(joint_velocity)
            torques.append(torque_nm)
            currents.append(current_amp)

        joint_state = JointState()
        joint_state.header.stamp = stamp
        joint_state.name = [str(motor_id) for motor_id in self.motor_ids]
        joint_state.position = positions
        joint_state.velocity = velocities
        joint_state.effort = torques
        self.telemetry_pub.publish(joint_state)

        # Raw ticks
        ticks_msg = Int32MultiArray()
        ticks_msg.layout = MultiArrayLayout()
        ticks_msg.layout.data_offset = 0
        ticks_msg.data = [ticks_by_id.get(mid, 0) for mid in self.motor_ids]
        self.ticks_pub.publish(ticks_msg)

        # Raw currents (amps)
        current_msg = Float32MultiArray()
        current_msg.data = currents
        self.current_pub.publish(current_msg)

    def publish_current_commands(self, _event):
        if not self.enable_auto_current_control:
            return
        for leg_name, support_mode in self.leg_support_mode.items():
            if not support_mode:
                continue
            wrench_state = self.leg_wrench.get(leg_name, {})
            if not bool(wrench_state.get("active", False)):
                continue
            jacobian = self._leg_jacobian(leg_name)
            if jacobian is None:
                continue

            leg_force = self._base_force_to_leg_force(leg_name, wrench_state.get("force", [0.0, 0.0, 0.0]))
            joint_torques = self._transpose_multiply(jacobian, leg_force)
            motor_ids = self.leg_to_motors.get(leg_name, [])

            for joint_index, motor_id in enumerate(motor_ids):
                torque_nm = self.joint_current_scale[joint_index] * joint_torques[joint_index]
                current_amp = abs(self._estimate_current_from_torque_nm(motor_id, torque_nm)) + self.support_current_margin_a
                current_amp = min(self.max_goal_current_a, max(self.min_goal_current_a, current_amp))
                packet = SetCurrent()
                packet.id = int(motor_id)
                packet.current = int(self._amp_to_current_raw(current_amp))
                self.current_cmd_pub.publish(packet)


if __name__ == "__main__":
    DynamixelBridge()
    rospy.spin()