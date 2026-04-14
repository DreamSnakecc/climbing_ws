#!/usr/bin/env python

import math

import numpy as np

import rospy
from climbing_msgs.msg import AdhesionCommand, BodyReference, EstimatedState, LegCenterCommand, StanceWrenchCommand
from geometry_msgs.msg import Point
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Float32MultiArray


def clamp(value, lower_bound, upper_bound):
    return max(lower_bound, min(upper_bound, value))


def quaternion_to_list(quaternion_msg):
    return [
        float(quaternion_msg.x),
        float(quaternion_msg.y),
        float(quaternion_msg.z),
        float(quaternion_msg.w),
    ]


def quaternion_conjugate(quaternion):
    return [-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]]


def quaternion_multiply(lhs, rhs):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum([value * value for value in quaternion]))
    if norm < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in quaternion]


def vector_dot(lhs, rhs):
    return float(sum([float(lhs[index]) * float(rhs[index]) for index in range(3)]))


def vector_norm(vector):
    return math.sqrt(vector_dot(vector, vector))


def normalize_vector(vector, fallback):
    try:
        parsed = [float(value) for value in vector]
    except (TypeError, ValueError):
        parsed = list(fallback)
    if len(parsed) != 3:
        parsed = list(fallback)
    norm = vector_norm(parsed)
    if norm < 1e-9:
        parsed = list(fallback)
        norm = vector_norm(parsed)
    return [value / max(norm, 1e-9) for value in parsed]


def vector_add(lhs, rhs):
    return [lhs[index] + rhs[index] for index in range(3)]


def vector_scale(vector, scalar):
    return [float(scalar) * float(value) for value in vector]


def cross(lhs, rhs):
    return [
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    ]


def rotate_vector(quaternion, vector):
    q = normalize_quaternion(quaternion)
    pure = [float(vector[0]), float(vector[1]), float(vector[2]), 0.0]
    rotated = quaternion_multiply(quaternion_multiply(q, pure), quaternion_conjugate(q))
    return rotated[:3]


def body_to_world(vector_body, body_orientation):
    return rotate_vector(body_orientation, vector_body)


def quaternion_to_axis_angle(quaternion):
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vector_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if vector_norm < 1e-9:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(vector_norm, qw)
    axis = [qx / vector_norm, qy / vector_norm, qz / vector_norm]
    return [angle * axis[0], angle * axis[1], angle * axis[2]]


def low_pass_blend(previous, current, alpha):
    return [(1.0 - alpha) * previous[index] + alpha * current[index] for index in range(len(previous))]


class StateEstimator(object):
    def __init__(self):
        rospy.init_node("state_estimator", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/state_estimator/" + name, default))

        self.leg_names = [str(name) for name in get_cfg("leg_names", ["lf", "rf", "rr", "lr"])]
        self.rate_hz = float(get_cfg("publish_rate_hz", 50.0))
        self.mocap_timeout_s = float(get_cfg("mocap_timeout_s", 0.25))
        self.use_external_pose_estimate = bool(get_cfg("use_external_pose_estimate", False))
        self.linear_velocity_alpha = float(get_cfg("linear_velocity_alpha", 0.35))
        self.angular_velocity_alpha = float(get_cfg("angular_velocity_alpha", 0.35))
        self.support_force_threshold_n = float(get_cfg("support_force_threshold_n", 5.0))
        self.adhesion_force_threshold_n = float(get_cfg("adhesion_force_threshold_n", 10.0))
        self.current_contact_threshold_a = float(get_cfg("current_contact_threshold_a", 0.12))
        self.torque_contact_threshold_nm = float(get_cfg("torque_contact_threshold_nm", 0.05))
        self.slip_warning_ratio = float(get_cfg("slip_warning_ratio", 0.75))
        self.early_contact_phase_threshold = float(get_cfg("early_contact_phase_threshold", 0.5))
        self.wall_mu = float(get_cfg("friction_coefficient", rospy.get_param("/wall/default_friction_coefficient", 0.5)))
        self.default_normal_force_limit = float(get_cfg("default_normal_force_limit", rospy.get_param("/wall/max_normal_force_n", 150.0)))
        legacy_axis = str(get_cfg("contact_normal_axis", rospy.get_param("/wall/legacy_contact_normal_axis", "z"))).lower()
        legacy_fallback = {
            "x": [1.0, 0.0, 0.0],
            "y": [0.0, 1.0, 0.0],
            "z": [0.0, 0.0, 1.0],
        }.get(legacy_axis, [0.0, 0.0, 1.0])
        self.wall_normal_body = normalize_vector(
            get_cfg("wall_normal_body", rospy.get_param("/wall/normal_body", legacy_fallback)),
            legacy_fallback,
        )
        self.contact_offset_sign = float(get_cfg("contact_offset_sign", 1.0))
        self.compression_ready_ratio = float(get_cfg("compression_ready_ratio", 0.8))
        self.normal_force_to_compression_gain_m_per_n = float(get_cfg("normal_force_to_compression_gain_m_per_n", 8.0e-5))
        self.stable_contact_confidence_threshold = float(get_cfg("stable_contact_confidence_threshold", 0.75))
        self.recent_contact_position_alpha = float(get_cfg("recent_contact_position_alpha", 0.2))
        self.adhesion_force_reference_n = [float(value) for value in get_cfg("adhesion_force_reference_n", [65.0, 83.0, 107.0, 145.0])]
        self.adhesion_skirt_compression_mm = [float(value) for value in get_cfg("adhesion_skirt_compression_mm", [6.5, 7.2, 8.3, 9.2])]
        self.fan_min_active_rpm = float(get_cfg("fan_min_active_rpm", 300.0))
        self.fan_adhesion_reference_rpm = [float(value) for value in get_cfg("fan_adhesion_reference_rpm", [30000.0, 40000.0])]
        self.fan_attached_current_min_a = [float(value) for value in get_cfg("fan_attached_current_min_a", [1.2, 1.8])]
        self.fan_attached_current_max_a = [float(value) for value in get_cfg("fan_attached_current_max_a", [2.0, 2.4])]
        self.fan_detached_current_min_a = [float(value) for value in get_cfg("fan_detached_current_min_a", [5.0, 7.0])]
        self.fan_detached_current_max_a = [float(value) for value in get_cfg("fan_detached_current_max_a", [5.4, 9.0])]
        self.fan_adhesion_confidence_threshold = float(
            get_cfg("fan_adhesion_confidence_threshold", self.stable_contact_confidence_threshold)
        )
        self.gravity_world = [float(value) for value in get_cfg("gravity_world", [0.0, 0.0, -9.81])]
        self.ekf_initial_covariance = max(float(get_cfg("ekf_initial_covariance", 0.25)), 1e-6)
        self.ekf_body_position_process_std_m = max(float(get_cfg("ekf_body_position_process_std_m", 0.02)), 1e-6)
        self.ekf_body_velocity_process_std_mps = max(float(get_cfg("ekf_body_velocity_process_std_mps", 0.15)), 1e-6)
        self.ekf_body_acceleration_process_std_mps2 = max(float(get_cfg("ekf_body_acceleration_process_std_mps2", 1.5)), 1e-6)
        # Keep legacy config keys for compatibility. They refer to the universal-joint-center support point.
        self.ekf_contact_foot_process_std_m = max(float(get_cfg("ekf_contact_foot_process_std_m", 0.003)), 1e-6)
        self.ekf_swing_foot_process_std_m = max(float(get_cfg("ekf_swing_foot_process_std_m", 0.08)), 1e-6)
        self.ekf_contact_position_measurement_std_m = max(float(get_cfg("ekf_contact_position_measurement_std_m", 0.01)), 1e-6)
        self.ekf_swing_position_measurement_std_m = max(float(get_cfg("ekf_swing_position_measurement_std_m", 0.20)), 1e-6)
        self.ekf_contact_velocity_measurement_std_mps = max(float(get_cfg("ekf_contact_velocity_measurement_std_mps", 0.05)), 1e-6)
        self.ekf_swing_velocity_measurement_std_mps = max(float(get_cfg("ekf_swing_velocity_measurement_std_mps", 1.20)), 1e-6)
        self.ekf_external_position_measurement_std_m = max(float(get_cfg("ekf_external_position_measurement_std_m", 0.01)), 1e-6)
        self.ekf_reset_position_jump_m = max(float(get_cfg("ekf_reset_position_jump_m", 0.75)), 1e-6)
        self.nominal_swing_duration_s = self._nominal_swing_duration()
        self.base_radius_m = float(rospy.get_param("/gait_controller/base_radius", 203.06)) / 1000.0
        self.nominal_x_m = float(rospy.get_param("/gait_controller/nominal_x", 118.75)) / 1000.0
        self.nominal_y_m = float(rospy.get_param("/gait_controller/nominal_y", 0.0)) / 1000.0
        self.legacy_nominal_control_point_z_m = float(rospy.get_param("/gait_controller/nominal_z", -299.2)) / 1000.0
        self.l_coxa = float(rospy.get_param("/gait_controller/link_coxa", 44.75)) / 1000.0
        self.l_femur = float(rospy.get_param("/gait_controller/link_femur", 74.0)) / 1000.0
        self.l_tibia = float(rospy.get_param("/gait_controller/link_tibia", 150.0)) / 1000.0
        self.l_a3 = float(rospy.get_param("/gait_controller/link_a3", 41.5)) / 1000.0
        self.l_d6 = float(rospy.get_param("/gait_controller/link_d6", -13.5)) / 1000.0
        self.l_d7 = float(rospy.get_param("/gait_controller/link_d7", -106.7)) / 1000.0
        self.suction_cavity_height_m = float(rospy.get_param("/robot/suction_cavity_height_mm", 20.0)) / 1000.0
        self.universal_joint_to_foot_center_rigid_offset_m = abs(self.l_d6 + self.l_d7) + self.suction_cavity_height_m
        self.nominal_universal_joint_center_z_m = float(
            rospy.get_param(
                "/gait_controller/nominal_universal_joint_center_z",
                (self.legacy_nominal_control_point_z_m + abs(self.l_d6 + self.l_d7)) * 1000.0,
            )
        ) / 1000.0
        self.skirt_height_uncompressed_m = float(rospy.get_param("/robot/skirt_height_uncompressed_mm", 20.0)) / 1000.0
        self.skirt_height_compressed_min_m = float(rospy.get_param("/robot/skirt_height_compressed_min_mm", 8.0)) / 1000.0
        self.skirt_height_compressed_max_m = float(rospy.get_param("/robot/skirt_height_compressed_max_mm", 10.0)) / 1000.0
        self.max_skirt_compression_m = max(self.skirt_height_uncompressed_m - self.skirt_height_compressed_min_m, 0.0)
        self.nominal_skirt_compression_m = max(
            self.skirt_height_uncompressed_m - 0.5 * (self.skirt_height_compressed_min_m + self.skirt_height_compressed_max_m),
            0.0,
        )

        self.leg_motor_ids = self._build_leg_motor_map()
        self.leg_hip_yaw = self._build_leg_yaw_map()
        self.swing_targets = {
            leg_name: {"support_leg": False, "desired_normal_force_limit": 0.0, "skirt_compression_target": 0.0}
            for leg_name in self.leg_names
        }
        self.stance_commands = {
            leg_name: {
                "active": False,
                "force": [0.0, 0.0, 0.0],
                "normal_force_limit": self.default_normal_force_limit,
                "required_adhesion_force": 0.0,
            }
            for leg_name in self.leg_names
        }
        self.body_reference_support_mask = [True] * len(self.leg_names)
        self.previous_body_reference_support_mask = [True] * len(self.leg_names)
        self.leg_swing_start_time = {leg_name: None for leg_name in self.leg_names}

        self.last_pose = None
        self.prev_pose = None
        self.last_imu = Imu()
        self.last_joint_state = JointState()
        self.last_joint_currents = []
        self.last_fan_currents = [0.0] * len(self.leg_names)
        self.last_fan_target_rpm = {leg_name: 0.0 for leg_name in self.leg_names}
        self.filtered_linear_velocity = [0.0, 0.0, 0.0]
        self.filtered_angular_velocity = [0.0, 0.0, 0.0]
        self.filtered_body_linear_velocity = [0.0, 0.0, 0.0]
        self.integrated_position_world = [0.0, 0.0, 0.0]
        self.last_publish_time_sec = None
        self.last_ekf_time_sec = None
        self.prev_universal_joint_center_positions = None
        self.prev_universal_joint_center_time_sec = None
        self.recent_stable_foot_center_positions = [self._nominal_foot_center_position(leg_name) for leg_name in self.leg_names]
        self.ekf_state_dim = 6 + 3 * len(self.leg_names)
        self.ekf_state = np.zeros((self.ekf_state_dim, 1), dtype=float)
        self.ekf_covariance = np.eye(self.ekf_state_dim, dtype=float) * self.ekf_initial_covariance
        self.ekf_initialized = False

        self.pub = rospy.Publisher("/state/estimated", EstimatedState, queue_size=20)
        rospy.Subscriber("/sensing/mocap_pose", PoseStamped, self.mocap_callback, queue_size=20)
        rospy.Subscriber("/jetson/imu_serial_bridge/imu", Imu, self.imu_callback, queue_size=50)
        rospy.Subscriber("/jetson/dynamixel_bridge/joint_state", JointState, self.joint_callback, queue_size=20)
        rospy.Subscriber("/jetson/dynamixel_bridge/joint_currents", Float32MultiArray, self.current_callback, queue_size=20)
        rospy.Subscriber("/jetson/fan_serial_bridge/fan_currents", Float32MultiArray, self.fan_current_callback, queue_size=20)
        rospy.Subscriber("/jetson/fan_serial_bridge/adhesion_command", AdhesionCommand, self.adhesion_command_callback, queue_size=50)
        rospy.Subscriber("/control/body_reference", BodyReference, self.body_reference_callback, queue_size=20)
        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self.swing_target_callback, queue_size=50)
        for leg_name in self.leg_names:
            rospy.Subscriber(
                "/control/stance_wrench/" + leg_name,
                StanceWrenchCommand,
                self.stance_wrench_callback,
                callback_args=leg_name,
                queue_size=20,
            )
        rospy.Timer(rospy.Duration(1.0 / max(self.rate_hz, 1.0)), self.publish_state)

    def _build_leg_motor_map(self):
        leg_map = {}
        legs = rospy.get_param("/legs", {})
        for leg_name in self.leg_names:
            leg_map[leg_name] = [int(value) for value in legs.get(leg_name, {}).get("motor_ids", [])]
        return leg_map

    def _build_leg_yaw_map(self):
        legs = rospy.get_param("/legs", {})
        yaw_map = {}
        for leg_name in self.leg_names:
            yaw_map[leg_name] = math.radians(float(legs.get(leg_name, {}).get("hip_yaw_deg", 0.0)))
        return yaw_map

    def _nominal_swing_duration(self):
        gait = str(rospy.get_param("/body_planner/gait", "crawl")).lower()
        gait_frequency_hz = float(rospy.get_param("/body_planner/gait_frequency_hz", 0.25))
        if gait == "trot":
            swing_ratio = float(rospy.get_param("/body_planner/trot_swing_ratio", 0.50))
        else:
            swing_ratio = float(rospy.get_param("/body_planner/crawl_swing_ratio", 0.25))
        if gait not in ["crawl", "trot"] or gait_frequency_hz <= 1e-6:
            return max(float(rospy.get_param("/swing_leg_controller/swing_duration_s", 0.65)), 1e-3)
        return max(swing_ratio / gait_frequency_hz, 1e-3)

    def _update_swing_phase_tracking(self, next_support_mask):
        now_sec = rospy.Time.now().to_sec()
        updated_mask = [bool(value) for value in next_support_mask]
        for leg_index, leg_name in enumerate(self.leg_names):
            previous_support = self.previous_body_reference_support_mask[leg_index] if leg_index < len(self.previous_body_reference_support_mask) else True
            current_support = updated_mask[leg_index] if leg_index < len(updated_mask) else True
            if previous_support and not current_support:
                self.leg_swing_start_time[leg_name] = now_sec
            elif current_support:
                self.leg_swing_start_time[leg_name] = None
        self.previous_body_reference_support_mask = list(updated_mask)

    def _leg_swing_phase(self, leg_name, planned_support):
        if planned_support:
            return 0.0
        start_time = self.leg_swing_start_time.get(leg_name)
        if start_time is None:
            return 0.0
        elapsed = rospy.Time.now().to_sec() - start_time
        return clamp(elapsed / max(self.nominal_swing_duration_s, 1e-3), 0.0, 1.0)

    def mocap_callback(self, msg):
        if self.last_pose is not None:
            self.prev_pose = self.last_pose
        self.last_pose = msg
        self._update_twist_from_mocap()

    def imu_callback(self, msg):
        self.last_imu = msg

    def joint_callback(self, msg):
        self.last_joint_state = msg

    def current_callback(self, msg):
        self.last_joint_currents = list(msg.data)

    def fan_current_callback(self, msg):
        values = list(msg.data)
        self.last_fan_currents = [float(values[index]) if index < len(values) else 0.0 for index in range(len(self.leg_names))]

    def adhesion_command_callback(self, msg):
        leg_index = int(msg.leg_index)
        if leg_index < 0 or leg_index >= len(self.leg_names):
            return
        self.last_fan_target_rpm[self.leg_names[leg_index]] = float(msg.target_rpm)

    def body_reference_callback(self, msg):
        if len(msg.support_mask) == len(self.leg_names):
            support_mask = [bool(value) for value in msg.support_mask]
            self._update_swing_phase_tracking(support_mask)
            self.body_reference_support_mask = support_mask

    def swing_target_callback(self, msg):
        if msg.leg_name not in self.swing_targets:
            return
        self.swing_targets[msg.leg_name]["support_leg"] = bool(msg.support_leg)
        self.swing_targets[msg.leg_name]["desired_normal_force_limit"] = float(msg.desired_normal_force_limit)
        self.swing_targets[msg.leg_name]["skirt_compression_target"] = float(msg.skirt_compression_target)

    def stance_wrench_callback(self, msg, leg_name):
        if leg_name not in self.stance_commands:
            return
        self.stance_commands[leg_name]["active"] = bool(msg.active)
        self.stance_commands[leg_name]["force"] = [
            float(msg.wrench.force.x),
            float(msg.wrench.force.y),
            float(msg.wrench.force.z),
        ]
        self.stance_commands[leg_name]["normal_force_limit"] = float(msg.normal_force_limit)
        self.stance_commands[leg_name]["required_adhesion_force"] = float(msg.required_adhesion_force)

    def _pose_stamp_to_sec(self, pose_msg):
        return pose_msg.header.stamp.to_sec() if pose_msg is not None else 0.0

    def _update_twist_from_mocap(self):
        if self.last_pose is None or self.prev_pose is None:
            return

        dt = self._pose_stamp_to_sec(self.last_pose) - self._pose_stamp_to_sec(self.prev_pose)
        if dt <= 1e-6:
            return

        current_position = self.last_pose.pose.position
        previous_position = self.prev_pose.pose.position
        linear_velocity = [
            (float(current_position.x) - float(previous_position.x)) / dt,
            (float(current_position.y) - float(previous_position.y)) / dt,
            (float(current_position.z) - float(previous_position.z)) / dt,
        ]
        self.filtered_linear_velocity = low_pass_blend(self.filtered_linear_velocity, linear_velocity, self.linear_velocity_alpha)

        current_orientation = quaternion_to_list(self.last_pose.pose.orientation)
        previous_orientation = quaternion_to_list(self.prev_pose.pose.orientation)
        delta_quaternion = quaternion_multiply(current_orientation, quaternion_conjugate(previous_orientation))
        angular_increment = quaternion_to_axis_angle(delta_quaternion)
        angular_velocity = [value / dt for value in angular_increment]
        self.filtered_angular_velocity = low_pass_blend(self.filtered_angular_velocity, angular_velocity, self.angular_velocity_alpha)

    def _recent_mocap_available(self):
        if self.last_pose is None:
            return False
        age = rospy.Time.now().to_sec() - self._pose_stamp_to_sec(self.last_pose)
        return age <= self.mocap_timeout_s

    def _leg_fan_current(self, leg_index):
        if leg_index < len(self.last_fan_currents):
            return float(self.last_fan_currents[leg_index])
        return 0.0

    @staticmethod
    def _interpolate_reference_curve(target_rpm, rpm_points, value_points):
        if not rpm_points or not value_points:
            return 0.0
        if len(rpm_points) != len(value_points):
            count = min(len(rpm_points), len(value_points))
            rpm_points = rpm_points[:count]
            value_points = value_points[:count]
        if len(rpm_points) == 1:
            return float(value_points[0])

        if target_rpm <= rpm_points[0]:
            left_index = 0
            right_index = 1
        elif target_rpm >= rpm_points[-1]:
            left_index = len(rpm_points) - 2
            right_index = len(rpm_points) - 1
        else:
            left_index = 0
            right_index = 1
            for index in range(1, len(rpm_points)):
                if target_rpm <= rpm_points[index]:
                    left_index = index - 1
                    right_index = index
                    break

        left_rpm = float(rpm_points[left_index])
        right_rpm = float(rpm_points[right_index])
        left_value = float(value_points[left_index])
        right_value = float(value_points[right_index])
        if abs(right_rpm - left_rpm) <= 1e-6:
            return left_value
        ratio = (float(target_rpm) - left_rpm) / (right_rpm - left_rpm)
        return left_value + ratio * (right_value - left_value)

    def _fan_adhesion_confidence(self, leg_name, leg_index):
        target_rpm = abs(float(self.last_fan_target_rpm.get(leg_name, 0.0)))
        if target_rpm < self.fan_min_active_rpm:
            return 0.0

        current_value = self._leg_fan_current(leg_index)
        attached_min = self._interpolate_reference_curve(
            target_rpm,
            self.fan_adhesion_reference_rpm,
            self.fan_attached_current_min_a,
        )
        attached_max = self._interpolate_reference_curve(
            target_rpm,
            self.fan_adhesion_reference_rpm,
            self.fan_attached_current_max_a,
        )
        detached_min = self._interpolate_reference_curve(
            target_rpm,
            self.fan_adhesion_reference_rpm,
            self.fan_detached_current_min_a,
        )
        detached_max = self._interpolate_reference_curve(
            target_rpm,
            self.fan_adhesion_reference_rpm,
            self.fan_detached_current_max_a,
        )

        attached_center = 0.5 * (attached_min + attached_max)
        detached_center = 0.5 * (detached_min + detached_max)
        boundary = 0.5 * (attached_max + detached_min)

        if current_value <= attached_max:
            span = max(attached_max - attached_min, 0.2)
            return clamp(1.0 - max(0.0, current_value - attached_center) / span, 0.0, 1.0)
        if current_value >= detached_min:
            span = max(detached_max - detached_min, 0.2)
            return clamp((detached_center - current_value) / span, 0.0, 1.0)

        transition_span = max(detached_min - attached_max, 1e-3)
        return clamp((boundary - current_value) / transition_span + 0.5, 0.0, 1.0)

    def _compression_from_required_adhesion_force(self, required_force_n):
        if required_force_n <= 0.0:
            return 0.0
        compression_mm = self._interpolate_reference_curve(
            float(required_force_n),
            self.adhesion_force_reference_n,
            self.adhesion_skirt_compression_mm,
        )
        return clamp(compression_mm / 1000.0, 0.0, self.max_skirt_compression_m)

    def _body_orientation_list(self):
        quaternion = quaternion_to_list(self.last_imu.orientation)
        if math.sqrt(sum([value * value for value in quaternion])) > 1e-6:
            return normalize_quaternion(quaternion)
        if self.last_pose is not None:
            return normalize_quaternion(quaternion_to_list(self.last_pose.pose.orientation))
        return [0.0, 0.0, 0.0, 1.0]

    def _joint_name_to_index_map(self):
        return {str(name): index for index, name in enumerate(self.last_joint_state.name)}

    def _nominal_universal_joint_center_position(self, leg_name):
        hip_yaw = self.leg_hip_yaw.get(leg_name, 0.0)
        cos_yaw = math.cos(hip_yaw)
        sin_yaw = math.sin(hip_yaw)
        hip_origin = [self.base_radius_m * cos_yaw, self.base_radius_m * sin_yaw, 0.0]
        return [
            hip_origin[0] + cos_yaw * self.nominal_x_m - sin_yaw * self.nominal_y_m,
            hip_origin[1] + sin_yaw * self.nominal_x_m + cos_yaw * self.nominal_y_m,
            self.nominal_universal_joint_center_z_m,
        ]

    def _foot_center_from_universal_joint_center(self, universal_joint_center, compression_estimate):
        del compression_estimate
        return [
            universal_joint_center[0] + self.contact_offset_sign * self.universal_joint_to_foot_center_rigid_offset_m * self.wall_normal_body[0],
            universal_joint_center[1] + self.contact_offset_sign * self.universal_joint_to_foot_center_rigid_offset_m * self.wall_normal_body[1],
            universal_joint_center[2] + self.contact_offset_sign * self.universal_joint_to_foot_center_rigid_offset_m * self.wall_normal_body[2],
        ]

    def _nominal_foot_center_position(self, leg_name):
        return self._foot_center_from_universal_joint_center(
            self._nominal_universal_joint_center_position(leg_name),
            self.nominal_skirt_compression_m,
        )

    def _forward_kinematics_leg_local(self, joint_vector):
        q1 = float(joint_vector[0])
        q2 = float(joint_vector[1])
        q3 = float(joint_vector[2])
        alpha = q2 - math.radians(90.0)
        radial_prime = self.l_tibia * math.cos(alpha) + self.l_a3 * math.cos(alpha + q3)
        p_z = self.l_tibia * math.sin(alpha) + self.l_a3 * math.sin(alpha + q3)
        radial_total = self.l_femur + radial_prime
        return [self.l_coxa + radial_total * math.cos(q1), radial_total * math.sin(q1), p_z]

    def _leg_joint_vector(self, leg_name, joint_index_map):
        joint_vector = []
        for motor_id in self.leg_motor_ids.get(leg_name, []):
            joint_index = joint_index_map.get(str(int(motor_id)))
            if joint_index is None or joint_index >= len(self.last_joint_state.position):
                return None
            joint_vector.append(float(self.last_joint_state.position[joint_index]))
        return joint_vector if len(joint_vector) == 3 else None

    def _estimate_universal_joint_center_positions(self):
        positions = []
        joint_index_map = self._joint_name_to_index_map()
        for leg_name in self.leg_names:
            joint_vector = self._leg_joint_vector(leg_name, joint_index_map)
            if joint_vector is None:
                positions.append(self._nominal_universal_joint_center_position(leg_name))
                continue
            local_position = self._forward_kinematics_leg_local(joint_vector)
            hip_yaw = self.leg_hip_yaw.get(leg_name, 0.0)
            cos_yaw = math.cos(hip_yaw)
            sin_yaw = math.sin(hip_yaw)
            hip_origin = [self.base_radius_m * cos_yaw, self.base_radius_m * sin_yaw, 0.0]
            positions.append([
                hip_origin[0] + cos_yaw * local_position[0] - sin_yaw * local_position[1],
                hip_origin[1] + sin_yaw * local_position[0] + cos_yaw * local_position[1],
                local_position[2],
            ])
        return positions

    def _estimate_universal_joint_center_velocities(self, universal_joint_center_positions):
        now_sec = rospy.Time.now().to_sec()
        velocities = [[0.0, 0.0, 0.0] for _ in universal_joint_center_positions]
        if self.prev_universal_joint_center_positions is None or self.prev_universal_joint_center_time_sec is None:
            self.prev_universal_joint_center_positions = [list(position) for position in universal_joint_center_positions]
            self.prev_universal_joint_center_time_sec = now_sec
            return velocities
        dt = now_sec - self.prev_universal_joint_center_time_sec
        if dt > 1e-6:
            for index, position in enumerate(universal_joint_center_positions):
                previous = self.prev_universal_joint_center_positions[index] if index < len(self.prev_universal_joint_center_positions) else position
                velocities[index] = [
                    (float(position[axis]) - float(previous[axis])) / dt
                    for axis in range(3)
                ]
        self.prev_universal_joint_center_positions = [list(position) for position in universal_joint_center_positions]
        self.prev_universal_joint_center_time_sec = now_sec
        return velocities

    def _aggregate_leg_scalar(self, motor_ids, values, index_map):
        total = 0.0
        count = 0
        for motor_id in motor_ids:
            joint_index = index_map.get(str(int(motor_id)))
            if joint_index is None or joint_index >= len(values):
                continue
            total += abs(float(values[joint_index]))
            count += 1
        return total, count

    def _leg_normal_force_limit(self, leg_name):
        stance_limit = float(self.stance_commands[leg_name]["normal_force_limit"])
        swing_limit = float(self.swing_targets[leg_name]["desired_normal_force_limit"])
        return max(self.default_normal_force_limit, stance_limit, swing_limit) if max(stance_limit, swing_limit) <= 0.0 else max(stance_limit, swing_limit)

    def _target_skirt_compression_m(self, leg_name):
        raw_target = float(self.swing_targets[leg_name].get("skirt_compression_target", 0.0))
        if raw_target <= 0.0:
            return 0.0
        if raw_target <= 1.0:
            return clamp(raw_target * self.max_skirt_compression_m, 0.0, self.max_skirt_compression_m)
        if raw_target <= self.max_skirt_compression_m:
            return clamp(raw_target, 0.0, self.max_skirt_compression_m)
        return clamp(raw_target / 1000.0, 0.0, self.max_skirt_compression_m)

    def _required_skirt_compression_m(self, leg_name):
        target_compression = self._target_skirt_compression_m(leg_name)
        adhesion_force_compression = self._compression_from_required_adhesion_force(
            float(self.stance_commands[leg_name].get("required_adhesion_force", 0.0))
        )
        required_compression = max(target_compression, adhesion_force_compression)
        if required_compression <= 1e-6:
            return self.nominal_skirt_compression_m
        return clamp(required_compression, 0.0, self.max_skirt_compression_m)

    def _estimate_skirt_compression(self, leg_name, normal_cmd, planned_support, measured_contact):
        target_compression = self._required_skirt_compression_m(leg_name)
        if not (planned_support or measured_contact):
            return target_compression
        load_based = clamp(normal_cmd * self.normal_force_to_compression_gain_m_per_n, 0.0, self.max_skirt_compression_m)
        return clamp(max(target_compression, load_based), 0.0, self.max_skirt_compression_m)

    def _compression_ready(self, leg_name, compression_estimate_m):
        required_compression = self._required_skirt_compression_m(leg_name)
        if required_compression <= 1e-6:
            return True
        return compression_estimate_m >= self.compression_ready_ratio * required_compression

    def _compression_progress(self, leg_name, compression_estimate_m):
        required_compression = self._required_skirt_compression_m(leg_name)
        if required_compression <= 1e-6:
            return 1.0
        return clamp(compression_estimate_m / required_compression, 0.0, 1.0)

    def _estimate_foot_center_positions(self, universal_joint_center_positions, skirt_compression_estimates):
        contact_positions = []
        for leg_index, universal_joint_center in enumerate(universal_joint_center_positions):
            compression_estimate = skirt_compression_estimates[leg_index] if leg_index < len(skirt_compression_estimates) else 0.0
            contact_positions.append(self._foot_center_from_universal_joint_center(universal_joint_center, compression_estimate))
        return contact_positions

    def _split_force_components(self, force_vector):
        normal_projection = vector_dot(force_vector, self.wall_normal_body)
        normal_cmd = abs(normal_projection)
        tangential_vector = [
            float(force_vector[index]) - normal_projection * self.wall_normal_body[index]
            for index in range(3)
        ]
        tangential_cmd = vector_norm(tangential_vector)
        return normal_cmd, tangential_cmd

    def _update_recent_stable_foot_center_positions(self, foot_center_positions, adhesion_mask, contact_confidence):
        updated = []
        for leg_index, foot_center in enumerate(foot_center_positions):
            previous = self.recent_stable_foot_center_positions[leg_index] if leg_index < len(self.recent_stable_foot_center_positions) else list(foot_center)
            stable = leg_index < len(adhesion_mask) and bool(adhesion_mask[leg_index]) and leg_index < len(contact_confidence) and float(contact_confidence[leg_index]) >= self.stable_contact_confidence_threshold
            if stable:
                updated.append(low_pass_blend(previous, foot_center, self.recent_contact_position_alpha))
            else:
                updated.append(list(previous))
        self.recent_stable_foot_center_positions = updated
        return updated

    def _update_position_estimate(self, linear_velocity_world):
        now_sec = rospy.Time.now().to_sec()
        if self.use_external_pose_estimate and self._recent_mocap_available():
            self.integrated_position_world = [
                float(self.last_pose.pose.position.x),
                float(self.last_pose.pose.position.y),
                float(self.last_pose.pose.position.z),
            ]
            self.last_publish_time_sec = now_sec
            return list(self.integrated_position_world)
        if self.last_publish_time_sec is None:
            self.last_publish_time_sec = now_sec
            return list(self.integrated_position_world)
        dt = max(0.0, now_sec - self.last_publish_time_sec)
        self.last_publish_time_sec = now_sec
        self.integrated_position_world = vector_add(self.integrated_position_world, vector_scale(linear_velocity_world, dt))
        return list(self.integrated_position_world)

    def _ekf_universal_joint_center_slice(self, leg_index):
        start = 6 + 3 * leg_index
        return slice(start, start + 3)

    def _measurement_noise_from_confidence(self, contact_confidence, contact_std, swing_std):
        blended_std = (1.0 - contact_confidence) * swing_std + contact_confidence * contact_std
        return max(blended_std * blended_std, 1e-9)

    def _external_position_world(self):
        if not (self.use_external_pose_estimate and self._recent_mocap_available()):
            return None
        return [
            float(self.last_pose.pose.position.x),
            float(self.last_pose.pose.position.y),
            float(self.last_pose.pose.position.z),
        ]

    def _reset_ekf_state(self, position_world, linear_velocity_world, universal_joint_center_positions, body_orientation, timestamp_sec):
        self.ekf_state.fill(0.0)
        self.ekf_state[0:3, 0] = np.array(position_world, dtype=float)
        self.ekf_state[3:6, 0] = np.array(linear_velocity_world, dtype=float)
        for leg_index, universal_joint_center in enumerate(universal_joint_center_positions):
            universal_joint_center_world = vector_add(position_world, body_to_world(universal_joint_center, body_orientation))
            self.ekf_state[self._ekf_universal_joint_center_slice(leg_index), 0] = np.array(universal_joint_center_world, dtype=float)
        self.ekf_covariance = np.eye(self.ekf_state_dim, dtype=float) * self.ekf_initial_covariance
        self.last_ekf_time_sec = timestamp_sec
        self.ekf_initialized = True
        self.integrated_position_world = list(position_world)

    def _predict_body_velocity_from_contacts(self, universal_joint_center_positions, universal_joint_center_velocities, contact_confidence):
        omega_body = [
            float(self.last_imu.angular_velocity.x),
            float(self.last_imu.angular_velocity.y),
            float(self.last_imu.angular_velocity.z),
        ]
        weighted_sum = [0.0, 0.0, 0.0]
        total_weight = 0.0
        for leg_index, universal_joint_center in enumerate(universal_joint_center_positions):
            weight = float(contact_confidence[leg_index]) if leg_index < len(contact_confidence) else 0.0
            if weight <= 1e-3:
                continue
            foot_velocity = universal_joint_center_velocities[leg_index] if leg_index < len(universal_joint_center_velocities) else [0.0, 0.0, 0.0]
            body_velocity_leg = vector_scale(vector_add(foot_velocity, cross(omega_body, universal_joint_center)), -1.0)
            weighted_sum = vector_add(weighted_sum, vector_scale(body_velocity_leg, weight))
            total_weight += weight
        if total_weight <= 1e-6:
            return list(self.filtered_linear_velocity)
        body_velocity = vector_scale(weighted_sum, 1.0 / total_weight)
        self.filtered_body_linear_velocity = low_pass_blend(self.filtered_body_linear_velocity, body_velocity, self.linear_velocity_alpha)
        world_velocity = body_to_world(self.filtered_body_linear_velocity, self._body_orientation_list())
        self.filtered_linear_velocity = low_pass_blend(self.filtered_linear_velocity, world_velocity, self.linear_velocity_alpha)
        return list(self.filtered_linear_velocity)

    def _build_ekf_process_noise(self, dt, contact_confidence):
        q_matrix = np.zeros((self.ekf_state_dim, self.ekf_state_dim), dtype=float)
        position_var = max((self.ekf_body_position_process_std_m * dt) ** 2, 1e-9)
        velocity_var = max(
            (self.ekf_body_velocity_process_std_mps * dt) ** 2 + (self.ekf_body_acceleration_process_std_mps2 * dt) ** 2,
            1e-9,
        )
        q_matrix[0:3, 0:3] = np.eye(3, dtype=float) * position_var
        q_matrix[3:6, 3:6] = np.eye(3, dtype=float) * velocity_var
        for leg_index in range(len(self.leg_names)):
            confidence = clamp(float(contact_confidence[leg_index]) if leg_index < len(contact_confidence) else 0.0, 0.0, 1.0)
            universal_joint_center_std = (1.0 - confidence) * self.ekf_swing_foot_process_std_m + confidence * self.ekf_contact_foot_process_std_m
            q_matrix[self._ekf_universal_joint_center_slice(leg_index), self._ekf_universal_joint_center_slice(leg_index)] = np.eye(3, dtype=float) * max((universal_joint_center_std * dt) ** 2, 1e-9)
        return q_matrix

    def _predict_ekf(self, dt, body_orientation, contact_confidence):
        if dt <= 0.0:
            return
        transition = np.eye(self.ekf_state_dim, dtype=float)
        transition[0:3, 3:6] = np.eye(3, dtype=float) * dt
        acceleration_body = [
            float(self.last_imu.linear_acceleration.x),
            float(self.last_imu.linear_acceleration.y),
            float(self.last_imu.linear_acceleration.z),
        ]
        acceleration_world = vector_add(body_to_world(acceleration_body, body_orientation), self.gravity_world)
        control = np.zeros((self.ekf_state_dim, 1), dtype=float)
        control[0:3, 0] = 0.5 * dt * dt * np.array(acceleration_world, dtype=float)
        control[3:6, 0] = dt * np.array(acceleration_world, dtype=float)
        self.ekf_state = transition.dot(self.ekf_state) + control
        process_noise = self._build_ekf_process_noise(dt, contact_confidence)
        self.ekf_covariance = transition.dot(self.ekf_covariance).dot(transition.T) + process_noise

    def _build_ekf_measurements(self, universal_joint_center_positions, universal_joint_center_velocities, contact_confidence, body_orientation):
        observation_rows = []
        measurements = []
        measurement_noise = []
        omega_body = [
            float(self.last_imu.angular_velocity.x),
            float(self.last_imu.angular_velocity.y),
            float(self.last_imu.angular_velocity.z),
        ]

        for leg_index, universal_joint_center in enumerate(universal_joint_center_positions):
            confidence = clamp(float(contact_confidence[leg_index]) if leg_index < len(contact_confidence) else 0.0, 0.0, 1.0)
            position_row = np.zeros((3, self.ekf_state_dim), dtype=float)
            position_row[:, 0:3] = -np.eye(3, dtype=float)
            position_row[:, self._ekf_universal_joint_center_slice(leg_index)] = np.eye(3, dtype=float)
            observation_rows.append(position_row)
            measurements.append(np.array(body_to_world(universal_joint_center, body_orientation), dtype=float).reshape(3, 1))
            measurement_noise.append(
                np.eye(3, dtype=float) * self._measurement_noise_from_confidence(
                    confidence,
                    self.ekf_contact_position_measurement_std_m,
                    self.ekf_swing_position_measurement_std_m,
                )
            )

            foot_velocity = universal_joint_center_velocities[leg_index] if leg_index < len(universal_joint_center_velocities) else [0.0, 0.0, 0.0]
            contact_velocity_body = vector_scale(vector_add(foot_velocity, cross(omega_body, universal_joint_center)), -1.0)
            velocity_row = np.zeros((3, self.ekf_state_dim), dtype=float)
            velocity_row[:, 3:6] = np.eye(3, dtype=float)
            observation_rows.append(velocity_row)
            measurements.append(np.array(body_to_world(contact_velocity_body, body_orientation), dtype=float).reshape(3, 1))
            measurement_noise.append(
                np.eye(3, dtype=float) * self._measurement_noise_from_confidence(
                    confidence,
                    self.ekf_contact_velocity_measurement_std_mps,
                    self.ekf_swing_velocity_measurement_std_mps,
                )
            )

        external_position_world = self._external_position_world()
        if external_position_world is not None:
            external_row = np.zeros((3, self.ekf_state_dim), dtype=float)
            external_row[:, 0:3] = np.eye(3, dtype=float)
            observation_rows.append(external_row)
            measurements.append(np.array(external_position_world, dtype=float).reshape(3, 1))
            measurement_noise.append(np.eye(3, dtype=float) * (self.ekf_external_position_measurement_std_m ** 2))

        if not observation_rows:
            return None, None, None
        observation_matrix = np.vstack(observation_rows)
        measurement_vector = np.vstack(measurements)
        noise_matrix = np.zeros((measurement_vector.shape[0], measurement_vector.shape[0]), dtype=float)
        cursor = 0
        for noise_block in measurement_noise:
            block_size = noise_block.shape[0]
            noise_matrix[cursor:cursor + block_size, cursor:cursor + block_size] = noise_block
            cursor += block_size
        return observation_matrix, measurement_vector, noise_matrix

    def _update_ekf(self, observation_matrix, measurement_vector, noise_matrix):
        if observation_matrix is None:
            return
        innovation = measurement_vector - observation_matrix.dot(self.ekf_state)
        innovation_covariance = observation_matrix.dot(self.ekf_covariance).dot(observation_matrix.T) + noise_matrix
        innovation_inverse = np.linalg.pinv(innovation_covariance)
        kalman_gain = self.ekf_covariance.dot(observation_matrix.T).dot(innovation_inverse)
        identity = np.eye(self.ekf_state_dim, dtype=float)
        self.ekf_state = self.ekf_state + kalman_gain.dot(innovation)
        joseph_factor = identity - kalman_gain.dot(observation_matrix)
        self.ekf_covariance = joseph_factor.dot(self.ekf_covariance).dot(joseph_factor.T) + kalman_gain.dot(noise_matrix).dot(kalman_gain.T)
        self.ekf_covariance = 0.5 * (self.ekf_covariance + self.ekf_covariance.T)

    def _estimate_body_state_ekf(self, universal_joint_center_positions, universal_joint_center_velocities, contact_confidence):
        now_sec = rospy.Time.now().to_sec()
        body_orientation = self._body_orientation_list()
        predicted_velocity_world = self._predict_body_velocity_from_contacts(universal_joint_center_positions, universal_joint_center_velocities, contact_confidence)
        external_position_world = self._external_position_world()
        if not self.ekf_initialized:
            initial_position_world = list(external_position_world) if external_position_world is not None else list(self.integrated_position_world)
            self._reset_ekf_state(initial_position_world, predicted_velocity_world, universal_joint_center_positions, body_orientation, now_sec)
        dt = 0.0 if self.last_ekf_time_sec is None else max(0.0, min(now_sec - self.last_ekf_time_sec, 0.2))
        self.last_ekf_time_sec = now_sec
        self._predict_ekf(dt, body_orientation, contact_confidence)
        observation_matrix, measurement_vector, noise_matrix = self._build_ekf_measurements(
            universal_joint_center_positions,
            universal_joint_center_velocities,
            contact_confidence,
            body_orientation,
        )
        self._update_ekf(observation_matrix, measurement_vector, noise_matrix)
        position_world = self.ekf_state[0:3, 0].tolist()
        linear_velocity_world = self.ekf_state[3:6, 0].tolist()
        if external_position_world is not None:
            position_error = vector_norm([
                position_world[index] - external_position_world[index]
                for index in range(3)
            ])
            if position_error >= self.ekf_reset_position_jump_m:
                self._reset_ekf_state(external_position_world, linear_velocity_world, universal_joint_center_positions, body_orientation, now_sec)
                position_world = self.ekf_state[0:3, 0].tolist()
                linear_velocity_world = self.ekf_state[3:6, 0].tolist()
        self.integrated_position_world = list(position_world)
        self.filtered_linear_velocity = low_pass_blend(self.filtered_linear_velocity, linear_velocity_world, self.linear_velocity_alpha)
        return position_world, linear_velocity_world

    def _estimate_leg_support_and_adhesion(self):
        plan_support_mask = []
        measured_contact_mask = []
        wall_touch_mask = []
        compression_ready_mask = []
        preload_ready_mask = []
        early_contact_mask = []
        contact_mask = []
        support_mask = []
        adhesion_mask = []
        attachment_ready_mask = []
        seal_confidence = []
        leg_torque_sum = []
        leg_torque_contact_confidence = []
        slip_risk = []
        normal_force_limit = []
        skirt_compression_estimate = []
        contact_confidence = []

        joint_index_map = self._joint_name_to_index_map()
        joint_torques = list(self.last_joint_state.effort)
        joint_currents = list(self.last_joint_currents)

        for leg_index, leg_name in enumerate(self.leg_names):
            stance_cmd = self.stance_commands[leg_name]
            requested_support = self.body_reference_support_mask[leg_index] if leg_index < len(self.body_reference_support_mask) else True

            leg_motor_ids = self.leg_motor_ids.get(leg_name, [])
            current_sum, current_count = self._aggregate_leg_scalar(leg_motor_ids, joint_currents, joint_index_map)
            torque_sum, torque_count = self._aggregate_leg_scalar(leg_motor_ids, joint_torques, joint_index_map)

            measured_contact = (current_count > 0 and current_sum >= self.current_contact_threshold_a * max(current_count, 1)) or (
                torque_count > 0 and torque_sum > self.torque_contact_threshold_nm * max(torque_count, 1)
            )

            force_vector = list(stance_cmd["force"])
            normal_cmd, tangential_cmd = self._split_force_components(force_vector)
            force_limit = self._leg_normal_force_limit(leg_name)
            adhesion_confidence = self._fan_adhesion_confidence(leg_name, leg_index)
            measured_torque_confidence = clamp(
                torque_sum / max(self.torque_contact_threshold_nm * max(torque_count, 1), 1e-6),
                0.0,
                1.0,
            ) if torque_count > 0 else 0.0

            planned_support = bool(requested_support)
            compression_estimate_m = self._estimate_skirt_compression(leg_name, normal_cmd, planned_support, measured_contact)
            normal_support = normal_cmd >= self.support_force_threshold_n
            compression_ready = measured_contact or self._compression_ready(leg_name, compression_estimate_m)
            compression_progress = self._compression_progress(leg_name, compression_estimate_m)
            swing_phase = self._leg_swing_phase(leg_name, planned_support)
            early_contact = (not planned_support) and measured_contact and swing_phase >= self.early_contact_phase_threshold
            wall_touch = (not planned_support) and measured_contact
            preload_ready = wall_touch and compression_ready
            seal_value = max(float(adhesion_confidence), float(compression_progress))
            attachment_ready = preload_ready and adhesion_confidence >= self.fan_adhesion_confidence_threshold
            confidence_value = max(
                1.0 if measured_contact else 0.0,
                1.0 if attachment_ready else 0.0,
                0.8 if preload_ready else 0.0,
                0.6 if early_contact else 0.0,
                clamp(normal_cmd / max(self.support_force_threshold_n, 1e-6), 0.0, 1.0) if planned_support else 0.0,
                adhesion_confidence,
                compression_progress,
            )

            if planned_support:
                actual_contact = measured_contact or normal_support or attachment_ready
            else:
                actual_contact = early_contact or preload_ready or attachment_ready

            support_value = actual_contact
            adhesion_value = attachment_ready and support_value and force_limit >= self.adhesion_force_threshold_n

            denom = max(self.wall_mu * max(normal_cmd, 1e-3), 1e-3)
            slip_ratio = tangential_cmd / denom
            if not support_value:
                slip_ratio = 0.0

            plan_support_mask.append(bool(planned_support))
            measured_contact_mask.append(bool(measured_contact))
            wall_touch_mask.append(bool(wall_touch))
            compression_ready_mask.append(bool(compression_ready))
            preload_ready_mask.append(bool(preload_ready))
            early_contact_mask.append(bool(early_contact))
            contact_mask.append(bool(actual_contact))
            support_mask.append(bool(support_value))
            adhesion_mask.append(bool(adhesion_value))
            attachment_ready_mask.append(bool(attachment_ready))
            seal_confidence.append(float(seal_value))
            leg_torque_sum.append(float(torque_sum))
            leg_torque_contact_confidence.append(float(measured_torque_confidence))
            normal_force_limit.append(float(force_limit))
            slip_risk.append(float(clamp(slip_ratio / max(self.slip_warning_ratio, 1e-3), 0.0, 1.0)))
            skirt_compression_estimate.append(float(compression_estimate_m))
            contact_confidence.append(float(confidence_value))

        return (
            plan_support_mask,
            measured_contact_mask,
            wall_touch_mask,
            compression_ready_mask,
            preload_ready_mask,
            early_contact_mask,
            contact_mask,
            support_mask,
            adhesion_mask,
            attachment_ready_mask,
            seal_confidence,
            leg_torque_sum,
            leg_torque_contact_confidence,
            normal_force_limit,
            slip_risk,
            skirt_compression_estimate,
            contact_confidence,
        )

    def _build_twist_message(self, state_msg, linear_velocity_world):
        state_msg.twist.linear.x = float(linear_velocity_world[0])
        state_msg.twist.linear.y = float(linear_velocity_world[1])
        state_msg.twist.linear.z = float(linear_velocity_world[2])
        state_msg.twist.angular.x = float(self.last_imu.angular_velocity.x) if self.last_imu is not None else self.filtered_angular_velocity[0]
        state_msg.twist.angular.y = float(self.last_imu.angular_velocity.y) if self.last_imu is not None else self.filtered_angular_velocity[1]
        state_msg.twist.angular.z = float(self.last_imu.angular_velocity.z) if self.last_imu is not None else self.filtered_angular_velocity[2]

    def _body_orientation_message(self):
        quaternion = quaternion_to_list(self.last_imu.orientation)
        if math.sqrt(sum([value * value for value in quaternion])) > 1e-6:
            return self.last_imu.orientation
        if self.last_pose is not None:
            return self.last_pose.pose.orientation
        identity = Imu().orientation
        identity.w = 1.0
        return identity

    def _build_fused_imu(self):
        fused_imu = Imu()
        fused_imu.header.stamp = rospy.Time.now()
        fused_imu.header.frame_id = self.last_imu.header.frame_id if self.last_imu.header.frame_id else "imu"
        fused_imu.orientation = self._body_orientation_message()
        fused_imu.orientation_covariance = list(self.last_imu.orientation_covariance)
        fused_imu.angular_velocity = self.last_imu.angular_velocity
        fused_imu.angular_velocity_covariance = list(self.last_imu.angular_velocity_covariance)
        fused_imu.linear_acceleration = self.last_imu.linear_acceleration
        fused_imu.linear_acceleration_covariance = list(self.last_imu.linear_acceleration_covariance)
        return fused_imu

    def publish_state(self, _event):
        msg = EstimatedState()
        msg.header.stamp = rospy.Time.now()
        universal_joint_center_positions = self._estimate_universal_joint_center_positions()
        universal_joint_center_velocities = self._estimate_universal_joint_center_velocities(universal_joint_center_positions)
        (
            plan_support_mask,
            measured_contact_mask,
            wall_touch_mask,
            compression_ready_mask,
            preload_ready_mask,
            early_contact_mask,
            contact_mask,
            support_mask,
            adhesion_mask,
            attachment_ready_mask,
            seal_confidence,
            leg_torque_sum,
            leg_torque_contact_confidence,
            normal_force_limit,
            slip_risk,
            skirt_compression_estimate,
            contact_confidence,
        ) = self._estimate_leg_support_and_adhesion()
        position_world, linear_velocity_world = self._estimate_body_state_ekf(
            universal_joint_center_positions,
            universal_joint_center_velocities,
            contact_confidence,
        )
        foot_center_positions = self._estimate_foot_center_positions(universal_joint_center_positions, skirt_compression_estimate)
        msg.pose.position.x = float(position_world[0])
        msg.pose.position.y = float(position_world[1])
        msg.pose.position.z = float(position_world[2])
        msg.pose.orientation = self._body_orientation_message()
        self._build_twist_message(msg, linear_velocity_world)
        msg.imu = self._build_fused_imu()
        msg.joint_currents = list(self.last_joint_currents)
        msg.joint_torques_est = list(self.last_joint_state.effort)
        msg.universal_joint_center_positions = [Point(position[0], position[1], position[2]) for position in universal_joint_center_positions]
        msg.foot_center_positions = [Point(position[0], position[1], position[2]) for position in foot_center_positions]
        recent_stable_foot_center_positions = self._update_recent_stable_foot_center_positions(foot_center_positions, adhesion_mask, contact_confidence)
        msg.recent_stable_foot_center_positions = [Point(position[0], position[1], position[2]) for position in recent_stable_foot_center_positions]
        msg.skirt_compression_estimate = skirt_compression_estimate
        msg.plan_support_mask = plan_support_mask
        msg.measured_contact_mask = measured_contact_mask
        msg.wall_touch_mask = wall_touch_mask
        msg.compression_ready_mask = compression_ready_mask
        msg.preload_ready_mask = preload_ready_mask
        msg.early_contact_mask = early_contact_mask
        msg.contact_mask = contact_mask
        msg.support_mask = support_mask
        msg.adhesion_mask = adhesion_mask
        msg.attachment_ready_mask = attachment_ready_mask
        msg.seal_confidence = seal_confidence
        msg.leg_torque_sum = leg_torque_sum
        msg.leg_torque_contact_confidence = leg_torque_contact_confidence
        msg.normal_force_limit = normal_force_limit
        msg.slip_risk = slip_risk
        msg.contact_confidence = contact_confidence
        msg.fan_current = [float(self._leg_fan_current(index)) for index in range(len(self.leg_names))]
        self.pub.publish(msg)


if __name__ == "__main__":
    StateEstimator()
    rospy.spin()