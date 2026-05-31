#!/usr/bin/env python

import math

import numpy as np

import rospy
from climbing_msgs.msg import BodyReference, EstimatedState, LegCenterCommand
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, String
from workspace_guard import workspace_guard


def clamp(value, lower_bound, upper_bound):
    return max(lower_bound, min(upper_bound, value))


def bezier4(points, phase):
    phase = clamp(phase, 0.0, 1.0)
    one_minus = 1.0 - phase
    coefficients = [
        one_minus ** 4,
        4.0 * phase * (one_minus ** 3),
        6.0 * (phase ** 2) * (one_minus ** 2),
        4.0 * (phase ** 3) * one_minus,
        phase ** 4,
    ]
    return sum([coefficients[index] * points[index] for index in range(5)])


def bezier4_derivative(points, phase):
    phase = clamp(phase, 0.0, 1.0)
    derivative_points = []
    for index in range(4):
        derivative_points.append(4.0 * (points[index + 1] - points[index]))

    one_minus = 1.0 - phase
    coefficients = [
        one_minus ** 3,
        3.0 * phase * (one_minus ** 2),
        3.0 * (phase ** 2) * one_minus,
        phase ** 3,
    ]
    return sum([coefficients[index] * derivative_points[index] for index in range(4)])


def smoothstep5(phase):
    phase = clamp(phase, 0.0, 1.0)
    return phase * phase * phase * (10.0 + phase * (-15.0 + 6.0 * phase))


def vector_add(lhs, rhs):
    return [lhs[index] + rhs[index] for index in range(len(lhs))]


def vector_sub(lhs, rhs):
    return [lhs[index] - rhs[index] for index in range(len(lhs))]


def vector_scale(vector, scalar):
    return [scalar * value for value in vector]


def vector_mul(lhs, rhs):
    return [lhs[index] * rhs[index] for index in range(len(lhs))]


def vector_dot(lhs, rhs):
    return sum([lhs[index] * rhs[index] for index in range(len(lhs))])


def vector_norm(vector):
    return math.sqrt(max(vector_dot(vector, vector), 0.0))


def normalize_vector(vector, fallback):
    norm = vector_norm(vector)
    if norm < 1e-9:
        norm = vector_norm(fallback)
        if norm < 1e-9:
            return [0.0, 0.0, 1.0]
        return [value / norm for value in fallback]
    return [value / norm for value in vector]


def quaternion_to_list(quaternion_msg):
    return [
        float(quaternion_msg.x),
        float(quaternion_msg.y),
        float(quaternion_msg.z),
        float(quaternion_msg.w),
    ]


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum([value * value for value in quaternion]))
    if norm < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in quaternion]


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


def rotate_vector(quaternion, vector):
    q = normalize_quaternion(quaternion)
    pure = [vector[0], vector[1], vector[2], 0.0]
    rotated = quaternion_multiply(quaternion_multiply(q, pure), quaternion_conjugate(q))
    return rotated[:3]


def world_to_body(vector_world, body_orientation):
    return rotate_vector(quaternion_conjugate(normalize_quaternion(body_orientation)), vector_world)


def quaternion_to_axis_angle(quaternion):
    qx, qy, qz, qw = normalize_quaternion(quaternion)
    vector_norm = math.sqrt(qx * qx + qy * qy + qz * qz)
    if vector_norm < 1e-9:
        return [0.0, 0.0, 0.0]
    angle = 2.0 * math.atan2(vector_norm, qw)
    axis = [qx / vector_norm, qy / vector_norm, qz / vector_norm]
    return [angle * axis[0], angle * axis[1], angle * axis[2]]


def cross(lhs, rhs):
    return [
        lhs[1] * rhs[2] - lhs[2] * rhs[1],
        lhs[2] * rhs[0] - lhs[0] * rhs[2],
        lhs[0] * rhs[1] - lhs[1] * rhs[0],
    ]


class SwingLegController(object):
    PHASE_SUPPORT = "SUPPORT"
    PHASE_LIFT_SWING = "LIFT_SWING"
    PHASE_PRELOAD = "PRELOAD"
    PHASE_ADMIT = "ADMIT"
    PHASE_RELEASE_WAIT = "RELEASE_WAIT"
    PHASE_LIFT = "LIFT"
    PHASE_TRANSFER = "TRANSFER"

    def __init__(self):
        rospy.init_node("swing_leg_controller", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/swing_leg_controller/" + name, default))

        self.leg_names = [str(name) for name in get_cfg("leg_names", ["lf", "rf", "rr", "lr"])]
        self.rate_hz = float(get_cfg("publish_rate_hz", 50.0))
        self.swing_duration_s = max(float(get_cfg("swing_duration_s", 0.65)), 0.1)
        self.clearance_m = float(get_cfg("clearance_m", 0.035))
        self.raibert_height_gain = float(get_cfg("raibert_height_gain", 1.0))
        self.raibert_velocity_gain = float(get_cfg("raibert_velocity_gain", 1.0))
        self.fixed_swing_distance_m = float(get_cfg("fixed_swing_distance_m", 0.0))
        self.stride_length_m = float(get_cfg("stride_length_m", 0.0))
        if self.stride_length_m > 0.0:
            self._swing_distance_m = 0.75 * self.stride_length_m   # 3L/4
            self._body_advance_per_swing_m = 0.25 * self.stride_length_m  # L/4
        else:
            self._swing_distance_m = 0.0
            self._body_advance_per_swing_m = 0.0
        self.foot_delta_x_limit = float(get_cfg("foot_delta_x_limit_m", 0.10))
        self.foot_delta_y_limit = float(get_cfg("foot_delta_y_limit_m", 0.10))
        self.max_position_offset = [float(value) for value in get_cfg("max_position_offset_m", [0.45, 0.35, 0.45])]
        self.feedforward_twist_gain = [float(value) for value in get_cfg("feedforward_twist_gain", [0.16, 0.16, 0.0])]
        self.body_position_gain = [float(value) for value in get_cfg("body_position_gain", [0.70, 0.70, 0.35])]
        self.body_velocity_gain = [float(value) for value in get_cfg("body_velocity_gain", [0.12, 0.12, 0.08])]
        self.body_angular_position_gain = [float(value) for value in get_cfg("body_angular_position_gain", [0.10, 0.10, 0.06])]
        self.body_angular_velocity_gain = [float(value) for value in get_cfg("body_angular_velocity_gain", [0.02, 0.02, 0.01])]
        self.tangential_align_duration_s = max(float(get_cfg("tangential_align_duration_s", 0.28)), 0.05)
        self.lift_duration_s = max(float(get_cfg("lift_duration_s", 0.20)), 0.05)
        self.transfer_duration_s = max(float(get_cfg("transfer_duration_s", self.tangential_align_duration_s)), 0.05)
        self.preload_duration_s = max(float(get_cfg("preload_duration_s", 0.16)), 0.05)
        self.compliant_settle_timeout_s = max(float(get_cfg("compliant_settle_timeout_s", 0.40)), 0.05)
        self.preload_extra_normal_m = max(float(get_cfg("preload_extra_normal_m", 0.005)), 0.0)
        self.fan_attach_sink_m = max(float(get_cfg("fan_attach_sink_m", 0.008)), 0.0)
        self.fan_off_dwell_s = max(float(get_cfg("fan_off_dwell_s", 1.0)), 0.0)
        self.fan_off_rpm_threshold = float(get_cfg("fan_off_rpm_threshold", 10000.0))
        self.fan_recovery_rpm_threshold = float(get_cfg("fan_recovery_rpm_threshold", 30000.0))
        self.normal_alignment_tolerance_m = max(float(get_cfg("normal_alignment_tolerance_m", 0.002)), 1e-4)
        self.preload_normal_force_limit_n = max(float(get_cfg("preload_normal_force_limit_n", 15.0)), 0.0)
        self.attach_normal_force_limit_n = max(float(get_cfg("attach_normal_force_limit_n", 25.0)), 0.0)
        self.compliant_velocity_limit = [float(value) for value in get_cfg("compliant_velocity_limit_mps", [0.03, 0.03, 0.035])]
        self.compliant_normal_velocity_limit_mps = max(
            float(get_cfg("compliant_normal_velocity_limit_mps", max(self.compliant_velocity_limit))),
            0.0,
        )
        self.compliant_normal_offset_limit_m = max(
            float(get_cfg("compliant_normal_offset_limit_m", self.fan_attach_sink_m)),
            0.0,
        )
        self.compliant_admittance_mass = max(float(get_cfg("compliant_admittance_mass", 0.35)), 1e-3)
        self.compliant_admittance_damping = max(float(get_cfg("compliant_admittance_damping", 18.0)), 0.0)
        self.compliant_admittance_stiffness = max(float(get_cfg("compliant_admittance_stiffness", 120.0)), 0.0)
        self.compliant_force_filter_alpha = clamp(float(get_cfg("compliant_force_filter_alpha", 0.35)), 0.0, 1.0)
        self.compliant_force_deadband_n = max(float(get_cfg("compliant_force_deadband_n", 1.0)), 0.0)
        self.compliant_force_limit_n = max(float(get_cfg("compliant_force_limit_n", 80.0)), self.compliant_force_deadband_n)
        self.compliant_force_sign = float(get_cfg("compliant_force_sign", 1.0))
        self.hold_position_states = set(
            str(value) for value in get_cfg("hold_position_states", ["INIT"])
        )
        self.use_contact_feedback = bool(get_cfg("use_contact_feedback", True))
        self.swing_lift_normal_m = max(float(get_cfg("swing_lift_normal_m", self.clearance_m)), 0.0)
        self.jacobian_delta_rad = max(float(get_cfg("jacobian_delta_rad", 1e-3)), 1e-5)
        self.workspace_check_enabled = bool(get_cfg("workspace_check_enabled", True))
        self.workspace_check_mode = str(get_cfg("workspace_check_mode", "per_cycle")).strip().lower()
        self.workspace_clamp_max_iter = max(int(get_cfg("workspace_clamp_max_iter", 10)), 1)
        self.workspace_warn_throttle_s = max(float(get_cfg("workspace_warn_throttle_s", 2.0)), 0.1)
        self.workspace_fk_tolerance_m = max(float(get_cfg("workspace_fk_tolerance_m", 0.002)), 1e-5)

        legacy_nominal_z_mm = float(rospy.get_param("/gait_controller/nominal_z", -299.2))
        self.nominal_x_m = float(rospy.get_param("/gait_controller/nominal_x", 118.75)) / 1000.0
        self.nominal_y_m = float(rospy.get_param("/gait_controller/nominal_y", 0.0)) / 1000.0
        self.l_coxa_m = float(rospy.get_param("/gait_controller/link_coxa", 44.75)) / 1000.0
        self.l_femur_m = float(rospy.get_param("/gait_controller/link_femur", 74.0)) / 1000.0
        self.l_tibia_m = float(rospy.get_param("/gait_controller/link_tibia", 150.0)) / 1000.0
        self.l_a3_m = float(rospy.get_param("/gait_controller/link_a3", 41.5)) / 1000.0
        self.nominal_z_m = float(
            rospy.get_param(
                "/gait_controller/nominal_universal_joint_center_z",
                legacy_nominal_z_mm,
            )
        ) / 1000.0
        self.operating_x_m = float(
            rospy.get_param(
                "/gait_controller/operating_universal_joint_center_x",
                self.nominal_x_m * 1000.0,
            )
        ) / 1000.0
        self.operating_y_m = float(
            rospy.get_param(
                "/gait_controller/operating_universal_joint_center_y",
                self.nominal_y_m * 1000.0,
            )
        ) / 1000.0
        self.operating_z_m = float(
            rospy.get_param(
                "/gait_controller/operating_universal_joint_center_z",
                self.nominal_z_m * 1000.0,
            )
        ) / 1000.0
        self.base_radius_m = float(rospy.get_param("/gait_controller/base_radius", 203.06)) / 1000.0
        self.joint_limit_deg = rospy.get_param(
            "/gait_controller/joint_limit_deg",
            {"j1": [-90.0, 90.0], "j2": [-10.0, 190.0], "j3": [-100.0, 100.0]},
        )
        self.leg_yaw_rad = self._build_leg_yaw_map()
        self.leg_to_motors = self._build_leg_motor_map()
        default_joint_order = []
        for leg_name in self.leg_names:
            default_joint_order.extend(self.leg_to_motors.get(leg_name, []))
        joint_order = rospy.get_param(
            "/jetson/dynamixel_bridge/ordered_motor_ids",
            rospy.get_param("/dynamixel_bridge/ordered_motor_ids", default_joint_order),
        )
        self.joint_name_to_index = {str(int(motor_id)): index for index, motor_id in enumerate(joint_order)}
        self.wall_normal_body = normalize_vector(
            [float(value) for value in get_cfg("wall_normal_body", rospy.get_param("/wall/normal_body", [0.0, 0.0, 1.0]))],
            [0.0, 0.0, 1.0],
        )

        self.body_reference = BodyReference()
        self.estimated_state = EstimatedState()
        self.have_body_reference = False
        self.have_estimated_state = False
        self._body_x_offset = 0.0  # stride gait: accumulated L/4 body advance
        self._body_y_offset = 0.0
        self._inflight_body_advance_m = 0.0
        self.last_update_time = None
        self.mission_state = "INIT"
        self.swing_phase_start = {}
        self._fan_off_request_time = {}  # leg_name -> time when fan-off was requested
        self._latest_fan_rpm = [0.0, 0.0, 0.0, 0.0]  # [lf, rf, rr, lr]
        self._fan_rpm_timeout_s = 3.0  # max wait for RPM check fallback
        self.swing_states = {}
        for leg_name in self.leg_names:
            nominal = self._operating_center_command(leg_name)
            self.swing_phase_start[leg_name] = None
            self.swing_states[leg_name] = {
                "position": list(nominal),
                "velocity": [0.0, 0.0, 0.0],
                "support_target": list(nominal),
                "start": list(nominal),
                "target": list(nominal),
                "support": True,
                "phase": self.PHASE_SUPPORT,
                "phase_started_at": None,
                "lift_target": list(nominal),
                "lift_swing_target": list(nominal),
                "preload_target": list(nominal),
                "attach_target": list(nominal),
                "compliant_joint_torque_bias": [0.0, 0.0, 0.0],
                "compliant_force_estimate": 0.0,
                "compliant_normal_offset": 0.0,
                "compliant_normal_velocity": 0.0,
                "last_joint_vector": [0.0, 0.0, 0.0],
                "attach_confirmed_time": None,
                "support_world_x": None,
                "support_world_y": None,
                "transfer_fraction": 0.0,
                "transfer_committed": False,
                "workspace_clamped": 0.0,
                "workspace_margin_m": 0.0,
                "workspace_check_us": 0.0,
                "workspace_last_joint_deg": [0.0, 0.0, 0.0],
            }

        self.pub = rospy.Publisher("/control/swing_leg_target", LegCenterCommand, queue_size=50)
        self.diagnostic_pubs = {
            leg_name: rospy.Publisher(
                "/control/swing_leg_diag/" + leg_name, Float32MultiArray, queue_size=20
            )
            for leg_name in self.leg_names
        }
        self.phase_id_map = {
            self.PHASE_SUPPORT: 0,
            self.PHASE_LIFT_SWING: 1,
            self.PHASE_PRELOAD: 2,
            self.PHASE_ADMIT: 3,
            self.PHASE_RELEASE_WAIT: 4,
            self.PHASE_LIFT: 5,
            self.PHASE_TRANSFER: 6,
        }
        self.diagnostic_field_labels = [
            "leg_index",
            "phase_id",
            "phase_elapsed_s",
            "cmd_normal_from_nominal_m",
            "lift_swing_target_normal_from_nominal_m",
            "preload_target_normal_from_nominal_m",
            "attach_target_normal_from_nominal_m",
            "compliant_force_estimate_n",
            "compliant_normal_offset_m",
            "compliant_normal_velocity_mps",
            "estimated_leg_normal_force_n",
            "joint_torque_bias_norm_nm",
            "attach_confirmed_time_s",
            "workspace_clamped",
            "workspace_margin_m",
            "workspace_check_us",
        ]
        rospy.Subscriber("/control/body_reference", BodyReference, self.body_reference_callback, queue_size=20)
        rospy.Subscriber("/state/estimated", EstimatedState, self.estimated_state_callback, queue_size=20)
        rospy.Subscriber("/control/mission_state", String, self._mission_state_callback, queue_size=10)
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/leg_rpm", Float32MultiArray,
            self._fan_rpm_callback, queue_size=10,
        )

    def _fan_rpm_callback(self, msg):
        values = [float(v) for v in msg.data[:4]]
        while len(values) < 4:
            values.append(0.0)
        self._latest_fan_rpm = values

    def _mission_state_callback(self, msg):
        try:
            value = str(msg.data)
        except (TypeError, ValueError):
            return
        if value:
            self.mission_state = value

    def _build_leg_yaw_map(self):
        leg_cfg = rospy.get_param("/legs", {})
        yaw_map = {}
        for leg_name in self.leg_names:
            yaw_deg = float(leg_cfg.get(leg_name, {}).get("hip_yaw_deg", 0.0))
            yaw_map[leg_name] = math.radians(yaw_deg)
        return yaw_map

    def _build_leg_motor_map(self):
        leg_cfg = rospy.get_param("/legs", {})
        return {
            str(leg_name): [int(value) for value in cfg.get("motor_ids", [])]
            for leg_name, cfg in leg_cfg.items()
        }

    def body_reference_callback(self, msg):
        self.body_reference = msg
        self.have_body_reference = True

    def estimated_state_callback(self, msg):
        self.estimated_state = msg
        self.have_estimated_state = True

    def _desired_support_mask(self):
        if len(self.body_reference.support_mask) == len(self.leg_names):
            return [bool(value) for value in self.body_reference.support_mask]
        return [True] * len(self.leg_names)

    def _estimated_support_mask(self):
        if len(self.estimated_state.support_mask) == len(self.leg_names):
            return [bool(value) for value in self.estimated_state.support_mask]
        return self._desired_support_mask()

    def _leg_slip_risk(self, leg_index):
        return 0.0

    def _leg_normal_force_limit(self, leg_index):
        if leg_index < len(self.estimated_state.normal_force_limit):
            return float(self.estimated_state.normal_force_limit[leg_index])
        return 0.0

    def _leg_mask_value(self, values, leg_index, default=False):
        if leg_index < len(values):
            return bool(values[leg_index])
        return bool(default)

    def _leg_fan_current(self, leg_index):
        if leg_index < len(self.estimated_state.fan_current):
            return float(self.estimated_state.fan_current[leg_index])
        return 0.0

    def _leg_float_value(self, values, leg_index, default=0.0):
        if leg_index < len(values):
            return float(values[leg_index])
        return float(default)

    def _normal_component(self, vector):
        normal_scalar = vector_dot(vector, self.wall_normal_body)
        return vector_scale(self.wall_normal_body, normal_scalar)

    def _tangential_component(self, vector):
        return vector_sub(vector, self._normal_component(vector))

    def _compose_tangent_and_normal(self, tangent_vector, normal_scalar):
        return vector_add(self._tangential_component(tangent_vector), vector_scale(self.wall_normal_body, normal_scalar))

    def _phase_elapsed(self, state, now_sec):
        if state["phase_started_at"] is None:
            return 0.0
        return max(0.0, now_sec - state["phase_started_at"])

    def _set_phase(self, state, phase_name, now_sec):
        state["phase"] = phase_name
        state["phase_started_at"] = now_sec
        state["phase_start_pos"] = list(state.get("position", [0.0, 0.0, 0.0]))

    def _step_toward_target(self, current_position, target_position, velocity_limits, dt, clamp_center=None):
        next_position = []
        next_velocity = []
        for axis in [0, 1, 2]:
            error = target_position[axis] - current_position[axis]
            velocity = clamp(error / max(dt, 1e-3), -velocity_limits[axis], velocity_limits[axis])
            position = current_position[axis] + velocity * dt
            if (error > 0.0 and position > target_position[axis]) or (error < 0.0 and position < target_position[axis]):
                position = target_position[axis]
                velocity = 0.0
            next_position.append(position)
            next_velocity.append(velocity)
        return self._clamp_position(next_position, clamp_center), next_velocity

    def _bezier_interp(self, start_pos, target_pos, duration_s, phase_elapsed):
        phase = min(phase_elapsed / max(duration_s, 1e-3), 1.0)
        pos = [0.0, 0.0, 0.0]
        for axis in [0, 1, 2]:
            cp = [start_pos[axis], start_pos[axis], target_pos[axis], target_pos[axis], target_pos[axis]]
            pos[axis] = bezier4(cp, phase)
        return pos

    def _smooth_interp(self, start_pos, target_pos, duration_s, phase_elapsed):
        phase = min(phase_elapsed / max(duration_s, 1e-3), 1.0)
        progress = smoothstep5(phase)
        return [
            start_pos[axis] + progress * (target_pos[axis] - start_pos[axis])
            for axis in [0, 1, 2]
        ]

    def _clamp_position(self, position, center=None):
        if center is None:
            center = [0.0, 0.0, self.nominal_z_m]
        return [
            clamp(position[0], center[0] - self.max_position_offset[0], center[0] + self.max_position_offset[0]),
            clamp(position[1], center[1] - self.max_position_offset[1], center[1] + self.max_position_offset[1]),
            clamp(position[2], center[2] - self.max_position_offset[2], center[2] + self.max_position_offset[2]),
        ]

    def _tangential_error_norm(self, lhs, rhs):
        return vector_norm(self._tangential_component(vector_sub(lhs, rhs)))

    def _normal_error(self, lhs, rhs):
        return vector_dot(vector_sub(lhs, rhs), self.wall_normal_body)

    def _body_orientation(self):
        if self.have_estimated_state:
            return quaternion_to_list(self.estimated_state.pose.orientation)
        return [0.0, 0.0, 0.0, 1.0]

    def _body_position_error(self):
        if not (self.have_body_reference and self.have_estimated_state):
            return [0.0, 0.0, 0.0]
        desired = self.body_reference.pose.position
        estimated = self.estimated_state.pose.position
        error_world = [
            float(desired.x) - float(estimated.x),
            float(desired.y) - float(estimated.y),
            float(desired.z) - float(estimated.z),
        ]
        return world_to_body(error_world, self._body_orientation())

    def _body_velocity_error(self):
        if not (self.have_body_reference and self.have_estimated_state):
            return [0.0, 0.0, 0.0]
        desired = self.body_reference.twist.linear
        estimated = self.estimated_state.twist.linear
        error_world = [
            float(desired.x) - float(estimated.x),
            float(desired.y) - float(estimated.y),
            float(desired.z) - float(estimated.z),
        ]
        return world_to_body(error_world, self._body_orientation())

    def _body_angular_position_error(self):
        if not (self.have_body_reference and self.have_estimated_state):
            return [0.0, 0.0, 0.0]
        q_des = quaternion_to_list(self.body_reference.pose.orientation)
        q_est = quaternion_to_list(self.estimated_state.pose.orientation)
        error_world = quaternion_to_axis_angle(quaternion_multiply(q_des, quaternion_conjugate(q_est)))
        return world_to_body(error_world, self._body_orientation())

    def _body_angular_velocity_error(self):
        if not (self.have_body_reference and self.have_estimated_state):
            return [0.0, 0.0, 0.0]
        desired = self.body_reference.twist.angular
        estimated = self.estimated_state.twist.angular
        error_world = [
            float(desired.x) - float(estimated.x),
            float(desired.y) - float(estimated.y),
            float(desired.z) - float(estimated.z),
        ]
        return world_to_body(error_world, self._body_orientation())

    def _desired_twist_body(self):
        if not self.have_body_reference:
            return [0.0, 0.0, 0.0]
        desired = self.body_reference.twist.linear
        linear_world = [float(desired.x), float(desired.y), float(desired.z)]
        return world_to_body(linear_world, self._body_orientation())

    def _hip_offset(self, leg_name):
        hip_yaw = self.leg_yaw_rad.get(leg_name, 0.0)
        return [
            self.base_radius_m * math.cos(hip_yaw),
            self.base_radius_m * math.sin(hip_yaw),
            0.0,
        ]

    def _base_delta_to_leg_delta(self, leg_name, dx_base, dy_base):
        hip_yaw = self.leg_yaw_rad.get(leg_name, 0.0)
        cos_yaw = math.cos(hip_yaw)
        sin_yaw = math.sin(hip_yaw)
        return [
            cos_yaw * dx_base + sin_yaw * dy_base,
            -sin_yaw * dx_base + cos_yaw * dy_base,
        ]

    def _leg_delta_to_base_delta(self, leg_name, dx_leg, dy_leg):
        hip_yaw = self.leg_yaw_rad.get(leg_name, 0.0)
        cos_yaw = math.cos(hip_yaw)
        sin_yaw = math.sin(hip_yaw)
        return [
            cos_yaw * dx_leg - sin_yaw * dy_leg,
            sin_yaw * dx_leg + cos_yaw * dy_leg,
        ]

    def _operating_center_command(self, leg_name):
        dx_leg = self.operating_x_m - self.nominal_x_m
        dy_leg = self.operating_y_m - self.nominal_y_m
        dx_base, dy_base = self._leg_delta_to_base_delta(leg_name, dx_leg, dy_leg)
        return [dx_base, dy_base, self.operating_z_m]

    def _leg_force_to_body_force(self, leg_name, leg_force):
        hip_yaw = self.leg_yaw_rad.get(leg_name, 0.0)
        cos_yaw = math.cos(hip_yaw)
        sin_yaw = math.sin(hip_yaw)
        return [
            cos_yaw * leg_force[0] - sin_yaw * leg_force[1],
            sin_yaw * leg_force[0] + cos_yaw * leg_force[1],
            leg_force[2],
        ]

    @staticmethod
    def _solution_cost(candidate_rad, reference_rad):
        return sum([(float(candidate_rad[index]) - float(reference_rad[index])) ** 2 for index in [0, 1, 2]])

    def _ik_candidates_rad(self, x_m, y_m, z_m):
        x_prime = float(x_m) - self.l_coxa_m
        q1 = math.atan2(float(y_m), x_prime)
        radial_total = math.hypot(x_prime, float(y_m))
        radial_prime = max(radial_total - self.l_femur_m, 1e-6)

        d_sq = radial_prime ** 2 + float(z_m) ** 2
        cos_theta3 = (d_sq - self.l_tibia_m ** 2 - self.l_a3_m ** 2) / (2.0 * self.l_tibia_m * self.l_a3_m)
        cos_theta3 = clamp(cos_theta3, -1.0, 1.0)
        sin_theta3_mag = math.sqrt(max(0.0, 1.0 - cos_theta3 ** 2))

        candidates = []
        for branch_sign in [-1.0, 1.0]:
            theta3 = math.atan2(branch_sign * sin_theta3_mag, cos_theta3)
            q2 = math.atan2(float(z_m), radial_prime) - math.atan2(
                self.l_a3_m * math.sin(theta3),
                self.l_tibia_m + self.l_a3_m * math.cos(theta3),
            ) + math.radians(90.0)
            candidates.append([q1, q2, theta3])
        return candidates

    def _joint_vector_from_position(self, leg_name, position, reference_rad=None):
        leg_delta = self._base_delta_to_leg_delta(leg_name, position[0], position[1])
        target_x = self.nominal_x_m + leg_delta[0]
        target_y = self.nominal_y_m + leg_delta[1]
        target_z = position[2]
        candidates = self._ik_candidates_rad(target_x, target_y, target_z)
        if reference_rad is None:
            reference_rad = [0.0, math.radians(90.0), math.radians(-60.0)]

        q3_0 = candidates[0][2]
        q3_1 = candidates[1][2]

        if q3_0 < 0.0 and q3_1 >= 0.0:
            chosen_idx = 0
        elif q3_1 < 0.0 and q3_0 >= 0.0:
            chosen_idx = 1
        else:
            ref = list(reference_rad)
            if ref[2] > 0.0:
                ref[2] = 0.0
            costs = [self._solution_cost(c, ref) for c in candidates]
            chosen_idx = 0 if costs[0] <= costs[1] else 1

        costs = [self._solution_cost(c, reference_rad) for c in candidates]
        cost_diff_rad2 = abs(costs[0] - costs[1])
        if cost_diff_rad2 < math.radians(10.0) ** 2:
            rospy.logwarn_throttle(
                2.0,
                "IK branch switching risk (leg=%s cost_diff=%.4f): "
                "cand1=[%.2f, %.2f, %.2f](cost=%.4f) "
                "cand2=[%.2f, %.2f, %.2f](cost=%.4f) "
                "reference=[%.2f, %.2f, %.2f] chosen=%d",
                leg_name, cost_diff_rad2,
                candidates[0][0], candidates[0][1], candidates[0][2], costs[0],
                candidates[1][0], candidates[1][1], candidates[1][2], costs[1],
                reference_rad[0], reference_rad[1], reference_rad[2],
                chosen_idx,
            )
        return candidates[chosen_idx]

    def _forward_kinematics_leg(self, joint_vector):
        q1 = float(joint_vector[0])
        q2 = float(joint_vector[1])
        q3 = float(joint_vector[2])
        alpha = q2 - math.radians(90.0)
        radial_prime = self.l_tibia_m * math.cos(alpha) + self.l_a3_m * math.cos(alpha + q3)
        p_z = self.l_tibia_m * math.sin(alpha) + self.l_a3_m * math.sin(alpha + q3)
        radial_total = self.l_femur_m + radial_prime
        return [
            self.l_coxa_m + radial_total * math.cos(q1),
            radial_total * math.sin(q1),
            p_z,
        ]

    def _leg_jacobian(self, joint_vector):
        base_position = self._forward_kinematics_leg(joint_vector)
        jacobian = [[0.0, 0.0, 0.0] for _ in range(3)]
        for column in [0, 1, 2]:
            perturbed = list(joint_vector)
            perturbed[column] += self.jacobian_delta_rad
            next_position = self._forward_kinematics_leg(perturbed)
            for row in [0, 1, 2]:
                jacobian[row][column] = (next_position[row] - base_position[row]) / max(self.jacobian_delta_rad, 1e-9)
        return jacobian

    def _leg_joint_torque_vector(self, leg_name):
        motor_ids = self.leg_to_motors.get(leg_name, [])
        joint_torques = list(self.estimated_state.joint_torques_est)
        if len(motor_ids) != 3 or len(joint_torques) == 0:
            return None
        torque_vector = []
        for motor_id in motor_ids:
            joint_index = self.joint_name_to_index.get(str(int(motor_id)))
            if joint_index is None or joint_index >= len(joint_torques):
                return None
            torque_vector.append(float(joint_torques[joint_index]))
        return torque_vector

    def _capture_compliant_torque_bias(self, leg_name, state):
        torque_vector = self._leg_joint_torque_vector(leg_name)
        state["compliant_joint_torque_bias"] = list(torque_vector) if torque_vector is not None else [0.0, 0.0, 0.0]
        state["compliant_force_estimate"] = 0.0
        state["compliant_normal_offset"] = 0.0
        state["compliant_normal_velocity"] = 0.0

    def _estimate_leg_normal_force(self, leg_name, state):
        torque_vector = self._leg_joint_torque_vector(leg_name)
        if torque_vector is None:
            return 0.0
        joint_vector = self._joint_vector_from_position(leg_name, state["position"], state.get("last_joint_vector"))
        state["last_joint_vector"] = list(joint_vector)
        jacobian = self._leg_jacobian(joint_vector)
        try:
            leg_force = np.linalg.lstsq(
                np.array(jacobian, dtype=float).T,
                np.array(vector_sub(torque_vector, state["compliant_joint_torque_bias"]), dtype=float),
                rcond=None,
            )[0]
        except np.linalg.LinAlgError:
            return 0.0
        body_force = self._leg_force_to_body_force(leg_name, leg_force.tolist())
        signed_normal_force = self.compliant_force_sign * vector_dot(body_force, self.wall_normal_body)
        return clamp(signed_normal_force, -self.compliant_force_limit_n, self.compliant_force_limit_n)

    def _update_normal_admittance(self, state, measured_force_n, dt):
        filtered_force = (1.0 - self.compliant_force_filter_alpha) * float(state["compliant_force_estimate"]) + self.compliant_force_filter_alpha * float(measured_force_n)
        state["compliant_force_estimate"] = filtered_force
        if filtered_force > self.compliant_force_deadband_n:
            effective_force = min(filtered_force - self.compliant_force_deadband_n, self.compliant_force_limit_n)
        elif filtered_force < -self.compliant_force_deadband_n:
            effective_force = max(filtered_force + self.compliant_force_deadband_n, -self.compliant_force_limit_n)
        else:
            effective_force = 0.0

        acceleration = (
            effective_force
            - self.compliant_admittance_damping * float(state["compliant_normal_velocity"])
            - self.compliant_admittance_stiffness * float(state["compliant_normal_offset"])
        ) / self.compliant_admittance_mass
        next_velocity = float(state["compliant_normal_velocity"]) + acceleration * dt
        next_velocity = clamp(next_velocity, -self.compliant_normal_velocity_limit_mps, self.compliant_normal_velocity_limit_mps)
        next_offset = float(state["compliant_normal_offset"]) + next_velocity * dt
        next_offset = clamp(next_offset, -self.compliant_normal_offset_limit_m, self.compliant_normal_offset_limit_m)
        if next_offset >= self.compliant_normal_offset_limit_m and next_velocity > 0.0:
            next_velocity = 0.0
        if next_offset <= -self.compliant_normal_offset_limit_m and next_velocity < 0.0:
            next_velocity = 0.0

        state["compliant_normal_velocity"] = next_velocity
        state["compliant_normal_offset"] = next_offset
        return next_offset, next_velocity

    def _freeze_compliant_state(self, state, frozen_position):
        frozen_target = self._clamp_position(
            frozen_position,
            state.get("support_target", None),
        )
        state["attach_target"] = list(frozen_target)
        state["position"] = list(frozen_target)
        state["velocity"] = [0.0, 0.0, 0.0]
        state["compliant_normal_velocity"] = 0.0

    def _target_delta(self, leg_name, leg_index):
        # Stride gait: fixed 3L/4 swing, body advances L/4 per leg.
        if self.stride_length_m > 0.0:
            return [
                clamp(self._swing_distance_m, -self.foot_delta_x_limit, self.foot_delta_x_limit),
                0.0,
                0.0,
            ]

  

        desired_twist = self._desired_twist_body()
        estimated_twist = [0.0, 0.0, 0.0]
        if self.have_estimated_state:
            estimated_linear = self.estimated_state.twist.linear
            estimated_twist = world_to_body(
                [float(estimated_linear.x), float(estimated_linear.y), float(estimated_linear.z)],
                self._body_orientation(),
            )

        nominal_height = max(abs(self.nominal_z_m), 1e-3)
        stance_time = 0.5 * self.swing_duration_s
        raibert_time = self.raibert_height_gain * math.sqrt(nominal_height / 9.81)
        raibert_delta = [
            raibert_time * (estimated_twist[0] - desired_twist[0]) + self.raibert_velocity_gain * stance_time * desired_twist[0],
            raibert_time * (estimated_twist[1] - desired_twist[1]) + self.raibert_velocity_gain * stance_time * desired_twist[1],
            0.0,
        ]
        
        body_position_term = vector_mul(self.body_position_gain, self._body_position_error())
        body_velocity_term = vector_mul(self.body_velocity_gain, self._body_velocity_error())
        feedforward_term = vector_mul(self.feedforward_twist_gain, desired_twist)

        # each swing cycle. Keep Z for normal force correction.
        body_position_term[0] = 0.0
        body_position_term[1] = 0.0
        body_velocity_term[0] = 0.0
        body_velocity_term[1] = 0.0

        angular_error = self._body_angular_position_error()
        angular_velocity_error = self._body_angular_velocity_error()
        angular_term = vector_add(
            vector_mul(self.body_angular_position_gain, angular_error),
            vector_mul(self.body_angular_velocity_gain, angular_velocity_error),
        )
        angular_correction = cross(angular_term, self._hip_offset(leg_name))
        slip_bias = [0.0, 0.0, 0.0]

        raw_target = vector_add(raibert_delta, feedforward_term)
        raw_target = vector_add(raw_target, body_position_term)
        raw_target = vector_add(raw_target, body_velocity_term)
        # raw_target = vector_add(raw_target, angular_correction)
        raw_target = vector_add(raw_target, slip_bias)

        return [
            clamp(raw_target[0], -min(self.max_position_offset[0], self.foot_delta_x_limit), min(self.max_position_offset[0], self.foot_delta_x_limit)),
            clamp(raw_target[1], -min(self.max_position_offset[1], self.foot_delta_y_limit), min(self.max_position_offset[1], self.foot_delta_y_limit)),
            clamp(raw_target[2], -self.max_position_offset[2], self.max_position_offset[2]),
        ]

    def _start_swing(self, leg_name, leg_index, stamp_sec):
        state = self.swing_states[leg_name]
        target_delta = self._target_delta(leg_name, leg_index)

        if self.stride_length_m > 0.0:
            # Estimated UJC positions include the hip-origin transform, whereas
            # LegCenterCommand.center is expressed in the controller command
            # frame. Keep the stride reference in the command frame.
            support_target = list(state["position"])
        else:
            support_target = self._operating_center_command(leg_name)
            if self.have_body_reference:
                support_target[0] -= self.body_reference.pose.position.x
                support_target[1] -= self.body_reference.pose.position.y
        target = [
            support_target[0] + target_delta[0],
            support_target[1] + target_delta[1],
            support_target[2],
        ]

        # start = support_target (actual ujc in stride mode, or body-frame
        # position otherwise)
        start = list(support_target)
        start_normal = vector_dot(start, self.wall_normal_body)
        target_normal = vector_dot(target, self.wall_normal_body)

        lift_target = self._compose_tangent_and_normal(start, start_normal + self.swing_lift_normal_m)
        swing_target = self._compose_tangent_and_normal(target, start_normal + self.swing_lift_normal_m)
        preload_target = self._compose_tangent_and_normal(target, target_normal - self.preload_extra_normal_m)     
        attach_target = list(preload_target)

        state["start"] = list(start)
        state["target"] = list(target)
        state["support_target"] = list(support_target)
        state["lift_target"] = self._clamp_position(lift_target, state["support_target"])
        state["lift_swing_target"] = self._clamp_position(swing_target, state["support_target"])
        state["preload_target"] = self._clamp_position(preload_target, state["support_target"])
        state["attach_target"] = self._clamp_position(attach_target, state["support_target"])
        state["compliant_joint_torque_bias"] = [0.0, 0.0, 0.0]
        state["compliant_force_estimate"] = 0.0
        state["compliant_normal_offset"] = 0.0
        state["compliant_normal_velocity"] = 0.0
        state["last_joint_vector"] = self._joint_vector_from_position(leg_name, start, state.get("last_joint_vector"))
        state["support"] = False
        state["attach_confirmed_time"] = None
        state["transfer_fraction"] = 0.0
        state["transfer_committed"] = False
        if self.stride_length_m > 0.0:
            self._set_phase(state, self.PHASE_LIFT, stamp_sec)
        else:
            self._set_phase(state, self.PHASE_LIFT_SWING, stamp_sec)
        self.swing_phase_start[leg_name] = stamp_sec

    def _dwell_position(self, leg_name):
        """Return the position to hold during fan-off dwell.
        
        In stride gait mode, stay at the leg's current position (which may
        be forward of operating center after a swing).  Otherwise use the
        operating center (legacy behaviour)."""
        if self.stride_length_m > 0.0:
            state = self.swing_states.get(leg_name, {})
            return list(state.get("position", self._operating_center_command(leg_name)))
        return self._operating_center_command(leg_name)

    def _stride_body_position(self):
        body_x = float(self.body_reference.pose.position.x) if self.have_body_reference else 0.0
        body_y = float(self.body_reference.pose.position.y) if self.have_body_reference else 0.0
        return [
            body_x + self._body_x_offset + self._inflight_body_advance_m,
            body_y + self._body_y_offset,
        ]

    def _capture_support_anchor(self, leg_name, position=None):
        if self.stride_length_m <= 0.0:
            return
        state = self.swing_states[leg_name]
        if position is None:
            position = state.get("position", self._operating_center_command(leg_name))
        body_x, body_y = self._stride_body_position()
        state["support_world_x"] = float(position[0]) + body_x
        state["support_world_y"] = float(position[1]) + body_y

    def _initialize_stride_anchors(self):
        if self.stride_length_m <= 0.0:
            return
        for support_leg in self.leg_names:
            state = self.swing_states[support_leg]
            if state.get("support_world_x") is None:
                self._capture_support_anchor(support_leg)

    def _support_fans_ready(self, swing_leg):
        for leg_index, leg_name in enumerate(self.leg_names):
            if leg_name == swing_leg:
                continue
            actual_rpm = abs(self._latest_fan_rpm[leg_index])
            if actual_rpm < self.fan_recovery_rpm_threshold:
                rospy.logwarn_throttle(
                    2.0,
                    "stride gait holding %s: support fan %s rpm=%.0f below %.0f",
                    swing_leg, leg_name, actual_rpm, self.fan_recovery_rpm_threshold,
                )
                return False
        return True

    def _begin_release_wait(self, leg_name, now_sec):
        state = self.swing_states[leg_name]
        self._initialize_stride_anchors()
        state["support"] = False
        state["velocity"] = [0.0, 0.0, 0.0]
        state["transfer_fraction"] = 0.0
        state["transfer_committed"] = False
        self._fan_off_request_time[leg_name] = now_sec
        self._set_phase(state, self.PHASE_RELEASE_WAIT, now_sec)
        self.swing_phase_start[leg_name] = now_sec

    def _update_stride_transfer_progress(self, dt):
        self._inflight_body_advance_m = 0.0
        if self.stride_length_m <= 0.0:
            return
        for leg_name in self.leg_names:
            state = self.swing_states[leg_name]
            if state.get("phase") != self.PHASE_TRANSFER or self.swing_phase_start[leg_name] is None:
                continue
            fraction = clamp(float(state.get("transfer_fraction", 0.0)), 0.0, 1.0)
            self._inflight_body_advance_m = smoothstep5(fraction) * self._body_advance_per_swing_m
            if not self._support_fans_ready(leg_name):
                return
            fraction = clamp(
                fraction + dt / self.transfer_duration_s,
                0.0,
                1.0,
            )
            state["transfer_fraction"] = fraction
            progress = smoothstep5(fraction)
            self._inflight_body_advance_m = progress * self._body_advance_per_swing_m
            if fraction >= 1.0 and not bool(state.get("transfer_committed", False)):
                self._body_x_offset += self._body_advance_per_swing_m
                state["transfer_committed"] = True
                self._inflight_body_advance_m = 0.0
            return

    def _support_command(self, leg_name, leg_index):
        state = self.swing_states[leg_name]
        state["support"] = True
        self.swing_phase_start[leg_name] = None
        state["velocity"] = [0.0, 0.0, 0.0]

        if self.stride_length_m > 0.0:
            if state.get("support_world_x") is None:
                self._capture_support_anchor(leg_name)
            # Stride gait: command the support leg backward in body frame
            # (support_world - body_x).  The leg is attached to the wall
            # (fan suction) and cannot move in world frame, so the backward
            # command pushes the body forward by L/4 per leg swing.
            body_x, body_y = self._stride_body_position()
            support_target = [
                state["support_world_x"] - body_x,
                state["support_world_y"] - body_y,
                self.operating_z_m,
            ]
        else:
            support_target = self._operating_center_command(leg_name)
            if self.have_body_reference:
                support_target[0] -= self.body_reference.pose.position.x
                support_target[1] -= self.body_reference.pose.position.y

        state["support_target"] = list(support_target)
        state["position"] = list(support_target)
        state["start"] = list(support_target)
        state["target"] = list(support_target)
        state["lift_target"] = list(support_target)
        state["lift_swing_target"] = list(support_target)
        state["preload_target"] = list(support_target)
        state["attach_target"] = list(support_target)
        state["compliant_joint_torque_bias"] = [0.0, 0.0, 0.0]
        state["compliant_force_estimate"] = 0.0
        state["compliant_normal_offset"] = 0.0
        state["compliant_normal_velocity"] = 0.0
        state["attach_confirmed_time"] = None
        state["phase"] = self.PHASE_SUPPORT
        state["phase_started_at"] = None
        return self._build_message(
            leg_name,
            support_target,
            [0.0, 0.0, 0.0],
            True,
            self._leg_normal_force_limit(leg_index),
        )

    def _guided_swing_command(self, leg_name, leg_index, now_sec, dt):
        state = self.swing_states[leg_name]
        phase = state["phase"]
        phase_elapsed = self._phase_elapsed(state, now_sec)
        attachment_ready = self._leg_mask_value(self.estimated_state.attachment_ready_mask, leg_index, False)
        clamp_center = state.get("support_target", self._operating_center_command(leg_name))

        if phase == self.PHASE_RELEASE_WAIT:
            actual_rpm = abs(self._latest_fan_rpm[leg_index])
            if actual_rpm <= self.fan_off_rpm_threshold:
                self._fan_off_request_time.pop(leg_name, None)
                self._start_swing(leg_name, leg_index, now_sec)
                return self._guided_swing_command(leg_name, leg_index, now_sec, dt)
            if phase_elapsed >= self._fan_rpm_timeout_s:
                rospy.logwarn_throttle(
                    2.0,
                    "stride gait waiting to lift %s: fan rpm=%.0f above release threshold %.0f",
                    leg_name, actual_rpm, self.fan_off_rpm_threshold,
                )
            cmd_position = list(state["position"])
            cmd_velocity = [0.0, 0.0, 0.0]
            normal_force_limit = 0.001
            support_leg = False

        elif phase == self.PHASE_LIFT:
            start_pos = list(state["phase_start_pos"])
            cmd_position = self._smooth_interp(start_pos, list(state["lift_target"]), self.lift_duration_s, phase_elapsed)
            cmd_velocity = [0.0, 0.0, 0.0]
            if phase_elapsed >= self.lift_duration_s:
                cmd_position = list(state["lift_target"])
                state["position"] = list(cmd_position)
                state["transfer_fraction"] = 0.0
                state["transfer_committed"] = False
                self._set_phase(state, self.PHASE_TRANSFER, now_sec)
            normal_force_limit = 0.001
            support_leg = False

        elif phase == self.PHASE_TRANSFER:
            fraction = clamp(float(state.get("transfer_fraction", 0.0)), 0.0, 1.0)
            progress = smoothstep5(fraction)
            start_pos = list(state["phase_start_pos"])
            target_pos = list(state["lift_swing_target"])
            cmd_position = [
                start_pos[axis] + progress * (target_pos[axis] - start_pos[axis])
                for axis in [0, 1, 2]
            ]
            cmd_velocity = [0.0, 0.0, 0.0]
            if fraction >= 1.0:
                cmd_position = list(target_pos)
                state["position"] = list(cmd_position)
                self._set_phase(state, self.PHASE_PRELOAD, now_sec)
            normal_force_limit = 0.001
            support_leg = False

        elif phase == self.PHASE_LIFT_SWING:
            lift_duration = max(self.tangential_align_duration_s, 0.05)
            start_pos = list(state["phase_start_pos"])
            lift_swing_target = list(state["lift_swing_target"])
            # XY follows Bezier smooth interpolation; Z (normal height) instantly
            # reaches and holds the target lift height for the entire phase.
            cmd_position = self._bezier_interp(start_pos, lift_swing_target, lift_duration, phase_elapsed)
            cmd_position[2] = lift_swing_target[2]
            cmd_velocity = [0.0, 0.0, 0.0]
            if phase_elapsed >= lift_duration:
                self._set_phase(state, self.PHASE_PRELOAD, now_sec)
            # Use tiny positive value so mission_supervisor does NOT replace
            # with default_normal_force_limit (150N) which would keep fan ON.
            normal_force_limit = 0.001
            support_leg = False

        elif phase == self.PHASE_PRELOAD:
            start_pos = list(state["phase_start_pos"])
            cmd_position = self._smooth_interp(start_pos, list(state["preload_target"]), self.preload_duration_s, phase_elapsed)
            cmd_velocity = [0.0, 0.0, 0.0]
            preload_complete = phase_elapsed >= self.preload_duration_s
            if self.stride_length_m <= 0.0:
                preload_complete = preload_complete or (
                    abs(self._normal_error(state["preload_target"], cmd_position))
                    <= self.normal_alignment_tolerance_m
                )
            if preload_complete:
                self._capture_compliant_torque_bias(leg_name, state)
                self._set_phase(state, self.PHASE_ADMIT, now_sec)
            normal_force_limit = self.preload_normal_force_limit_n
            support_leg = False

        elif phase == self.PHASE_ADMIT:
            measured_force_n = self._estimate_leg_normal_force(leg_name, state)
            normal_offset, normal_velocity = self._update_normal_admittance(state, measured_force_n, dt)
            target_normal_scalar = vector_dot(state["preload_target"], self.wall_normal_body) + normal_offset
            cmd_position = self._clamp_position(
                self._compose_tangent_and_normal(state["attach_target"], target_normal_scalar),
                clamp_center,
            )
            cmd_velocity = vector_scale(self.wall_normal_body, normal_velocity)
            if self.stride_length_m > 0.0:
                rpm_ready = abs(self._latest_fan_rpm[leg_index]) >= self.fan_recovery_rpm_threshold
                attachment_confirmed = attachment_ready if self.use_contact_feedback else True
                attach_ready = rpm_ready and attachment_confirmed
            else:
                attach_ready = attachment_ready
                if not self.use_contact_feedback:
                    attach_ready = False
            if attach_ready:
                if state["attach_confirmed_time"] is None:
                    self._freeze_compliant_state(state, cmd_position)
                    state["attach_confirmed_time"] = now_sec
                if now_sec - state["attach_confirmed_time"] >= 0.2:
                    support_leg = True
                else:
                    support_leg = False
            else:
                state["attach_confirmed_time"] = None
                support_leg = False
            if self.stride_length_m > 0.0 and phase_elapsed >= self.compliant_settle_timeout_s and not support_leg:
                rospy.logwarn_throttle(
                    2.0,
                    "stride gait waiting to attach %s: fan rpm=%.0f threshold=%.0f contact=%s",
                    leg_name, abs(self._latest_fan_rpm[leg_index]),
                    self.fan_recovery_rpm_threshold, bool(attachment_ready),
                )
            if self.stride_length_m <= 0.0 and phase_elapsed >= self.compliant_settle_timeout_s and not support_leg:
                support_leg = True
                self._freeze_compliant_state(state, cmd_position)
            normal_force_limit = self.attach_normal_force_limit_n

        else:
            cmd_position = list(state["position"])
            cmd_velocity = [0.0, 0.0, 0.0]
            normal_force_limit = self._leg_normal_force_limit(leg_index)
            support_leg = True

        state["position"] = list(cmd_position)
        state["velocity"] = list(cmd_velocity)
        return cmd_position, cmd_velocity, support_leg, normal_force_limit

    def _build_message(self, leg_name, position, velocity, support_leg, normal_force_limit):
        msg = LegCenterCommand()
        msg.header.stamp = rospy.Time.now()
        msg.leg_name = leg_name
        msg.center = Point(position[0], position[1], position[2])
        msg.center_velocity = Vector3(velocity[0], velocity[1], velocity[2])
        if self.mission_state in self.hold_position_states:
            msg.support_leg = False
        else:
            msg.support_leg = bool(support_leg)
        msg.desired_normal_force_limit = float(normal_force_limit)
        # Fill in target positions for CSV recording
        state = self.swing_states.get(leg_name, {})
        nominal = self._operating_center_command(leg_name)
        lift_tgt = state.get("lift_swing_target", nominal)
        preload_tgt = state.get("preload_target", nominal)
        attach_tgt = state.get("attach_target", nominal)
        msg.lift_swing_target = Point(lift_tgt[0], lift_tgt[1], lift_tgt[2])
        msg.preload_target = Point(preload_tgt[0], preload_tgt[1], preload_tgt[2])
        msg.attach_target = Point(attach_tgt[0], attach_tgt[1], attach_tgt[2])
        return msg

    def _publish_leg_diagnostic(self, leg_name, leg_index, cmd_position, now_sec):
        publisher = self.diagnostic_pubs.get(leg_name)
        if publisher is None:
            return
        state = self.swing_states[leg_name]
        phase = state.get("phase", self.PHASE_SUPPORT)
        phase_id = float(self.phase_id_map.get(phase, -1))
        phase_started_at = state.get("phase_started_at")
        phase_elapsed = 0.0 if phase_started_at is None else max(0.0, now_sec - float(phase_started_at))
        nominal_position = self._operating_center_command(leg_name)
        nominal_normal = vector_dot(nominal_position, self.wall_normal_body)
        cmd_normal_from_nominal = vector_dot(cmd_position, self.wall_normal_body) - nominal_normal
        lift_swing_normal_from_nominal = vector_dot(state.get("lift_swing_target", nominal_position), self.wall_normal_body) - nominal_normal
        preload_normal_from_nominal = vector_dot(state.get("preload_target", nominal_position), self.wall_normal_body) - nominal_normal
        attach_normal_from_nominal = vector_dot(state.get("attach_target", nominal_position), self.wall_normal_body) - nominal_normal
        bias_vector = state.get("compliant_joint_torque_bias", [0.0, 0.0, 0.0])
        bias_norm = vector_norm(bias_vector if isinstance(bias_vector, list) else list(bias_vector))
        attach_confirmed_time = state.get("attach_confirmed_time")
        attach_confirmed_age = 0.0 if attach_confirmed_time is None else max(0.0, now_sec - float(attach_confirmed_time))
        try:
            measured_force = self._estimate_leg_normal_force(leg_name, state)
        except Exception:
            measured_force = 0.0

        msg = Float32MultiArray()
        msg.layout = MultiArrayLayout()
        msg.layout.dim = [
            MultiArrayDimension(label=label, size=1, stride=len(self.diagnostic_field_labels))
            for label in self.diagnostic_field_labels
        ]

        msg.data = [
            float(leg_index),
            float(phase_id),
            float(phase_elapsed),
            float(cmd_normal_from_nominal),
            float(lift_swing_normal_from_nominal),
            float(preload_normal_from_nominal),
            float(attach_normal_from_nominal),
            float(state.get("compliant_force_estimate", 0.0)),
            float(state.get("compliant_normal_offset", 0.0)),
            float(state.get("compliant_normal_velocity", 0.0)),
            float(measured_force),
            float(bias_norm),
            float(attach_confirmed_age),
            float(state.get("workspace_clamped", 0.0)),
            float(state.get("workspace_margin_m", 0.0)),
            float(state.get("workspace_check_us", 0.0)),
        ]
        publisher.publish(msg)

    def _workspace_guarded_position(self, leg_name, candidate_position, reference_position):
        state = self.swing_states[leg_name]
        if not self.workspace_check_enabled or self.workspace_check_mode != "per_cycle":
            state["workspace_clamped"] = 0.0
            state["workspace_margin_m"] = 0.0
            state["workspace_check_us"] = 0.0
            return list(candidate_position)

        checked_position, is_clamped, margin_m, joint_solution_deg, elapsed_us = workspace_guard(
            leg_name=leg_name,
            candidate_center_body_m=list(candidate_position),
            reference_center_body_m=list(reference_position),
            last_joint_deg=state.get("workspace_last_joint_deg", [0.0, 0.0, 0.0]),
            model={
                "nominal_x_m": self.nominal_x_m,
                "nominal_y_m": self.nominal_y_m,
                "nominal_z_m": self.nominal_z_m,
                "operating_x_m": self.operating_x_m,
                "operating_y_m": self.operating_y_m,
                "operating_z_m": self.operating_z_m,
                "l_coxa": self.l_coxa_m,
                "l_femur": self.l_femur_m,
                "l_tibia": self.l_tibia_m,
                "l_a3": self.l_a3_m,
                "leg_yaw_rad": self.leg_yaw_rad,
            },
            joint_limits_deg=self.joint_limit_deg,
            clamp_max_iter=self.workspace_clamp_max_iter,
            fk_tol_m=self.workspace_fk_tolerance_m,
        )
        state["workspace_last_joint_deg"] = list(joint_solution_deg)
        state["workspace_clamped"] = 1.0 if is_clamped else 0.0
        state["workspace_margin_m"] = float(margin_m)
        state["workspace_check_us"] = float(elapsed_us)
        if is_clamped:
            rospy.logwarn_throttle(
                self.workspace_warn_throttle_s,
                "workspace_guard clamped %s from [%.3f %.3f %.3f] to [%.3f %.3f %.3f] margin=%.4f",
                leg_name,
                float(candidate_position[0]),
                float(candidate_position[1]),
                float(candidate_position[2]),
                float(checked_position[0]),
                float(checked_position[1]),
                float(checked_position[2]),
                float(margin_m),
            )
        return list(checked_position)

    def _publish_guarded_command(self, leg_name, leg_index, position, velocity, support_leg, normal_force_limit):
        reference_position = self.swing_states[leg_name].get("support_target", self._operating_center_command(leg_name))
        guarded_position = self._workspace_guarded_position(leg_name, position, reference_position)
        self.swing_states[leg_name]["position"] = list(guarded_position)
        msg = self._build_message(
            leg_name,
            guarded_position,
            velocity,
            support_leg,
            normal_force_limit,
        )
        self.pub.publish(msg)
        self._publish_leg_diagnostic(leg_name, leg_index, guarded_position, rospy.Time.now().to_sec())
        return msg

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            now_sec = now.to_sec()
            if self.last_update_time is None:
                dt = 1.0 / max(self.rate_hz, 1.0)
            else:
                dt = clamp(now_sec - self.last_update_time, 1e-3, 0.1)
            self.last_update_time = now_sec

            desired_support_mask = self._desired_support_mask()
            self._update_stride_transfer_progress(dt)

            for leg_index, leg_name in enumerate(self.leg_names):
                desired_support = desired_support_mask[leg_index] if leg_index < len(desired_support_mask) else True

                if desired_support and self.swing_phase_start[leg_name] is None:
                    support_msg = self._support_command(leg_name, leg_index)
                    self._publish_guarded_command(
                        leg_name,
                        leg_index,
                        [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                        [support_msg.center_velocity.x, support_msg.center_velocity.y, support_msg.center_velocity.z],
                        support_msg.support_leg,
                        support_msg.desired_normal_force_limit,
                    )
                    continue

                if self.swing_phase_start[leg_name] is None and not desired_support:
                    if self.stride_length_m > 0.0:
                        if not self._support_fans_ready(leg_name):
                            continue
                        self._begin_release_wait(leg_name, now_sec)
                    else:
                        # Legacy dwell mode: let the fan spin down before motion.
                        if self.fan_off_dwell_s > 0:
                            fan_off_time = self._fan_off_request_time.get(leg_name)
                            if fan_off_time is None:
                                # With adhesion feedback, wait for the previous
                                # support fan before releasing this leg.
                                prev_idx = (leg_index - 1) % len(self.leg_names)
                                prev_rpm = abs(self._latest_fan_rpm[prev_idx])
                                if self.use_contact_feedback and prev_rpm < self.fan_recovery_rpm_threshold:
                                    continue

                                # First cycle: request fan off and hold position.
                                self._fan_off_request_time[leg_name] = now_sec
                                dwell_pos = self._dwell_position(leg_name)
                                self._publish_guarded_command(leg_name, leg_index, dwell_pos, [0.0, 0.0, 0.0], False, 0.001)
                                continue
                            elif now_sec - fan_off_time < self.fan_off_dwell_s:
                                # Still waiting for minimum dwell time.
                                dwell_pos = self._dwell_position(leg_name)
                                self._publish_guarded_command(leg_name, leg_index, dwell_pos, [0.0, 0.0, 0.0], False, 0.001)
                                continue
                            else:
                                # Dwell elapsed; enforce RPM only when feedback
                                # is part of this legacy-mode test.
                                leg_idx = self.leg_names.index(leg_name)
                                actual_rpm = abs(self._latest_fan_rpm[leg_idx])
                                total_wait = now_sec - fan_off_time
                                rpm_ok = (
                                    not self.use_contact_feedback
                                    or actual_rpm <= self.fan_off_rpm_threshold
                                )
                                timed_out = total_wait >= self._fan_rpm_timeout_s

                                if rpm_ok or timed_out:
                                    # RPM confirmed low, or timeout fallback.
                                    self._fan_off_request_time.pop(leg_name, None)
                                    if timed_out and not rpm_ok:
                                        rospy.logwarn_throttle(
                                            5.0,
                                            "fan off timeout for %s "
                                            "(rpm=%.0f, threshold=%.0f, waited=%.1fs)",
                                            leg_name, actual_rpm,
                                            self.fan_off_rpm_threshold, total_wait,
                                        )
                                else:
                                    # Fan not yet at target RPM; hold position.
                                    dwell_pos = self._dwell_position(leg_name)
                                    self._publish_guarded_command(leg_name, leg_index, dwell_pos, [0.0, 0.0, 0.0], False, 0.001)
                                    continue
                        self._start_swing(leg_name, leg_index, now_sec)

                if self.swing_phase_start[leg_name] is None:
                    support_msg = self._support_command(leg_name, leg_index)
                    self._publish_guarded_command(
                        leg_name,
                        leg_index,
                        [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                        [support_msg.center_velocity.x, support_msg.center_velocity.y, support_msg.center_velocity.z],
                        support_msg.support_leg,
                        support_msg.desired_normal_force_limit,
                    )
                    continue

                cmd_position, cmd_velocity, support_leg, normal_force_limit = self._guided_swing_command(leg_name, leg_index, now_sec, dt)

                # If support_leg=True (state 2), transition back to SUPPORT
                if support_leg and desired_support:
                    self.swing_phase_start[leg_name] = None
                    if self.stride_length_m > 0.0:
                        state = self.swing_states[leg_name]
                        self._capture_support_anchor(leg_name, state["position"])
                    support_msg = self._support_command(leg_name, leg_index)
                    self._publish_guarded_command(
                        leg_name,
                        leg_index,
                        [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                        [support_msg.center_velocity.x, support_msg.center_velocity.y, support_msg.center_velocity.z],
                        support_msg.support_leg,
                        support_msg.desired_normal_force_limit,
                    )
                    continue

                self._publish_guarded_command(
                    leg_name,
                    leg_index,
                    cmd_position,
                    cmd_velocity,
                    support_leg,
                    normal_force_limit,
                )
            rate.sleep()


if __name__ == "__main__":
    node = SwingLegController()
    node.spin()
