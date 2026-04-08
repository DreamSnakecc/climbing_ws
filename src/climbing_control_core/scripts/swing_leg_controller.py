#!/usr/bin/env python

import math

import rospy
from climbing_msgs.msg import BodyReference, EstimatedState, LegCenterCommand
from geometry_msgs.msg import Point, Vector3


def clamp(value, lower_bound, upper_bound):
    return max(lower_bound, min(upper_bound, value))


def smoothstep5(phase):
    phase = clamp(phase, 0.0, 1.0)
    return phase * phase * phase * (10.0 + phase * (-15.0 + 6.0 * phase))


def smoothstep5_derivative(phase):
    phase = clamp(phase, 0.0, 1.0)
    return 30.0 * phase * phase * (1.0 - phase) * (1.0 - phase)


def raised_cosine(phase):
    phase = clamp(phase, 0.0, 1.0)
    return 0.5 - 0.5 * math.cos(2.0 * math.pi * phase)


def raised_cosine_derivative(phase):
    phase = clamp(phase, 0.0, 1.0)
    return math.pi * math.sin(2.0 * math.pi * phase)


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
    PHASE_DETACH_SLIDE = "DETACH_SLIDE"
    PHASE_TANGENTIAL_ALIGN = "TANGENTIAL_ALIGN"
    PHASE_PRELOAD_COMPRESS = "PRELOAD_COMPRESS"
    PHASE_FAN_ATTACH = "FAN_ATTACH"
    PHASE_COMPLIANT_SETTLE = "COMPLIANT_SETTLE"
    PHASE_ATTACHED_HOLD = "ATTACHED_HOLD"

    def __init__(self):
        rospy.init_node("swing_leg_controller", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/swing_leg_controller/" + name, default))

        self.leg_names = [str(name) for name in get_cfg("leg_names", ["lf", "rf", "lr", "rr"])]
        self.rate_hz = float(get_cfg("publish_rate_hz", 50.0))
        self.swing_duration_s = max(float(get_cfg("swing_duration_s", 0.65)), 0.1)
        self.clearance_m = float(get_cfg("clearance_m", 0.035))
        self.retraction_m = float(get_cfg("retraction_m", 0.015))
        self.touchdown_extension_m = float(get_cfg("touchdown_extension_m", 0.008))
        self.slip_clearance_gain = float(get_cfg("slip_clearance_gain", 0.5))
        self.raibert_height_gain = float(get_cfg("raibert_height_gain", 1.0))
        self.raibert_velocity_gain = float(get_cfg("raibert_velocity_gain", 1.0))
        self.foot_delta_x_limit = float(get_cfg("foot_delta_x_limit_m", 0.10))
        self.foot_delta_y_limit = float(get_cfg("foot_delta_y_limit_m", 0.10))
        self.nominal_skirt_compression_target = float(get_cfg("nominal_skirt_compression_target", 0.0))
        self.swing_skirt_compression_target = float(get_cfg("swing_skirt_compression_target", 0.0))
        self.max_position_offset = [float(value) for value in get_cfg("max_position_offset_m", [0.045, 0.035, 0.05])]
        self.max_velocity = [float(value) for value in get_cfg("max_velocity_mps", [0.30, 0.25, 0.30])]
        self.feedforward_twist_gain = [float(value) for value in get_cfg("feedforward_twist_gain", [0.16, 0.16, 0.0])]
        self.body_position_gain = [float(value) for value in get_cfg("body_position_gain", [0.70, 0.70, 0.35])]
        self.body_velocity_gain = [float(value) for value in get_cfg("body_velocity_gain", [0.12, 0.12, 0.08])]
        self.body_angular_position_gain = [float(value) for value in get_cfg("body_angular_position_gain", [0.10, 0.10, 0.06])]
        self.body_angular_velocity_gain = [float(value) for value in get_cfg("body_angular_velocity_gain", [0.02, 0.02, 0.01])]
        self.admittance_mass = [float(value) for value in get_cfg("admittance_mass", [0.8, 0.8, 0.6])]
        self.admittance_damping = [float(value) for value in get_cfg("admittance_damping", [18.0, 18.0, 16.0])]
        self.admittance_stiffness = [float(value) for value in get_cfg("admittance_stiffness", [85.0, 85.0, 70.0])]
        self.detach_contact_offset_m = max(float(get_cfg("detach_contact_offset_m", 0.006)), 0.0)
        self.detach_slide_duration_s = max(float(get_cfg("detach_slide_duration_s", 0.12)), 0.02)
        self.tangential_align_duration_s = max(float(get_cfg("tangential_align_duration_s", 0.28)), 0.05)
        self.preload_duration_s = max(float(get_cfg("preload_duration_s", 0.16)), 0.05)
        self.fan_attach_command_duration_s = max(float(get_cfg("fan_attach_command_duration_s", 0.05)), 0.0)
        self.compliant_settle_timeout_s = max(float(get_cfg("compliant_settle_timeout_s", 0.40)), 0.05)
        self.preload_extra_normal_m = max(float(get_cfg("preload_extra_normal_m", 0.004)), 0.0)
        self.fan_attach_sink_m = max(float(get_cfg("fan_attach_sink_m", 0.008)), 0.0)
        self.tangential_alignment_tolerance_m = max(float(get_cfg("tangential_alignment_tolerance_m", 0.008)), 1e-4)
        self.normal_alignment_tolerance_m = max(float(get_cfg("normal_alignment_tolerance_m", 0.004)), 1e-4)
        self.light_contact_skirt_compression_target = float(get_cfg("light_contact_skirt_compression_target", 0.20))
        self.preload_skirt_compression_target = float(get_cfg("preload_skirt_compression_target", 0.85))
        self.preload_normal_force_limit_n = max(float(get_cfg("preload_normal_force_limit_n", 15.0)), 0.0)
        self.attach_normal_force_limit_n = max(float(get_cfg("attach_normal_force_limit_n", 25.0)), 0.0)
        self.fan_attach_current_threshold_a = max(float(get_cfg("fan_attach_current_threshold_a", 0.10)), 0.0)
        self.fan_attach_current_full_scale_a = max(float(get_cfg("fan_attach_current_full_scale_a", 0.35)), self.fan_attach_current_threshold_a + 1e-3)
        self.detach_velocity_limit = [float(value) for value in get_cfg("detach_velocity_limit_mps", [0.05, 0.05, 0.05])]
        self.tangential_velocity_limit = [float(value) for value in get_cfg("tangential_velocity_limit_mps", [0.12, 0.12, 0.03])]
        self.preload_velocity_limit = [float(value) for value in get_cfg("preload_velocity_limit_mps", [0.04, 0.04, 0.03])]
        self.compliant_velocity_limit = [float(value) for value in get_cfg("compliant_velocity_limit_mps", [0.03, 0.03, 0.035])]
        self.compliant_tangential_gain = max(float(get_cfg("compliant_tangential_gain", 8.0)), 0.0)
        self.compliant_normal_gain = max(float(get_cfg("compliant_normal_gain", 10.0)), 0.0)

        legacy_nominal_z_mm = float(rospy.get_param("/gait_controller/nominal_z", -299.2))
        self.universal_joint_rigid_offset_m = abs(
            float(rospy.get_param("/gait_controller/link_d6", -13.5)) + float(rospy.get_param("/gait_controller/link_d7", -106.7))
        ) / 1000.0
        self.nominal_z_m = float(
            rospy.get_param(
                "/gait_controller/nominal_universal_joint_center_z",
                legacy_nominal_z_mm + self.universal_joint_rigid_offset_m * 1000.0,
            )
        ) / 1000.0
        self.base_radius_m = float(rospy.get_param("/gait_controller/base_radius", 203.06)) / 1000.0
        self.leg_yaw_rad = self._build_leg_yaw_map()
        self.wall_normal_body = normalize_vector(
            [float(value) for value in get_cfg("wall_normal_body", rospy.get_param("/wall/normal_body", [0.0, 0.0, 1.0]))],
            [0.0, 0.0, 1.0],
        )

        self.body_reference = BodyReference()
        self.estimated_state = EstimatedState()
        self.have_body_reference = False
        self.have_estimated_state = False
        self.last_update_time = None
        self.swing_phase_start = {}
        self.swing_states = {}
        for leg_name in self.leg_names:
            nominal = [0.0, 0.0, self.nominal_z_m]
            self.swing_phase_start[leg_name] = None
            self.swing_states[leg_name] = {
                "position": list(nominal),
                "velocity": [0.0, 0.0, 0.0],
                "start": list(nominal),
                "target": list(nominal),
                "support": True,
                "phase": self.PHASE_SUPPORT,
                "phase_started_at": None,
                "light_contact_target": list(nominal),
                "tangential_target": list(nominal),
                "preload_target": list(nominal),
                "attach_target": list(nominal),
            }

        self.pub = rospy.Publisher("/control/swing_leg_target", LegCenterCommand, queue_size=50)
        rospy.Subscriber("/control/body_reference", BodyReference, self.body_reference_callback, queue_size=20)
        rospy.Subscriber("/state/estimated", EstimatedState, self.estimated_state_callback, queue_size=20)

    def _build_leg_yaw_map(self):
        leg_cfg = rospy.get_param("/legs", {})
        yaw_map = {}
        for leg_name in self.leg_names:
            yaw_deg = float(leg_cfg.get(leg_name, {}).get("hip_yaw_deg", 0.0))
            yaw_map[leg_name] = math.radians(yaw_deg)
        return yaw_map

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
        if leg_index < len(self.estimated_state.slip_risk):
            return clamp(float(self.estimated_state.slip_risk[leg_index]), 0.0, 1.0)
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

    def _step_toward_target(self, current_position, target_position, velocity_limits, dt):
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
        return self._clamp_position(next_position), next_velocity

    def _clamp_position(self, position):
        return [
            clamp(position[0], -self.max_position_offset[0], self.max_position_offset[0]),
            clamp(position[1], -self.max_position_offset[1], self.max_position_offset[1]),
            clamp(position[2], self.nominal_z_m - self.max_position_offset[2], self.nominal_z_m + self.max_position_offset[2]),
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

    def _target_delta(self, leg_name, leg_index):
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

        angular_error = self._body_angular_position_error()
        angular_velocity_error = self._body_angular_velocity_error()
        angular_term = vector_add(
            vector_mul(self.body_angular_position_gain, angular_error),
            vector_mul(self.body_angular_velocity_gain, angular_velocity_error),
        )
        angular_correction = cross(angular_term, self._hip_offset(leg_name))
        slip_bias = [0.0, 0.0, self.slip_clearance_gain * self.clearance_m * self._leg_slip_risk(leg_index)]

        raw_target = vector_add(raibert_delta, feedforward_term)
        raw_target = vector_add(raw_target, body_position_term)
        raw_target = vector_add(raw_target, body_velocity_term)
        raw_target = vector_add(raw_target, angular_correction)
        raw_target = vector_add(raw_target, slip_bias)

        return [
            clamp(raw_target[0], -min(self.max_position_offset[0], self.foot_delta_x_limit), min(self.max_position_offset[0], self.foot_delta_x_limit)),
            clamp(raw_target[1], -min(self.max_position_offset[1], self.foot_delta_y_limit), min(self.max_position_offset[1], self.foot_delta_y_limit)),
            clamp(raw_target[2], -self.max_position_offset[2], self.max_position_offset[2]),
        ]

    def _start_swing(self, leg_name, leg_index, stamp_sec):
        state = self.swing_states[leg_name]
        target_delta = self._target_delta(leg_name, leg_index)
        target = [target_delta[0], target_delta[1], self.nominal_z_m + target_delta[2]]
        start = list(state["position"])
        start_normal = vector_dot(start, self.wall_normal_body)
        target_normal = vector_dot(target, self.wall_normal_body)
        light_contact_target = self._compose_tangent_and_normal(start, start_normal - self.detach_contact_offset_m)
        tangential_target = self._compose_tangent_and_normal(target, start_normal - self.detach_contact_offset_m)
        preload_target = self._compose_tangent_and_normal(target, target_normal + self.preload_extra_normal_m)
        attach_target = self._compose_tangent_and_normal(target, target_normal + self.preload_extra_normal_m + self.fan_attach_sink_m)

        state["start"] = list(start)
        state["target"] = list(target)
        state["light_contact_target"] = self._clamp_position(light_contact_target)
        state["tangential_target"] = self._clamp_position(tangential_target)
        state["preload_target"] = self._clamp_position(preload_target)
        state["attach_target"] = self._clamp_position(attach_target)
        state["support"] = False
        self._set_phase(state, self.PHASE_DETACH_SLIDE, stamp_sec)
        self.swing_phase_start[leg_name] = stamp_sec

    def _support_command(self, leg_name, leg_index):
        state = self.swing_states[leg_name]
        state["support"] = True
        self.swing_phase_start[leg_name] = None
        state["velocity"] = [0.0, 0.0, 0.0]
        support_target = [0.0, 0.0, self.nominal_z_m]
        state["position"] = list(support_target)
        state["start"] = list(support_target)
        state["target"] = list(support_target)
        state["light_contact_target"] = list(support_target)
        state["tangential_target"] = list(support_target)
        state["preload_target"] = list(support_target)
        state["attach_target"] = list(support_target)
        state["phase"] = self.PHASE_SUPPORT
        state["phase_started_at"] = None
        return self._build_message(
            leg_name,
            support_target,
            [0.0, 0.0, 0.0],
            True,
            self.nominal_skirt_compression_target,
            self._leg_normal_force_limit(leg_index),
        )

    def _reference_trajectory(self, leg_name, leg_index, phase):
        state = self.swing_states[leg_name]
        start = state["start"]
        target = state["target"]
        swing_clearance = self.clearance_m * (1.0 + self.slip_clearance_gain * self._leg_slip_risk(leg_index))

        retraction = self.retraction_m * (1.0 - phase)
        touchdown_extension = self.touchdown_extension_m * phase

        ref_position = [0.0, 0.0, 0.0]
        ref_velocity = [0.0, 0.0, 0.0]
        for axis in [0, 1]:
            control_points = [start[axis], start[axis], target[axis], target[axis], target[axis]]
            ref_position[axis] = bezier4(control_points, phase)
            ref_velocity[axis] = bezier4_derivative(control_points, phase) / self.swing_duration_s

        z_control_points = [start[2], start[2] + swing_clearance, target[2] + swing_clearance, target[2], target[2]]
        ref_position[2] = bezier4(z_control_points, phase)
        ref_velocity[2] = bezier4_derivative(z_control_points, phase) / self.swing_duration_s
        ref_position[0] -= retraction
        ref_position[0] += touchdown_extension
        ref_velocity[0] += (self.retraction_m + self.touchdown_extension_m) / self.swing_duration_s

        return ref_position, ref_velocity

    def _update_admittance(self, leg_name, ref_position, ref_velocity, dt):
        state = self.swing_states[leg_name]
        next_velocity = []
        next_position = []
        for axis in [0, 1, 2]:
            position_error = ref_position[axis] - state["position"][axis]
            velocity_error = ref_velocity[axis] - state["velocity"][axis]
            acceleration = (
                self.admittance_stiffness[axis] * position_error
                + self.admittance_damping[axis] * velocity_error
            ) / max(self.admittance_mass[axis], 1e-3)
            velocity = state["velocity"][axis] + acceleration * dt
            velocity = clamp(velocity, -self.max_velocity[axis], self.max_velocity[axis])
            position = state["position"][axis] + velocity * dt
            next_velocity.append(velocity)
            next_position.append(position)

        next_position = self._clamp_position(next_position)

        state["position"] = next_position
        state["velocity"] = next_velocity
        return next_position, next_velocity

    def _guided_swing_command(self, leg_name, leg_index, now_sec, dt):
        state = self.swing_states[leg_name]
        phase = state["phase"]
        phase_elapsed = self._phase_elapsed(state, now_sec)
        compression_ready = self._leg_mask_value(self.estimated_state.compression_ready_mask, leg_index, False)
        preload_ready = self._leg_mask_value(self.estimated_state.preload_ready_mask, leg_index, compression_ready)
        wall_touch = self._leg_mask_value(self.estimated_state.wall_touch_mask, leg_index, False)
        attachment_ready = self._leg_mask_value(self.estimated_state.attachment_ready_mask, leg_index, False)
        adhesion_ready = self._leg_mask_value(self.estimated_state.adhesion_mask, leg_index, attachment_ready)
        measured_contact = self._leg_mask_value(self.estimated_state.measured_contact_mask, leg_index, False)
        early_contact = self._leg_mask_value(self.estimated_state.early_contact_mask, leg_index, False)
        fan_current = self._leg_fan_current(leg_index)
        seal_confidence = self._leg_float_value(self.estimated_state.seal_confidence, leg_index, 0.0)
        torque_contact_confidence = self._leg_float_value(self.estimated_state.leg_torque_contact_confidence, leg_index, 0.0)

        if phase == self.PHASE_DETACH_SLIDE:
            cmd_position, cmd_velocity = self._step_toward_target(state["position"], state["light_contact_target"], self.detach_velocity_limit, dt)
            if abs(self._normal_error(state["light_contact_target"], cmd_position)) <= self.normal_alignment_tolerance_m or phase_elapsed >= self.detach_slide_duration_s:
                self._set_phase(state, self.PHASE_TANGENTIAL_ALIGN, now_sec)
            skirt_target = self.light_contact_skirt_compression_target
            normal_force_limit = 0.0
            support_leg = False
        elif phase == self.PHASE_TANGENTIAL_ALIGN:
            cmd_position, cmd_velocity = self._step_toward_target(state["position"], state["tangential_target"], self.tangential_velocity_limit, dt)
            if self._tangential_error_norm(state["tangential_target"], cmd_position) <= self.tangential_alignment_tolerance_m or phase_elapsed >= self.tangential_align_duration_s or wall_touch:
                self._set_phase(state, self.PHASE_PRELOAD_COMPRESS, now_sec)
            skirt_target = self.light_contact_skirt_compression_target
            normal_force_limit = 0.0
            support_leg = False
        elif phase == self.PHASE_PRELOAD_COMPRESS:
            cmd_position, cmd_velocity = self._step_toward_target(state["position"], state["preload_target"], self.preload_velocity_limit, dt)
            if preload_ready or abs(self._normal_error(state["preload_target"], cmd_position)) <= self.normal_alignment_tolerance_m or phase_elapsed >= self.preload_duration_s:
                self._set_phase(state, self.PHASE_FAN_ATTACH, now_sec)
            skirt_target = self.preload_skirt_compression_target
            normal_force_limit = self.preload_normal_force_limit_n
            support_leg = False
        elif phase == self.PHASE_FAN_ATTACH:
            cmd_position, cmd_velocity = self._step_toward_target(state["position"], state["preload_target"], self.preload_velocity_limit, dt)
            if phase_elapsed >= self.fan_attach_command_duration_s:
                self._set_phase(state, self.PHASE_COMPLIANT_SETTLE, now_sec)
            skirt_target = self.preload_skirt_compression_target
            normal_force_limit = self.attach_normal_force_limit_n
            support_leg = False
        elif phase == self.PHASE_COMPLIANT_SETTLE:
            current_tangent = self._tangential_component(state["position"])
            target_tangent = self._tangential_component(state["attach_target"])
            tangential_error = vector_sub(target_tangent, current_tangent)
            tangential_velocity = vector_scale(tangential_error, self.compliant_tangential_gain * max(0.15, 1.0 - torque_contact_confidence))

            sink_ratio = clamp(
                (fan_current - self.fan_attach_current_threshold_a)
                / max(self.fan_attach_current_full_scale_a - self.fan_attach_current_threshold_a, 1e-3),
                0.0,
                1.0,
            )
            sink_ratio = max(sink_ratio, seal_confidence, 0.5 * torque_contact_confidence)
            target_normal_scalar = vector_dot(state["preload_target"], self.wall_normal_body) + sink_ratio * self.fan_attach_sink_m
            current_normal_scalar = vector_dot(state["position"], self.wall_normal_body)
            normal_gain = self.compliant_normal_gain * (1.0 + 0.5 * torque_contact_confidence)
            normal_velocity_scalar = normal_gain * (target_normal_scalar - current_normal_scalar)
            desired_velocity = vector_add(tangential_velocity, vector_scale(self.wall_normal_body, normal_velocity_scalar))
            cmd_velocity = []
            cmd_position = []
            for axis in [0, 1, 2]:
                velocity = clamp(desired_velocity[axis], -self.compliant_velocity_limit[axis], self.compliant_velocity_limit[axis])
                cmd_velocity.append(velocity)
                cmd_position.append(state["position"][axis] + velocity * dt)
            cmd_position = self._clamp_position(cmd_position)
            if attachment_ready or adhesion_ready:
                self._set_phase(state, self.PHASE_ATTACHED_HOLD, now_sec)
            skirt_target = self.preload_skirt_compression_target
            normal_force_limit = self.attach_normal_force_limit_n
            support_leg = False
        else:
            cmd_position = list(state["position"])
            cmd_velocity = [0.0, 0.0, 0.0]
            skirt_target = self.preload_skirt_compression_target if phase == self.PHASE_ATTACHED_HOLD else self.nominal_skirt_compression_target
            normal_force_limit = self._leg_normal_force_limit(leg_index)
            support_leg = phase == self.PHASE_ATTACHED_HOLD

        if phase == self.PHASE_ATTACHED_HOLD:
            cmd_position = list(state["attach_target"])
            cmd_velocity = [0.0, 0.0, 0.0]
            skirt_target = self.preload_skirt_compression_target
            normal_force_limit = self.attach_normal_force_limit_n
            support_leg = True

        if phase == self.PHASE_COMPLIANT_SETTLE and phase_elapsed >= self.compliant_settle_timeout_s and not (attachment_ready or adhesion_ready):
            cmd_velocity = [0.0, 0.0, 0.0]

        if (measured_contact or early_contact or wall_touch) and phase in [self.PHASE_TANGENTIAL_ALIGN, self.PHASE_PRELOAD_COMPRESS]:
            skirt_target = max(skirt_target, self.light_contact_skirt_compression_target)

        state["position"] = list(cmd_position)
        state["velocity"] = list(cmd_velocity)
        return cmd_position, cmd_velocity, support_leg, skirt_target, normal_force_limit

    def _build_message(self, leg_name, position, velocity, support_leg, skirt_compression_target, normal_force_limit):
        msg = LegCenterCommand()
        msg.header.stamp = rospy.Time.now()
        msg.leg_name = leg_name
        msg.center = Point(position[0], position[1], position[2])
        msg.center_velocity = Vector3(velocity[0], velocity[1], velocity[2])
        msg.skirt_compression_target = float(skirt_compression_target)
        msg.support_leg = bool(support_leg)
        msg.desired_normal_force_limit = float(normal_force_limit)
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
            estimated_support_mask = self._estimated_support_mask()

            for leg_index, leg_name in enumerate(self.leg_names):
                desired_support = desired_support_mask[leg_index] if leg_index < len(desired_support_mask) else True
                attachment_ready = self._leg_mask_value(self.estimated_state.attachment_ready_mask, leg_index, False)
                adhesion_ready = self._leg_mask_value(self.estimated_state.adhesion_mask, leg_index, attachment_ready)

                if desired_support and self.swing_phase_start[leg_name] is None:
                    self.pub.publish(self._support_command(leg_name, leg_index))
                    continue

                if self.swing_phase_start[leg_name] is None and not desired_support:
                    self._start_swing(leg_name, leg_index, now_sec)

                if self.swing_phase_start[leg_name] is None:
                    self.pub.publish(self._support_command(leg_name, leg_index))
                    continue

                if desired_support and (attachment_ready or adhesion_ready):
                    self.swing_phase_start[leg_name] = None
                    self.pub.publish(self._support_command(leg_name, leg_index))
                    continue

                state = self.swing_states[leg_name]
                if state["phase"] == self.PHASE_ATTACHED_HOLD and desired_support and (attachment_ready or adhesion_ready):
                    self.swing_phase_start[leg_name] = None
                    self.pub.publish(self._support_command(leg_name, leg_index))
                    continue

                cmd_position, cmd_velocity, support_leg, skirt_target, normal_force_limit = self._guided_swing_command(leg_name, leg_index, now_sec, dt)
                self.pub.publish(
                    self._build_message(
                        leg_name,
                        cmd_position,
                        cmd_velocity,
                        support_leg,
                        skirt_target,
                        normal_force_limit,
                    )
                )
            rate.sleep()


if __name__ == "__main__":
    node = SwingLegController()
    node.spin()