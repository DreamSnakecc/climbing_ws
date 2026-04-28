#!/usr/bin/env python

import math

import numpy as np

import rospy
from climbing_msgs.msg import BodyReference, EstimatedState, LegCenterCommand
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, MultiArrayLayout, String


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
    PHASE_TEST_LIFT_CLEARANCE = "TEST_LIFT_CLEARANCE"
    PHASE_TEST_PRESS_CONTACT = "TEST_PRESS_CONTACT"
    PHASE_DETACH_SLIDE = "DETACH_SLIDE"
    PHASE_TANGENTIAL_ALIGN = "TANGENTIAL_ALIGN"
    PHASE_PRELOAD_COMPRESS = "PRELOAD_COMPRESS"
    PHASE_COMPLIANT_SETTLE = "COMPLIANT_SETTLE"
    PHASE_ATTACHED_HOLD = "ATTACHED_HOLD"

    def __init__(self):
        rospy.init_node("swing_leg_controller", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/swing_leg_controller/" + name, default))

        self.leg_names = [str(name) for name in get_cfg("leg_names", ["lf", "rf", "rr", "lr"])]
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
        self.compliant_settle_timeout_s = max(float(get_cfg("compliant_settle_timeout_s", 0.40)), 0.05)
        self.preload_extra_normal_m = max(float(get_cfg("preload_extra_normal_m", 0.004)), 0.0)
        self.fan_attach_sink_m = max(float(get_cfg("fan_attach_sink_m", 0.008)), 0.0)
        self.tangential_alignment_tolerance_m = max(float(get_cfg("tangential_alignment_tolerance_m", 0.008)), 1e-4)
        self.normal_alignment_tolerance_m = max(float(get_cfg("normal_alignment_tolerance_m", 0.004)), 1e-4)
        self.light_contact_skirt_compression_target = float(get_cfg("light_contact_skirt_compression_target", 0.20))
        self.preload_skirt_compression_target = float(get_cfg("preload_skirt_compression_target", 0.85))
        self.preload_normal_force_limit_n = max(float(get_cfg("preload_normal_force_limit_n", 15.0)), 0.0)
        self.attach_normal_force_limit_n = max(float(get_cfg("attach_normal_force_limit_n", 25.0)), 0.0)
        self.detach_velocity_limit = [float(value) for value in get_cfg("detach_velocity_limit_mps", [0.05, 0.05, 0.05])]
        self.tangential_velocity_limit = [float(value) for value in get_cfg("tangential_velocity_limit_mps", [0.12, 0.12, 0.03])]
        self.preload_velocity_limit = [float(value) for value in get_cfg("preload_velocity_limit_mps", [0.04, 0.04, 0.03])]
        self.compliant_velocity_limit = [float(value) for value in get_cfg("compliant_velocity_limit_mps", [0.03, 0.03, 0.035])]
        self.test_lift_velocity_limit = [float(value) for value in get_cfg("test_lift_velocity_limit_mps", [0.08, 0.08, 0.08])]
        self.test_press_velocity_limit = [float(value) for value in get_cfg("test_press_velocity_limit_mps", [0.04, 0.04, 0.04])]
        # Dwell times keep TEST_LIFT_CLEARANCE / TEST_PRESS_CONTACT held at the alignment target long enough to be observable on the wire.
        self.test_lift_dwell_s = max(float(get_cfg("test_lift_dwell_s", 0.0)), 0.0)
        self.test_press_dwell_s = max(float(get_cfg("test_press_dwell_s", 0.0)), 0.0)
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
        self.compliant_start_requires_contact = bool(get_cfg("compliant_start_requires_contact", False))
        # Force support_leg=False when mission_state is INIT.
        # Prevent dynamixel_bridge from switching motors to current mode on PC startup.
        # Only enforced in INIT; stick/climb/pause/fault follow original support_leg.
        self.hold_position_states = set(
            str(value) for value in get_cfg("hold_position_states", ["INIT"])
        )
        # Global contact feedback switch. When false, all contact/adhesion metrics
        # are treated as false; swing sequence driven purely by trajectory+time
        # Useful when contact thresholds are uncalibrated
        self.use_contact_feedback = bool(get_cfg("use_contact_feedback", True))
        self.contact_hold_min_s = max(float(get_cfg("contact_hold_min_s", 0.04)), 0.0)
        # Lift above the support plane during TANGENTIAL_ALIGN
        # Default matches clearance_m; override via swing_lift_normal_m.
        self.swing_lift_normal_m = max(float(get_cfg("swing_lift_normal_m", self.clearance_m)), 0.0)
        # PRESS-phase delta-contact detector: baseline (sum|joint effort|, sum|joint current|) is
        # captured on the first PRESS tick (leg just came off static LIFT hold, still in free air),
        # then contact is confirmed when either aggregate rises above its per-motor threshold x N.
        self.press_contact_torque_delta_nm = max(float(get_cfg("press_contact_torque_delta_nm", 0.10)), 0.0)
        self.press_contact_current_delta_a = max(float(get_cfg("press_contact_current_delta_a", 0.20)), 0.0)
        self.press_contact_require_both = bool(get_cfg("press_contact_require_both", False))
        self.jacobian_delta_rad = max(float(get_cfg("jacobian_delta_rad", 1e-3)), 1e-5)

        legacy_nominal_z_mm = float(rospy.get_param("/gait_controller/nominal_z", -299.2))
        self.nominal_x_m = float(rospy.get_param("/gait_controller/nominal_x", 118.75)) / 1000.0
        self.nominal_y_m = float(rospy.get_param("/gait_controller/nominal_y", 0.0)) / 1000.0
        self.l_coxa_m = float(rospy.get_param("/gait_controller/link_coxa", 44.75)) / 1000.0
        self.l_femur_m = float(rospy.get_param("/gait_controller/link_femur", 74.0)) / 1000.0
        self.l_tibia_m = float(rospy.get_param("/gait_controller/link_tibia", 150.0)) / 1000.0
        self.l_a3_m = float(rospy.get_param("/gait_controller/link_a3", 41.5)) / 1000.0
        self.universal_joint_rigid_offset_m = abs(
            float(rospy.get_param("/gait_controller/link_d6", -13.5)) + float(rospy.get_param("/gait_controller/link_d7", -106.7))
        ) / 1000.0
        self.nominal_z_m = float(
            rospy.get_param(
                "/gait_controller/nominal_universal_joint_center_z",
                legacy_nominal_z_mm + self.universal_joint_rigid_offset_m * 1000.0,
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
        self.last_update_time = None
        # Default INIT: before first mission_state treat as idle, motors stay in position mode
        self.mission_state = "INIT"
        self.swing_phase_start = {}
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
                "press_target": list(nominal),
                "light_contact_target": list(nominal),
                "tangential_target": list(nominal),
                "preload_target": list(nominal),
                "attach_target": list(nominal),
                "compliant_joint_torque_bias": [0.0, 0.0, 0.0],
                "compliant_force_estimate": 0.0,
                "compliant_normal_offset": 0.0,
                "compliant_normal_velocity": 0.0,
                "contact_active_since": None,
                "last_joint_vector": [0.0, 0.0, 0.0],
                "test_override_value": None,
                "test_press_value": None,
                "test_experiment_active": False,
                "press_torque_sum_baseline": None,
                "press_current_sum_baseline": None,
                "press_contact_confirmed": False,
                "press_contact_confirmed_at": None,
                "press_torque_delta": 0.0,
                "press_current_delta": 0.0,
            }

        self.pub = rospy.Publisher("/control/swing_leg_target", LegCenterCommand, queue_size=50)
        # Per-leg diagnostic stream so the test/observer can recover state-machine internals (phase id,
        # admittance offset/velocity, filtered force estimate, etc.) without scraping the controller's
        # in-memory dicts. Layout label is documented in _publish_leg_diagnostic.
        self.diagnostic_pubs = {
            leg_name: rospy.Publisher(
                "/control/swing_leg_diag/" + leg_name, Float32MultiArray, queue_size=20
            )
            for leg_name in self.leg_names
        }
        self.phase_id_map = {
            self.PHASE_SUPPORT: 0,
            self.PHASE_TEST_LIFT_CLEARANCE: 1,
            self.PHASE_TEST_PRESS_CONTACT: 2,
            self.PHASE_DETACH_SLIDE: 3,
            self.PHASE_TANGENTIAL_ALIGN: 4,
            self.PHASE_PRELOAD_COMPRESS: 5,
            self.PHASE_COMPLIANT_SETTLE: 6,
            self.PHASE_ATTACHED_HOLD: 7,
        }
        self.diagnostic_field_labels = [
            "leg_index",
            "phase_id",
            "phase_elapsed_s",
            "cmd_normal_from_nominal_m",
            "lift_target_normal_from_nominal_m",
            "press_target_normal_from_nominal_m",
            "preload_target_normal_from_nominal_m",
            "attach_target_normal_from_nominal_m",
            "compliant_force_estimate_n",
            "compliant_normal_offset_m",
            "compliant_normal_velocity_mps",
            "estimated_leg_normal_force_n",
            "joint_torque_bias_norm_nm",
            "contact_active_since_age_s",
            "test_experiment_active",
            "test_hold_at_lift",
            "test_lift_aligned",
            "leg_torque_sum_nm",
            "leg_current_sum_a",
            "press_torque_baseline_nm",
            "press_current_baseline_a",
            "press_torque_delta_nm",
            "press_current_delta_a",
            "press_contact_confirmed",
        ]
        rospy.Subscriber("/control/body_reference", BodyReference, self.body_reference_callback, queue_size=20)
        rospy.Subscriber("/state/estimated", EstimatedState, self.estimated_state_callback, queue_size=20)
        rospy.Subscriber("/control/mission_state", String, self._mission_state_callback, queue_size=10)

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
        state["phase_start_pos"] = list(state.get("position", [0.0, 0.0, 0.0]))
        # Hold/aligned flags only describe the LIFT phase; clear them on any other transition
        # so the diagnostic vector doesn't report stale values.
        if phase_name != self.PHASE_TEST_LIFT_CLEARANCE:
            state["test_hold_at_lift"] = False
            state["test_lift_aligned"] = False

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
        """Bezier4 position interpolation from start to target over given duration.

        Returns [pos_x, pos_y, pos_z], [0, 0, 0].
        The Bezier control points are [start, start, target, target, target]
        so motion starts and ends smoothly at zero velocity.
        """
        phase = min(phase_elapsed / max(duration_s, 1e-3), 1.0)
        pos = [0.0, 0.0, 0.0]
        for axis in [0, 1, 2]:
            cp = [start_pos[axis], start_pos[axis], target_pos[axis], target_pos[axis], target_pos[axis]]
            pos[axis] = bezier4(cp, phase)
        return pos

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
        """LegCenterCommand.center that maps to the operating UJC in leg frame 0."""
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
            reference_rad = [0.0, 0.0, 0.0]
        return min(candidates, key=lambda candidate: self._solution_cost(candidate, reference_rad))

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

    def _aggregate_leg_abs_sum(self, leg_name, values):
        """Sum of |values[joint_index]| over this leg's motor ids.

        Mirrors state_estimator._aggregate_leg_scalar so the delta-contact detector matches
        the same raw-effort / raw-current units used elsewhere. Returns (total, count).
        """
        motor_ids = self.leg_to_motors.get(leg_name, [])
        total = 0.0
        count = 0
        if not values:
            return total, count
        for motor_id in motor_ids:
            joint_index = self.joint_name_to_index.get(str(int(motor_id)))
            if joint_index is None or joint_index >= len(values):
                continue
            total += abs(float(values[joint_index]))
            count += 1
        return total, count

    def _leg_torque_sum_nm(self, leg_name):
        values = list(self.estimated_state.joint_torques_est) if self.estimated_state.joint_torques_est else []
        total, _count = self._aggregate_leg_abs_sum(leg_name, values)
        return total

    def _leg_current_sum_a(self, leg_name):
        values = list(self.estimated_state.joint_currents) if self.estimated_state.joint_currents else []
        total, _count = self._aggregate_leg_abs_sum(leg_name, values)
        return total

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
        # Bidirectional admittance around the nominal preload point: the offset floats in
        # [-offset_limit, +offset_limit] along wall_normal_body so the leg can either retract
        # (outward, +n) when the wall over-reacts or extend (inward, -n) when skirt vacuum
        # out-pulls the command. A symmetric deadband suppresses both-sign noise below
        # compliant_force_deadband_n; above it, effective_force preserves the sign of
        # filtered_force (classic sign-preserving deadband).
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
        # Two-sided clamp: positive offset retracts the leg along +n, negative offset extends it
        # along -n. Upstream position clamps still bound the raw foot command.
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

    def _contact_hold_satisfied(self, state, contact_active, now_sec):
        if contact_active:
            if state["contact_active_since"] is None:
                state["contact_active_since"] = now_sec
            return (now_sec - float(state["contact_active_since"])) >= self.contact_hold_min_s
        state["contact_active_since"] = None
        return False

    def _test_trigger_normal_override(self, leg_name):
        override_leg_name = str(
            rospy.get_param(
                "/swing_leg_controller/test_trigger_leg_name",
                rospy.get_param("~test_trigger_leg_name", ""),
            )
        ).strip()
        if override_leg_name != str(leg_name):
            return None

        override_normal_travel = float(
            rospy.get_param(
                "/swing_leg_controller/test_trigger_normal_travel_m",
                rospy.get_param("~test_trigger_normal_travel_m", 0.0),
            )
        )
        max_normal_travel = max(self.max_position_offset)
        return clamp(override_normal_travel, -max_normal_travel, max_normal_travel)

    def _test_trigger_press_override(self):
        press_normal_travel = float(
            rospy.get_param(
                "/swing_leg_controller/test_trigger_press_normal_travel_m",
                rospy.get_param("~test_trigger_press_normal_travel_m", -0.003),
            )
        )
        max_normal_travel = max(self.max_position_offset)
        return clamp(press_normal_travel, -max_normal_travel, max_normal_travel)

    def _test_trigger_hold_at_lift(self):
        """Runtime flag that blocks the LIFT->PRESS transition while True.

        The flag is polled every control tick so the test script can freeze the
        leg at the lift target, prompt the operator, and then release the hold
        so the staged press/compliance sequence can continue.
        """
        try:
            return bool(
                rospy.get_param(
                    "/swing_leg_controller/test_trigger_hold_at_lift",
                    rospy.get_param("~test_trigger_hold_at_lift", False),
                )
            )
        except Exception:
            return False

    def _test_force_support_reset_enabled(self):
        """Allow force-reset to SUPPORT when running staged tests.

        This is intentionally opt-in via ROS param so normal whole-body control
        behavior is unchanged unless a test harness enables it.
        """
        try:
            return bool(
                rospy.get_param(
                    "/swing_leg_controller/test_force_support_reset_enable",
                    rospy.get_param("~test_force_support_reset_enable", False),
                )
            )
        except Exception:
            return False

    def _apply_test_trigger_override(self, leg_name, state):
        lift_normal_travel = self._test_trigger_normal_override(leg_name)
        if lift_normal_travel is None:
            state["test_override_value"] = None
            state["test_press_value"] = None
            state["test_experiment_active"] = False
            return None

        press_normal_travel = self._test_trigger_press_override()
        operating_center = self._operating_center_command(leg_name)
        lift_target = self._clamp_position(
            vector_add(
                operating_center,
                vector_scale(self.wall_normal_body, lift_normal_travel),
            ),
            operating_center,
        )
        press_target = self._clamp_position(
            vector_add(
                operating_center,
                vector_scale(self.wall_normal_body, press_normal_travel),
            ),
            operating_center,
        )

        state["lift_target"] = list(lift_target)
        state["press_target"] = list(press_target)
        state["support_target"] = list(operating_center)
        if state.get("phase") != self.PHASE_ATTACHED_HOLD:
            state["target"] = list(lift_target)
            state["tangential_target"] = list(lift_target)
            state["preload_target"] = list(press_target)
            state["attach_target"] = list(press_target)
        state["test_override_value"] = lift_normal_travel
        state["test_press_value"] = press_normal_travel
        state["test_experiment_active"] = True

        rospy.loginfo_throttle(
            1.0,
            "swing_leg_controller staged test active for %s: lift_normal=%.4f press_normal=%.4f lift_z=%.4f press_z=%.4f",
            leg_name,
            lift_normal_travel,
            press_normal_travel,
            state["lift_target"][2],
            state["preload_target"][2],
        )
        return lift_normal_travel

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
        override_normal_travel = self._test_trigger_normal_override(leg_name)
        self._apply_test_trigger_override(leg_name, state)
        if override_normal_travel is None:
            support_target = self._operating_center_command(leg_name)
            target = [
                support_target[0] + target_delta[0],
                support_target[1] + target_delta[1],
                support_target[2] + target_delta[2],
            ]
        else:
            target = list(state["lift_target"])
        start = list(state["position"])
        start_normal = vector_dot(start, self.wall_normal_body)
        target_normal = vector_dot(target, self.wall_normal_body)

        # DETACH: Slight retract from contact surface
        light_contact_target = self._compose_tangent_and_normal(start, start_normal - self.detach_contact_offset_m)
        # TANGENTIAL: Move to target XY while lifting clearance_m along normal for obstacle clearance
        tang_normal = start_normal + self.swing_lift_normal_m
        tangential_target = self._compose_tangent_and_normal(target, tang_normal)
        # PRELOAD: Press back to surface from lifted position
        preload_target = self._compose_tangent_and_normal(target, target_normal + self.preload_extra_normal_m)
        # ATTACH: Same XY as preload; COMPLIANT_SETTLE admittance will
        # drive deeper normal based on contact force sensing.
        attach_target = list(preload_target)

        state["start"] = list(start)
        state["target"] = list(target)
        state["support_target"] = list(self._operating_center_command(leg_name))
        state["light_contact_target"] = self._clamp_position(light_contact_target, state["support_target"])
        state["tangential_target"] = self._clamp_position(tangential_target, state["support_target"])
        state["preload_target"] = self._clamp_position(preload_target, state["support_target"])
        state["attach_target"] = self._clamp_position(attach_target, state["support_target"])
        if override_normal_travel is not None:
            state["target"] = list(state["lift_target"])
            state["tangential_target"] = list(state["lift_target"])
            state["preload_target"] = list(state["press_target"])
            state["attach_target"] = list(state["press_target"])
        state["compliant_joint_torque_bias"] = [0.0, 0.0, 0.0]
        state["compliant_force_estimate"] = 0.0
        state["compliant_normal_offset"] = 0.0
        state["compliant_normal_velocity"] = 0.0
        state["contact_active_since"] = None
        state["last_joint_vector"] = self._joint_vector_from_position(leg_name, start, state.get("last_joint_vector"))
        state["support"] = False
        state["press_torque_sum_baseline"] = None
        state["press_current_sum_baseline"] = None
        state["press_contact_confirmed"] = False
        state["press_contact_confirmed_at"] = None
        state["press_torque_delta"] = 0.0
        state["press_current_delta"] = 0.0
        if override_normal_travel is None:
            state["test_experiment_active"] = False
            self._set_phase(state, self.PHASE_DETACH_SLIDE, stamp_sec)
        else:
            self._set_phase(state, self.PHASE_TEST_LIFT_CLEARANCE, stamp_sec)
        self.swing_phase_start[leg_name] = stamp_sec

    def _support_command(self, leg_name, leg_index):
        state = self.swing_states[leg_name]
        state["support"] = True
        self.swing_phase_start[leg_name] = None
        state["velocity"] = [0.0, 0.0, 0.0]
        support_target = self._operating_center_command(leg_name)
        state["support_target"] = list(support_target)
        state["position"] = list(support_target)
        state["start"] = list(support_target)
        state["target"] = list(support_target)
        state["lift_target"] = list(support_target)
        state["press_target"] = list(support_target)
        state["light_contact_target"] = list(support_target)
        state["tangential_target"] = list(support_target)
        state["preload_target"] = list(support_target)
        state["attach_target"] = list(support_target)
        state["compliant_joint_torque_bias"] = [0.0, 0.0, 0.0]
        state["compliant_force_estimate"] = 0.0
        state["compliant_normal_offset"] = 0.0
        state["compliant_normal_velocity"] = 0.0
        state["contact_active_since"] = None
        state["phase"] = self.PHASE_SUPPORT
        state["phase_started_at"] = None
        state["test_override_value"] = None
        state["test_press_value"] = None
        state["test_experiment_active"] = False
        state["test_hold_at_lift"] = False
        state["test_lift_aligned"] = False
        state["press_torque_sum_baseline"] = None
        state["press_current_sum_baseline"] = None
        state["press_contact_confirmed"] = False
        state["press_contact_confirmed_at"] = None
        state["press_torque_delta"] = 0.0
        state["press_current_delta"] = 0.0
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

    def _update_admittance(self, leg_name, ref_position, ref_velocity, dt, clamp_center=None):
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

        next_position = self._clamp_position(next_position, clamp_center)

        state["position"] = next_position
        state["velocity"] = next_velocity
        return next_position, next_velocity

    def _guided_swing_command(self, leg_name, leg_index, now_sec, dt):
        state = self.swing_states[leg_name]
        self._apply_test_trigger_override(leg_name, state)
        phase = state["phase"]
        phase_elapsed = self._phase_elapsed(state, now_sec)
        wall_touch = self._leg_mask_value(self.estimated_state.wall_touch_mask, leg_index, False)
        attachment_ready = self._leg_mask_value(self.estimated_state.attachment_ready_mask, leg_index, False)
        adhesion_ready = self._leg_mask_value(self.estimated_state.adhesion_mask, leg_index, attachment_ready)
        measured_contact = self._leg_mask_value(self.estimated_state.measured_contact_mask, leg_index, False)
        early_contact = self._leg_mask_value(self.estimated_state.early_contact_mask, leg_index, False)
        # Contact switch disabled: ignore all feedback, trajectory+time only
        if not self.use_contact_feedback:
            wall_touch = False
            attachment_ready = False
            adhesion_ready = False
            measured_contact = False
            early_contact = False
        clamp_center = state.get("support_target", self._operating_center_command(leg_name))

        if phase == self.PHASE_TEST_LIFT_CLEARANCE:
            cmd_position, cmd_velocity = self._step_toward_target(state["position"], state["lift_target"], self.test_lift_velocity_limit, dt, clamp_center)
            aligned = abs(self._normal_error(state["lift_target"], cmd_position)) <= self.normal_alignment_tolerance_m
            hold_at_lift = self._test_trigger_hold_at_lift()
            state["test_hold_at_lift"] = hold_at_lift
            state["test_lift_aligned"] = bool(aligned)
            if aligned and phase_elapsed >= self.test_lift_dwell_s and not hold_at_lift:
                self._set_phase(state, self.PHASE_TEST_PRESS_CONTACT, now_sec)
            elif aligned:
                # Hold exactly at the lift target while dwell time accumulates (or while hold_at_lift is set by the operator)
                # so observers can verify the lift was reached before pressing.
                cmd_position = list(state["lift_target"])
                cmd_velocity = [0.0, 0.0, 0.0]
                if hold_at_lift:
                    rospy.loginfo_throttle(
                        2.0,
                        "swing_leg_controller staged test holding at LIFT for %s (waiting on test_trigger_hold_at_lift)",
                        leg_name,
                    )
            skirt_target = self.swing_skirt_compression_target
            normal_force_limit = 0.0
            support_leg = False
        elif phase == self.PHASE_TEST_PRESS_CONTACT:
            # No explicit touchdown detection here: once PRESS target is reached (and optional
            # dwell is satisfied), we immediately enter COMPLIANT_SETTLE. The fan test harness
            # keys off phase progression so fan start and admittance start happen together.
            cmd_position, cmd_velocity = self._step_toward_target(state["position"], state["press_target"], self.test_press_velocity_limit, dt, clamp_center)
            aligned = abs(self._normal_error(state["press_target"], cmd_position)) <= self.normal_alignment_tolerance_m
            if state.get("press_torque_sum_baseline") is None:
                # Keep these diagnostic fields populated for logging compatibility, but they no
                # longer gate phase transitions.
                torque_sum_now = self._leg_torque_sum_nm(leg_name)
                current_sum_now = self._leg_current_sum_a(leg_name)
                state["press_torque_sum_baseline"] = float(torque_sum_now)
                state["press_current_sum_baseline"] = float(current_sum_now)
                state["press_torque_delta"] = 0.0
                state["press_current_delta"] = 0.0
                self._capture_compliant_torque_bias(leg_name, state)

            if aligned and phase_elapsed >= self.test_press_dwell_s:
                state["press_contact_confirmed"] = True
                state["press_contact_confirmed_at"] = now_sec
                self._set_phase(state, self.PHASE_COMPLIANT_SETTLE, now_sec)
            elif aligned:
                cmd_position = list(state["press_target"])
                cmd_velocity = [0.0, 0.0, 0.0]
            skirt_target = self.preload_skirt_compression_target
            normal_force_limit = 0.0
            support_leg = False
        elif phase == self.PHASE_DETACH_SLIDE:
            # Bezier smooth retract from surface (fixed start position).
            start_pos = list(state["phase_start_pos"])
            cmd_position = self._bezier_interp(start_pos, list(state["light_contact_target"]), self.detach_slide_duration_s, phase_elapsed)
            cmd_velocity = [0.0, 0.0, 0.0]
            if abs(self._normal_error(state["light_contact_target"], cmd_position)) <= self.normal_alignment_tolerance_m or phase_elapsed >= self.detach_slide_duration_s:
                self._set_phase(state, self.PHASE_TANGENTIAL_ALIGN, now_sec)
            skirt_target = self.light_contact_skirt_compression_target
            normal_force_limit = 0.0
            support_leg = False
        elif phase == self.PHASE_TANGENTIAL_ALIGN:
            # Bezier smooth swing to target with clearance.
            tang_duration = max(self.tangential_align_duration_s, 0.05)
            phase_progress = min(phase_elapsed / tang_duration, 1.0)
            swing_clearance = self.clearance_m * (1.0 + self.slip_clearance_gain * self._leg_slip_risk(leg_index))
            start_pos = list(state["phase_start_pos"])
            target_xy = list(state["target"])
            ref_pos = [0.0, 0.0, 0.0]
            for axis in [0, 1]:
                cp = [start_pos[axis], start_pos[axis], target_xy[axis], target_xy[axis], target_xy[axis]]
                ref_pos[axis] = bezier4(cp, phase_progress)
            z_start = start_pos[2]
            z_target = state["tangential_target"][2]
            lift_height = z_start + swing_clearance
            z_cp = [z_start, lift_height, lift_height + swing_clearance, z_target, z_target]
            ref_pos[2] = bezier4(z_cp, phase_progress)
            cmd_position = ref_pos
            cmd_velocity = [0.0, 0.0, 0.0]
            if phase_elapsed >= tang_duration:
                self._set_phase(state, self.PHASE_PRELOAD_COMPRESS, now_sec)
            skirt_target = self.light_contact_skirt_compression_target
            normal_force_limit = 0.0
            support_leg = False
        elif phase == self.PHASE_PRELOAD_COMPRESS:
            # Bezier smooth press to preload target (fixed start position).
            start_pos = list(state["phase_start_pos"])
            cmd_position = self._bezier_interp(start_pos, list(state["preload_target"]), self.preload_duration_s, phase_elapsed)
            cmd_velocity = [0.0, 0.0, 0.0]
            if abs(self._normal_error(state["preload_target"], cmd_position)) <= self.normal_alignment_tolerance_m or phase_elapsed >= self.preload_duration_s:
                self._capture_compliant_torque_bias(leg_name, state)
                self._set_phase(state, self.PHASE_COMPLIANT_SETTLE, now_sec)
            skirt_target = self.preload_skirt_compression_target
            normal_force_limit = self.preload_normal_force_limit_n
            support_leg = False
        elif phase == self.PHASE_COMPLIANT_SETTLE:
            contact_active = measured_contact or wall_touch
            contact_stable = self._contact_hold_satisfied(state, contact_active, now_sec)
            admittance_enabled = contact_stable if self.compliant_start_requires_contact else True
            if admittance_enabled:
                measured_force_n = self._estimate_leg_normal_force(leg_name, state)
                normal_offset, normal_velocity = self._update_normal_admittance(state, measured_force_n, dt)
                target_normal_scalar = vector_dot(state["preload_target"], self.wall_normal_body) + normal_offset
                cmd_position = self._clamp_position(
                    self._compose_tangent_and_normal(state["attach_target"], target_normal_scalar),
                    clamp_center,
                )
                cmd_velocity = vector_scale(self.wall_normal_body, normal_velocity)
            else:
                cmd_position = list(state["preload_target"])
                cmd_velocity = [0.0, 0.0, 0.0]
                state["compliant_normal_velocity"] = 0.0
            if attachment_ready or adhesion_ready:
                self._freeze_compliant_state(state, cmd_position)
                self._set_phase(state, self.PHASE_ATTACHED_HOLD, now_sec)
                cmd_position = list(state["attach_target"])
                cmd_velocity = [0.0, 0.0, 0.0]
            # When contact off, use compliant_settle_timeout_s as COMPLIANT_SETTLE->ATTACHED_HOLD
            # fallback to avoid stall without contact feedback
            elif not self.use_contact_feedback and phase_elapsed >= self.compliant_settle_timeout_s:
                self._freeze_compliant_state(state, cmd_position)
                self._set_phase(state, self.PHASE_ATTACHED_HOLD, now_sec)
                cmd_position = list(state["attach_target"])
                cmd_velocity = [0.0, 0.0, 0.0]
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
            state["compliant_normal_velocity"] = 0.0

        if (measured_contact or early_contact or wall_touch) and phase in [self.PHASE_TANGENTIAL_ALIGN, self.PHASE_PRELOAD_COMPRESS, self.PHASE_TEST_PRESS_CONTACT]:
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
        # During hold_position_states (INIT), set support_leg=False.
        # Prevent dynamixel_bridge switching to current mode and losing position hold
        # Center command still published; IK continues holding operating UJC target
        if self.mission_state in self.hold_position_states:
            msg.support_leg = False
        else:
            msg.support_leg = bool(support_leg)
        msg.desired_normal_force_limit = float(normal_force_limit)
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
        lift_normal_from_nominal = vector_dot(state.get("lift_target", nominal_position), self.wall_normal_body) - nominal_normal
        press_normal_from_nominal = vector_dot(state.get("press_target", nominal_position), self.wall_normal_body) - nominal_normal
        preload_normal_from_nominal = vector_dot(state.get("preload_target", nominal_position), self.wall_normal_body) - nominal_normal
        attach_normal_from_nominal = vector_dot(state.get("attach_target", nominal_position), self.wall_normal_body) - nominal_normal
        bias_vector = state.get("compliant_joint_torque_bias", [0.0, 0.0, 0.0])
        bias_norm = vector_norm(bias_vector if isinstance(bias_vector, list) else list(bias_vector))
        contact_active_since = state.get("contact_active_since")
        contact_age = 0.0 if contact_active_since is None else max(0.0, now_sec - float(contact_active_since))
        # Snapshot the latest measured force (recomputed lazily from joint torques) so the observer can
        # see what the admittance actually sees, not only the smoothed estimate.
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
        # Snapshot the PRESS-phase delta-contact detector so the observer (and the test harness
        # that gates the fan) can follow what the controller saw tick-by-tick, without having to
        # re-aggregate joint effort/current itself.
        leg_torque_sum_now = self._leg_torque_sum_nm(leg_name)
        leg_current_sum_now = self._leg_current_sum_a(leg_name)
        press_torque_baseline_raw = state.get("press_torque_sum_baseline")
        press_current_baseline_raw = state.get("press_current_sum_baseline")
        press_torque_baseline = float(press_torque_baseline_raw) if press_torque_baseline_raw is not None else float("nan")
        press_current_baseline = float(press_current_baseline_raw) if press_current_baseline_raw is not None else float("nan")

        msg.data = [
            float(leg_index),
            float(phase_id),
            float(phase_elapsed),
            float(cmd_normal_from_nominal),
            float(lift_normal_from_nominal),
            float(press_normal_from_nominal),
            float(preload_normal_from_nominal),
            float(attach_normal_from_nominal),
            float(state.get("compliant_force_estimate", 0.0)),
            float(state.get("compliant_normal_offset", 0.0)),
            float(state.get("compliant_normal_velocity", 0.0)),
            float(measured_force),
            float(bias_norm),
            float(contact_age),
            1.0 if bool(state.get("test_experiment_active", False)) else 0.0,
            1.0 if bool(state.get("test_hold_at_lift", False)) else 0.0,
            1.0 if bool(state.get("test_lift_aligned", False)) else 0.0,
            float(leg_torque_sum_now),
            float(leg_current_sum_now),
            press_torque_baseline,
            press_current_baseline,
            float(state.get("press_torque_delta", 0.0)),
            float(state.get("press_current_delta", 0.0)),
            1.0 if bool(state.get("press_contact_confirmed", False)) else 0.0,
        ]
        publisher.publish(msg)

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
                test_force_reset_enabled = self._test_force_support_reset_enabled()

                if desired_support and self.swing_phase_start[leg_name] is None:
                    support_msg = self._support_command(leg_name, leg_index)
                    self.pub.publish(support_msg)
                    self._publish_leg_diagnostic(
                        leg_name,
                        leg_index,
                        [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                        now_sec,
                    )
                    continue

                if self.swing_phase_start[leg_name] is None and not desired_support:
                    self._start_swing(leg_name, leg_index, now_sec)

                if self.swing_phase_start[leg_name] is None:
                    support_msg = self._support_command(leg_name, leg_index)
                    self.pub.publish(support_msg)
                    self._publish_leg_diagnostic(
                        leg_name,
                        leg_index,
                        [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                        now_sec,
                    )
                    continue

                if desired_support and test_force_reset_enabled:
                    # Test-only safeguard: when upper-level commands all-support, force
                    # any lingering swing phase back to SUPPORT so the next staged test
                    # starts from a clean state.
                    self.swing_phase_start[leg_name] = None
                    support_msg = self._support_command(leg_name, leg_index)
                    self.pub.publish(support_msg)
                    self._publish_leg_diagnostic(
                        leg_name,
                        leg_index,
                        [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                        now_sec,
                    )
                    continue

                if desired_support and (attachment_ready or adhesion_ready):
                    self.swing_phase_start[leg_name] = None
                    support_msg = self._support_command(leg_name, leg_index)
                    self.pub.publish(support_msg)
                    self._publish_leg_diagnostic(
                        leg_name,
                        leg_index,
                        [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                        now_sec,
                    )
                    continue

                state = self.swing_states[leg_name]
                if state["phase"] == self.PHASE_ATTACHED_HOLD and desired_support and (attachment_ready or adhesion_ready):
                    self.swing_phase_start[leg_name] = None
                    support_msg = self._support_command(leg_name, leg_index)
                    self.pub.publish(support_msg)
                    self._publish_leg_diagnostic(
                        leg_name,
                        leg_index,
                        [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                        now_sec,
                    )
                    continue
                # Contact off: after compliant_settle_timeout_s in ATTACHED_HOLD, unconditionally return to SUPPORT
                if state["phase"] == self.PHASE_ATTACHED_HOLD and desired_support and not self.use_contact_feedback:
                    elapsed = self._phase_elapsed(state, now_sec)
                    if elapsed >= self.compliant_settle_timeout_s:
                        self.swing_phase_start[leg_name] = None
                        support_msg = self._support_command(leg_name, leg_index)
                        self.pub.publish(support_msg)
                        self._publish_leg_diagnostic(
                            leg_name,
                            leg_index,
                            [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                            now_sec,
                        )
                        continue

                
                # if self.mission_state in self.hold_position_states and self.swing_phase_start[leg_name] is not None:
                #     self.swing_phase_start[leg_name] = None
                #     support_msg = self._support_command(leg_name, leg_index)
                #     self.pub.publish(support_msg)
                #     self._publish_leg_diagnostic(
                #         leg_name,
                #         leg_index,
                #         [support_msg.center.x, support_msg.center.y, support_msg.center.z],
                #         now_sec,
                #     )
                #     continue

                cmd_position, cmd_velocity, support_leg, skirt_target, normal_force_limit = self._guided_swing_command(leg_name, leg_index, now_sec, dt)
                self._publish_leg_diagnostic(leg_name, leg_index, cmd_position, now_sec)
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