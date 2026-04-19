#!/usr/bin/env python3

import argparse
import copy
import json
import math
import os
import sys

import rosgraph
import rospy
from climbing_msgs.msg import AdhesionCommand, BodyReference, EstimatedState, LegCenterCommand, SwingLegDebug
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


def clamp(value, lower_bound, upper_bound):
    return max(lower_bound, min(upper_bound, value))


def vector_dot(lhs, rhs):
    return sum([float(lhs[index]) * float(rhs[index]) for index in range(min(len(lhs), len(rhs)))])


def vector_add(lhs, rhs):
    return [float(lhs[index]) + float(rhs[index]) for index in range(min(len(lhs), len(rhs)))]


def vector_scale(vector, scale):
    return [float(value) * float(scale) for value in vector]


def vector_norm(vector):
    return math.sqrt(sum([float(value) * float(value) for value in vector]))


def normalize_vector(vector, fallback):
    norm = vector_norm(vector)
    if norm <= 1e-9:
        return list(fallback)
    return [float(value) / norm for value in vector]


def axis_name_from_mode(mode):
    return "wall_normal" if str(mode).strip().lower() == "wall_normal" else "ground_vertical"


def quaternion_conjugate(quaternion):
    return [-float(quaternion[0]), -float(quaternion[1]), -float(quaternion[2]), float(quaternion[3])]


def quaternion_multiply(lhs, rhs):
    x1, y1, z1, w1 = [float(value) for value in lhs]
    x2, y2, z2, w2 = [float(value) for value in rhs]
    return [
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ]


def rotate_vector_by_quaternion(vector, quaternion):
    pure = [float(vector[0]), float(vector[1]), float(vector[2]), 0.0]
    rotated = quaternion_multiply(quaternion_multiply(quaternion, pure), quaternion_conjugate(quaternion))
    return rotated[:3]


def quaternion_from_pose(pose_msg):
    return [
        float(pose_msg.orientation.x),
        float(pose_msg.orientation.y),
        float(pose_msg.orientation.z),
        float(pose_msg.orientation.w),
    ]


class SwingLegStateMachineTester(object):
    def __init__(self, args):
        rospy.init_node("test_swing_leg_state_machine", anonymous=False)

        self.args = args
        self.leg_names = [str(name) for name in rospy.get_param("/swing_leg_controller/leg_names", ["lf", "rf", "rr", "lr"])]
        legs_cfg = rospy.get_param("/legs", {})
        self.leg_motor_ids = {
            str(leg_name): [int(value) for value in cfg.get("motor_ids", [])]
            for leg_name, cfg in legs_cfg.items()
        }
        if args.leg not in self.leg_names:
            raise RuntimeError("Unsupported leg %s. Valid legs: %s" % (args.leg, ", ".join(self.leg_names)))
        self.leg_name = args.leg
        self.leg_index = self.leg_names.index(self.leg_name)
        self.leg_hip_yaw_rad = {
            str(leg_name): math.radians(float(cfg.get("hip_yaw_deg", 0.0)))
            for leg_name, cfg in legs_cfg.items()
        }

        self.preload_normal_force_limit_n = float(rospy.get_param("/swing_leg_controller/preload_normal_force_limit_n", 15.0))
        self.attach_normal_force_limit_n = float(rospy.get_param("/swing_leg_controller/attach_normal_force_limit_n", 25.0))
        self.contact_hold_min_s = float(rospy.get_param("/swing_leg_controller/contact_hold_min_s", 0.04))
        self.command_reach_tolerance_m = float(rospy.get_param("/swing_leg_controller/normal_alignment_tolerance_m", 0.004))
        self.actual_reach_tolerance_m = max(self.command_reach_tolerance_m * 1.5, 0.006)
        self.preload_skirt_target = float(rospy.get_param("/swing_leg_controller/preload_skirt_compression_target", 0.85))
        self.light_contact_skirt_target = float(rospy.get_param("/swing_leg_controller/light_contact_skirt_compression_target", 0.20))
        self.force_limit_tolerance_n = max(float(args.force_limit_tolerance_n), 1e-3)
        self.max_position_offset = [float(value) for value in rospy.get_param("/swing_leg_controller/max_position_offset_m", [0.045, 0.035, 0.05])]
        self.nominal_x_m = float(rospy.get_param("/gait_controller/nominal_x", 118.75)) / 1000.0
        self.nominal_y_m = float(rospy.get_param("/gait_controller/nominal_y", 0.0)) / 1000.0
        self.link_coxa_m = float(rospy.get_param("/gait_controller/link_coxa", 44.75)) / 1000.0
        self.link_femur_m = float(rospy.get_param("/gait_controller/link_femur", 74.0)) / 1000.0
        self.link_tibia_m = float(rospy.get_param("/gait_controller/link_tibia", 150.0)) / 1000.0
        self.link_a3_m = float(rospy.get_param("/gait_controller/link_a3", 41.5)) / 1000.0
        self.base_radius_m = float(rospy.get_param("/gait_controller/base_radius", 203.06)) / 1000.0
        self.wall_normal_body = normalize_vector(
            [float(value) for value in rospy.get_param("/swing_leg_controller/wall_normal_body", rospy.get_param("/wall/normal_body", [0.0, 0.0, 1.0]))],
            [0.0, 0.0, 1.0],
        )
        self.test_axis_mode = axis_name_from_mode(args.test_axis)
        configured_test_axis = rospy.get_param("/swing_leg_controller/test_axis_body", [0.0, 0.0, 1.0])
        self.test_axis_body = list(self.wall_normal_body) if self.test_axis_mode == "wall_normal" else normalize_vector(
            [float(value) for value in configured_test_axis],
            [0.0, 0.0, 1.0],
        )
        self.current_contact_threshold_a = float(rospy.get_param("/state_estimator/current_contact_threshold_a", 0.4))
        self.torque_contact_threshold_nm = float(rospy.get_param("/state_estimator/torque_contact_threshold_nm", 0.5))
        legacy_nominal_z_mm = float(rospy.get_param("/gait_controller/nominal_z", -299.2))
        universal_joint_rigid_offset_m = abs(
            float(rospy.get_param("/gait_controller/link_d6", -13.5)) + float(rospy.get_param("/gait_controller/link_d7", -106.7))
        ) / 1000.0
        self.nominal_z_m = float(
            rospy.get_param(
                "/gait_controller/nominal_universal_joint_center_z",
                legacy_nominal_z_mm + universal_joint_rigid_offset_m * 1000.0,
            )
        ) / 1000.0
        self.nominal_foot_center = [0.0, 0.0, self.nominal_z_m]
        self.nominal_axis_scalar = vector_dot(self.nominal_foot_center, self.test_axis_body)
        self.nominal_body_center_by_leg = {
            leg_name: self._body_from_leg_local([
                self.nominal_x_m,
                self.nominal_y_m,
                self.nominal_z_m,
            ], leg_name)
            for leg_name in self.leg_names
        }
        self.max_trigger_axis_travel_m = sum([
            abs(float(self.test_axis_body[index])) * float(self.max_position_offset[index])
            for index in range(min(len(self.test_axis_body), len(self.max_position_offset)))
        ])
        self.requested_trigger_lift_travel_m = clamp(
            float(args.trigger_normal_travel_m),
            0.0,
            max(self.max_trigger_axis_travel_m, 1e-3),
        )
        self.requested_trigger_press_travel_m = clamp(
            float(args.trigger_press_normal_travel_m),
            -max(self.max_trigger_axis_travel_m, 1e-3),
            max(self.max_trigger_axis_travel_m, 1e-3),
        )

        self.output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
        os.makedirs(self.output_dir, exist_ok=True)
        self.log_path = os.path.join(
            self.output_dir,
            "swing_state_machine_%s_%s.jsonl" % (self.leg_name, rospy.Time.now().to_nsec()),
        )

        self.last_body_reference = None
        self.last_estimated_state = None
        self.last_swing_target = None
        self.last_adhesion_command = None
        self.last_mission_state = None
        self.last_mission_active = False
        self.last_joint_state = None
        self.last_swing_debug = None
        self.phase_seen = []
        self.last_printed_phase = None
        self.last_print_time = rospy.Time(0)
        self.last_missing_target_diagnostic_time = None
        self.contact_active_since = None
        self.trigger_started_at = None
        self.trigger_reference_seed = None
        self.preload_reference_axis_scalar = None
        self.first_contact_hold_time = None
        self.first_compliant_response_time = None
        self.peak_compliant_extra_axis_offset_m = 0.0
        self.peak_target_axis_from_nominal_m = 0.0
        self.override_params_active = False
        self.debug_message_seen = False
        self.seen_lift_command_reached = False
        self.seen_press_command_reached = False
        self.seen_actual_lift_reached = False
        self.seen_actual_press_reached = False
        self.seen_contact_detected = False
        self.seen_admittance_started = False
        self.seen_admittance_response = False
        self.contact_detection_modes_seen = set()

        self.body_reference_pub = None
        if self.args.trigger_swing:
            self.body_reference_pub = rospy.Publisher("/control/body_reference", BodyReference, queue_size=20)
            self._configure_test_override_params()

        rospy.Subscriber("/control/body_reference", BodyReference, self.body_reference_callback, queue_size=20)
        rospy.Subscriber("/state/estimated", EstimatedState, self.estimated_state_callback, queue_size=20)
        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self.swing_target_callback, queue_size=50)
        rospy.Subscriber("/jetson/fan_serial_bridge/adhesion_command", AdhesionCommand, self.adhesion_command_callback, queue_size=20)
        rospy.Subscriber("/control/mission_state", String, self.mission_state_callback, queue_size=20)
        rospy.Subscriber("/control/mission_active", Bool, self.mission_active_callback, queue_size=20)
        rospy.Subscriber("/jetson/dynamixel_bridge/joint_state", JointState, self.joint_state_callback, queue_size=20)
        rospy.Subscriber("/control/swing_leg_debug", SwingLegDebug, self.swing_debug_callback, queue_size=50)

    def body_reference_callback(self, msg):
        self.last_body_reference = msg

    def estimated_state_callback(self, msg):
        self.last_estimated_state = msg

    def swing_target_callback(self, msg):
        if msg.leg_name == self.leg_name:
            self.last_swing_target = msg

    def adhesion_command_callback(self, msg):
        if int(msg.leg_index) == self.leg_index:
            self.last_adhesion_command = msg

    def mission_state_callback(self, msg):
        self.last_mission_state = str(msg.data)

    def mission_active_callback(self, msg):
        self.last_mission_active = bool(msg.data)

    def joint_state_callback(self, msg):
        self.last_joint_state = msg

    def swing_debug_callback(self, msg):
        if msg.leg_name == self.leg_name:
            self.last_swing_debug = msg
            self.debug_message_seen = True

    def _leg_value(self, values, default=None):
        if values is None or self.leg_index >= len(values):
            return default
        return values[self.leg_index]

    def _support_mask_for_leg(self, support):
        mask = [True] * len(self.leg_names)
        mask[self.leg_index] = bool(support)
        return mask

    def _seed_trigger_reference(self):
        if self.last_estimated_state is not None:
            pose = copy.deepcopy(self.last_estimated_state.pose)
        elif self.last_body_reference is not None:
            pose = copy.deepcopy(self.last_body_reference.pose)
        else:
            pose = Pose()
            pose.orientation.w = 1.0

        if self.last_body_reference is not None:
            gait_mode = int(self.last_body_reference.gait_mode)
        else:
            gait_mode = 0

        twist = Twist()
        return {
            "pose": pose,
            "gait_mode": gait_mode,
            "orientation": quaternion_from_pose(pose),
        }

    def _make_trigger_body_reference(self, support_leg):
        msg = BodyReference()
        msg.header.stamp = rospy.Time.now()
        if self.trigger_reference_seed is None:
            self.trigger_reference_seed = self._seed_trigger_reference()

        msg.pose = copy.deepcopy(self.trigger_reference_seed["pose"])
        msg.twist = Twist()
        msg.gait_mode = int(self.trigger_reference_seed["gait_mode"])
        msg.support_mask = self._support_mask_for_leg(support_leg)
        return msg

    def _configure_test_override_params(self):
        rospy.set_param("/swing_leg_controller/test_trigger_leg_name", str(self.leg_name))
        rospy.set_param("/swing_leg_controller/test_trigger_normal_travel_m", float(self.requested_trigger_lift_travel_m))
        rospy.set_param("/swing_leg_controller/test_trigger_press_normal_travel_m", float(self.requested_trigger_press_travel_m))
        rospy.set_param("/swing_leg_controller/test_axis_mode", str(self.test_axis_mode))
        rospy.set_param("/swing_leg_controller/test_axis_body", [float(value) for value in self.test_axis_body])
        self.override_params_active = True

    def _clear_test_override_params(self):
        if not self.override_params_active:
            return

        for param_name in [
            "/swing_leg_controller/test_trigger_leg_name",
            "/swing_leg_controller/test_trigger_normal_travel_m",
            "/swing_leg_controller/test_trigger_press_normal_travel_m",
            "/swing_leg_controller/test_axis_mode",
            "/swing_leg_controller/test_axis_body",
        ]:
            if rospy.has_param(param_name):
                rospy.delete_param(param_name)
        self.override_params_active = False

    def _publish_trigger_reference(self, now):
        if self.body_reference_pub is None:
            return
        if self.trigger_started_at is None:
            self.trigger_started_at = now

        elapsed = (now - self.trigger_started_at).to_sec()
        support_leg = elapsed >= self.args.swing_duration_s
        msg = self._make_trigger_body_reference(support_leg)
        self.body_reference_pub.publish(msg)

    def _contact_hold_elapsed(self, now):
        if self.last_estimated_state is None:
            self.contact_active_since = None
            return 0.0

        measured_contact = bool(self._leg_value(self.last_estimated_state.measured_contact_mask, False))
        wall_touch = bool(self._leg_value(self.last_estimated_state.wall_touch_mask, False))
        contact_active = measured_contact or wall_touch
        if contact_active:
            if self.contact_active_since is None:
                self.contact_active_since = now
            return max((now - self.contact_active_since).to_sec(), 0.0)
        self.contact_active_since = None
        return 0.0

    def _topic_publishers(self, topic_name):
        try:
            master = rosgraph.Master(rospy.get_name())
            publishers, _, _ = master.getSystemState()
        except Exception:
            return []

        for published_topic, node_names in publishers:
            if published_topic == topic_name:
                return sorted(node_names)
        return []

    def _maybe_print_missing_target_diagnostic(self, now):
        if self.last_swing_target is not None:
            return

        if now.to_sec() < max(self.args.startup_grace_s, 0.0):
            return

        if self.last_missing_target_diagnostic_time is not None:
            elapsed = (now - self.last_missing_target_diagnostic_time).to_sec()
            if elapsed < max(self.args.diagnostic_repeat_s, 0.0):
                return

        swing_publishers = self._topic_publishers("/control/swing_leg_target")
        debug_publishers = self._topic_publishers("/control/swing_leg_debug")
        state_publishers = self._topic_publishers("/state/estimated")
        body_reference_publishers = self._topic_publishers("/control/body_reference")
        print(
            "[{stamp:8.3f}] diagnostic: no /control/swing_leg_target message for leg={leg}. "
            "publishers swing_target={swing} swing_debug={debug} state={state} body_reference={body}. "
            "received state={state_seen} body_reference={body_seen} debug_seen={debug_seen} trigger_swing={trigger}.".format(
                stamp=now.to_sec(),
                leg=self.leg_name,
                swing=swing_publishers if swing_publishers else ["none"],
                debug=debug_publishers if debug_publishers else ["none"],
                state=state_publishers if state_publishers else ["none"],
                body=body_reference_publishers if body_reference_publishers else ["none"],
                state_seen=str(self.last_estimated_state is not None),
                body_seen=str(self.last_body_reference is not None),
                debug_seen=str(self.last_swing_debug is not None),
                trigger=str(bool(self.args.trigger_swing)),
            ),
            file=sys.stderr,
        )
        sys.stderr.flush()
        self.last_missing_target_diagnostic_time = now

    def _phase_from_outputs(self):
        if self.last_swing_debug is not None:
            return str(self.last_swing_debug.phase)
        if self.last_swing_target is None:
            return "NO_TARGET"

        support_leg = bool(self.last_swing_target.support_leg)
        normal_force_limit = float(self.last_swing_target.desired_normal_force_limit)
        skirt_target = float(self.last_swing_target.skirt_compression_target)
        active_axis = self._active_axis()
        nominal_axis_scalar = self._nominal_scalar_for_axis(active_axis)
        target_axis_from_nominal = vector_dot(
            [
                float(self.last_swing_target.center.x),
                float(self.last_swing_target.center.y),
                float(self.last_swing_target.center.z),
            ],
            active_axis,
        ) - nominal_axis_scalar

        if support_leg:
            if abs(normal_force_limit - self.attach_normal_force_limit_n) <= self.force_limit_tolerance_n and skirt_target >= self.preload_skirt_target:
                return "ATTACHED_HOLD"
            return "SUPPORT"
        if self.args.trigger_swing and abs(normal_force_limit) <= self.force_limit_tolerance_n:
            staged_midpoint = 0.5 * (self.requested_trigger_lift_travel_m + self.requested_trigger_press_travel_m)
            if target_axis_from_nominal >= staged_midpoint:
                return "TEST_LIFT_CLEARANCE"
            return "TEST_PRESS_CONTACT"
        if abs(normal_force_limit - self.attach_normal_force_limit_n) <= self.force_limit_tolerance_n:
            return "COMPLIANT_SETTLE"
        if abs(normal_force_limit - self.preload_normal_force_limit_n) <= self.force_limit_tolerance_n:
            return "PRELOAD_COMPRESS"
        if skirt_target >= self.light_contact_skirt_target:
            return "DETACH_OR_TANGENTIAL"
        return "SWING_FREE"

    def _active_axis(self):
        if self.last_swing_debug is not None:
            return normalize_vector(
                [
                    float(self.last_swing_debug.active_axis_body.x),
                    float(self.last_swing_debug.active_axis_body.y),
                    float(self.last_swing_debug.active_axis_body.z),
                ],
                self.test_axis_body,
            )
        return list(self.test_axis_body)

    def _nominal_scalar_for_axis(self, axis):
        return vector_dot(self.nominal_foot_center, axis)

    def _target_axis_metrics(self, phase, swing, contact_hold_satisfied, now):
        if swing is None:
            return {}

        active_axis = self._active_axis()
        nominal_axis_scalar = self._nominal_scalar_for_axis(active_axis)
        target_center = [float(swing.center.x), float(swing.center.y), float(swing.center.z)]
        target_velocity = [float(swing.center_velocity.x), float(swing.center_velocity.y), float(swing.center_velocity.z)]
        target_axis_scalar = vector_dot(target_center, active_axis)
        target_axis_velocity = vector_dot(target_velocity, active_axis)
        target_axis_from_nominal = target_axis_scalar - nominal_axis_scalar
        self.peak_target_axis_from_nominal_m = max(self.peak_target_axis_from_nominal_m, target_axis_from_nominal)

        if phase in ["PRELOAD_COMPRESS", "COMPLIANT_SETTLE"] and not contact_hold_satisfied:
            self.preload_reference_axis_scalar = target_axis_scalar

        if contact_hold_satisfied and self.first_contact_hold_time is None:
            self.first_contact_hold_time = float(now.to_sec())

        compliant_extra_axis_offset = 0.0
        compliant_response_detected = False
        compliant_response_latency_s = None
        preload_reference_axis_from_nominal = None
        if self.preload_reference_axis_scalar is not None:
            preload_reference_axis_from_nominal = self.preload_reference_axis_scalar - nominal_axis_scalar
            compliant_extra_axis_offset = target_axis_scalar - self.preload_reference_axis_scalar
            if compliant_extra_axis_offset > 0.0:
                self.peak_compliant_extra_axis_offset_m = max(self.peak_compliant_extra_axis_offset_m, compliant_extra_axis_offset)
            if contact_hold_satisfied and compliant_extra_axis_offset > 1e-4:
                compliant_response_detected = True
                if self.first_compliant_response_time is None:
                    self.first_compliant_response_time = float(now.to_sec())

        if self.first_contact_hold_time is not None and self.first_compliant_response_time is not None:
            compliant_response_latency_s = self.first_compliant_response_time - self.first_contact_hold_time

        return {
            "test_axis_name": self.test_axis_mode,
            "target_axis_position_m": round(target_axis_scalar, 5),
            "target_axis_velocity_mps": round(target_axis_velocity, 5),
            "target_axis_from_nominal_m": round(target_axis_from_nominal, 5),
            "preload_reference_axis_from_nominal_m": None if preload_reference_axis_from_nominal is None else round(preload_reference_axis_from_nominal, 5),
            "compliant_extra_axis_offset_m": round(compliant_extra_axis_offset, 5),
            "compliant_response_detected": bool(compliant_response_detected),
            "compliant_response_latency_s": None if compliant_response_latency_s is None else round(compliant_response_latency_s, 4),
            "peak_target_axis_from_nominal_m": round(self.peak_target_axis_from_nominal_m, 5),
            "peak_compliant_extra_axis_offset_m": round(self.peak_compliant_extra_axis_offset_m, 5),
            "target_normal_position_m": round(target_axis_scalar, 5),
            "target_normal_velocity_mps": round(target_axis_velocity, 5),
            "target_normal_from_nominal_m": round(target_axis_from_nominal, 5),
            "preload_reference_normal_from_nominal_m": None if preload_reference_axis_from_nominal is None else round(preload_reference_axis_from_nominal, 5),
            "compliant_extra_normal_offset_m": round(compliant_extra_axis_offset, 5),
            "peak_target_normal_from_nominal_m": round(self.peak_target_axis_from_nominal_m, 5),
            "peak_compliant_extra_normal_offset_m": round(self.peak_compliant_extra_axis_offset_m, 5),
        }

    def _leg_joint_feedback(self):
        if self.last_joint_state is None:
            return None

        joint_name_to_index = {str(name): index for index, name in enumerate(self.last_joint_state.name)}
        motor_ids = self.leg_motor_ids.get(self.leg_name, [])
        if not motor_ids:
            return None

        positions = []
        velocities = []
        efforts = []
        reported_motor_ids = []
        for motor_id in motor_ids:
            joint_index = joint_name_to_index.get(str(int(motor_id)))
            if joint_index is None:
                continue
            reported_motor_ids.append(int(motor_id))
            positions.append(float(self.last_joint_state.position[joint_index]) if joint_index < len(self.last_joint_state.position) else 0.0)
            velocities.append(float(self.last_joint_state.velocity[joint_index]) if joint_index < len(self.last_joint_state.velocity) else 0.0)
            efforts.append(float(self.last_joint_state.effort[joint_index]) if joint_index < len(self.last_joint_state.effort) else 0.0)

        if not reported_motor_ids:
            return None

        return {
            "motor_ids": reported_motor_ids,
            "position_rad": [round(value, 5) for value in positions],
            "velocity_rad_s": [round(value, 5) for value in velocities],
            "effort": [round(value, 5) for value in efforts],
        }

    def _rotate_z(self, vector, yaw_rad):
        cos_yaw = math.cos(yaw_rad)
        sin_yaw = math.sin(yaw_rad)
        return [
            cos_yaw * float(vector[0]) - sin_yaw * float(vector[1]),
            sin_yaw * float(vector[0]) + cos_yaw * float(vector[1]),
            float(vector[2]),
        ]

    def _forward_kinematics_leg_local(self, joint_vector_rad):
        q1 = float(joint_vector_rad[0])
        q2 = float(joint_vector_rad[1])
        q3 = float(joint_vector_rad[2])
        alpha = q2 - math.radians(90.0)
        radial_prime = self.link_tibia_m * math.cos(alpha) + self.link_a3_m * math.cos(alpha + q3)
        z_value = self.link_tibia_m * math.sin(alpha) + self.link_a3_m * math.sin(alpha + q3)
        radial_total = self.link_femur_m + radial_prime
        return [
            self.link_coxa_m + radial_total * math.cos(q1),
            radial_total * math.sin(q1),
            z_value,
        ]

    def _body_from_leg_local(self, local_center, leg_name):
        hip_yaw_rad = float(self.leg_hip_yaw_rad.get(leg_name, 0.0))
        offset = self._rotate_z([self.base_radius_m, 0.0, 0.0], hip_yaw_rad)
        rotated_local = self._rotate_z(local_center, hip_yaw_rad)
        return vector_add(offset, rotated_local)

    def _actual_center_metrics(self, joint_feedback, data):
        joint_vector_rad = joint_feedback.get("position_rad") or []
        if len(joint_vector_rad) != 3:
            return {}

        active_axis = self._active_axis()
        nominal_axis_scalar = self._nominal_scalar_for_axis(active_axis)
        local_center = self._forward_kinematics_leg_local(joint_vector_rad)
        body_center = self._body_from_leg_local(local_center, self.leg_name)
        nominal_body_center = self.nominal_body_center_by_leg.get(self.leg_name)
        if nominal_body_center is None:
            return {}

        body_delta = [
            float(body_center[index]) - float(nominal_body_center[index])
            for index in range(3)
        ]
        controller_frame_center = [body_delta[0], body_delta[1], body_center[2]]
        actual_axis_scalar = vector_dot(controller_frame_center, active_axis)
        metrics = {
            "actual_center_estimate": {
                "x": round(body_delta[0], 5),
                "y": round(body_delta[1], 5),
                "z": round(body_center[2], 5),
            },
            "actual_center_local_estimate": {
                "x": round(local_center[0], 5),
                "y": round(local_center[1], 5),
                "z": round(local_center[2], 5),
            },
            "actual_axis_position_m": round(actual_axis_scalar, 5),
            "actual_axis_from_nominal_m": round(actual_axis_scalar - nominal_axis_scalar, 5),
            "actual_normal_position_m": round(actual_axis_scalar, 5),
            "actual_normal_from_nominal_m": round(actual_axis_scalar - nominal_axis_scalar, 5),
        }

        target_axis_position = data.get("target_axis_position_m", data.get("target_normal_position_m"))
        if target_axis_position is not None:
            tracking_error = actual_axis_scalar - float(target_axis_position)
            metrics["actual_axis_tracking_error_m"] = round(tracking_error, 5)
            metrics["actual_normal_tracking_error_m"] = round(tracking_error, 5)
        return metrics

    def _leg_joint_index_map(self):
        if self.last_joint_state is None:
            return {}
        return {str(name): index for index, name in enumerate(self.last_joint_state.name)}

    def _aggregate_leg_signal(self, values):
        index_map = self._leg_joint_index_map()
        motor_ids = self.leg_motor_ids.get(self.leg_name, [])
        aggregated = []
        for motor_id in motor_ids:
            joint_index = index_map.get(str(int(motor_id)))
            if joint_index is None or joint_index >= len(values):
                continue
            aggregated.append(float(values[joint_index]))
        return aggregated

    def _contact_detected_by(self, data):
        reasons = []
        est = self.last_estimated_state
        if est is not None:
            current_values = self._aggregate_leg_signal(list(est.joint_currents))
            torque_values = self._aggregate_leg_signal(list(est.joint_torques_est))
            current_sum = sum([abs(value) for value in current_values])
            torque_sum = sum([abs(value) for value in torque_values])
            if current_values:
                current_threshold = self.current_contact_threshold_a * len(current_values)
                data["leg_current_sum_a"] = round(current_sum, 4)
                data["leg_current_contact_threshold_a"] = round(current_threshold, 4)
                data["leg_current_contact_triggered"] = bool(current_sum >= current_threshold)
                if current_sum >= current_threshold:
                    reasons.append("current")
            if torque_values:
                torque_threshold = self.torque_contact_threshold_nm * len(torque_values)
                data["leg_joint_torque_sum_nm"] = round(torque_sum, 4)
                data["leg_joint_torque_contact_threshold_nm"] = round(torque_threshold, 4)
                data["leg_joint_torque_contact_triggered"] = bool(torque_sum >= torque_threshold)
                if torque_sum >= torque_threshold:
                    reasons.append("torque")
        if data.get("measured_contact"):
            reasons.append("measured_contact_mask")
        if data.get("wall_touch"):
            reasons.append("wall_touch_mask")
        if data.get("early_contact"):
            reasons.append("early_contact_mask")
        if data.get("attachment_ready"):
            reasons.append("attachment_ready")
        unique_reasons = []
        for reason in reasons:
            if reason not in unique_reasons:
                unique_reasons.append(reason)
        if unique_reasons:
            self.contact_detection_modes_seen.update(unique_reasons)
        return "|".join(unique_reasons) if unique_reasons else "none"

    def _update_summary_flags(self, data):
        self.seen_lift_command_reached = self.seen_lift_command_reached or bool(data.get("lift_command_reached"))
        self.seen_press_command_reached = self.seen_press_command_reached or bool(data.get("press_command_reached"))
        self.seen_actual_lift_reached = self.seen_actual_lift_reached or bool(data.get("actual_lift_reached"))
        self.seen_actual_press_reached = self.seen_actual_press_reached or bool(data.get("actual_press_reached"))
        self.seen_contact_detected = self.seen_contact_detected or bool(data.get("contact_detected"))
        self.seen_admittance_started = self.seen_admittance_started or bool(data.get("admittance_started"))
        self.seen_admittance_response = self.seen_admittance_response or bool(data.get("admittance_response_detected"))

    def _classify_failure_stage(self):
        if self.last_swing_target is None:
            return "no_target"
        if not self.seen_lift_command_reached:
            return "no_lift_command"
        if not self.seen_actual_lift_reached:
            return "no_lift_motion"
        if not self.seen_press_command_reached:
            return "no_press_command"
        if not self.seen_actual_press_reached:
            return "no_press_motion"
        if not self.seen_contact_detected:
            return "motion_but_no_contact"
        if not self.seen_admittance_started:
            return "contact_but_no_admittance_start"
        if not self.seen_admittance_response:
            return "contact_but_no_admittance_response"
        return "success"

    def _snapshot(self, now):
        phase = self._phase_from_outputs()
        est = self.last_estimated_state
        swing = self.last_swing_target
        adhesion = self.last_adhesion_command
        swing_debug = self.last_swing_debug
        contact_hold_elapsed = self._contact_hold_elapsed(now)

        data = {
            "stamp": now.to_sec(),
            "leg_name": self.leg_name,
            "mission_state": self.last_mission_state,
            "mission_active": bool(self.last_mission_active),
            "debug_message_seen": bool(self.debug_message_seen),
            "phase": phase,
            "inferred_phase": phase,
            "contact_hold_elapsed_s": round(contact_hold_elapsed, 4),
            "contact_hold_satisfied": contact_hold_elapsed >= self.contact_hold_min_s,
            "test_axis_name": self.test_axis_mode,
            "trigger_lift_travel_request_m": round(self.requested_trigger_lift_travel_m, 5),
            "trigger_press_travel_request_m": round(self.requested_trigger_press_travel_m, 5),
            "trigger_normal_travel_request_m": round(self.requested_trigger_lift_travel_m, 5),
            "trigger_press_normal_travel_request_m": round(self.requested_trigger_press_travel_m, 5),
        }

        if self.last_body_reference is not None and self.leg_index < len(self.last_body_reference.support_mask):
            data["body_reference_support"] = bool(self.last_body_reference.support_mask[self.leg_index])

        if swing_debug is not None:
            active_axis = [
                float(swing_debug.active_axis_body.x),
                float(swing_debug.active_axis_body.y),
                float(swing_debug.active_axis_body.z),
            ]
            data.update(
                {
                    "phase": str(swing_debug.phase),
                    "debug_phase": str(swing_debug.phase),
                    "inferred_phase": str(swing_debug.phase),
                    "debug_transition_reason": str(swing_debug.transition_reason),
                    "debug_test_mode_active": bool(swing_debug.test_mode_active),
                    "debug_desired_support": bool(swing_debug.desired_support),
                    "debug_estimated_support": bool(swing_debug.estimated_support),
                    "debug_support_leg_command": bool(swing_debug.support_leg_command),
                    "debug_measured_contact": bool(swing_debug.measured_contact),
                    "debug_wall_touch": bool(swing_debug.wall_touch),
                    "debug_early_contact": bool(swing_debug.early_contact),
                    "debug_attachment_ready": bool(swing_debug.attachment_ready),
                    "debug_adhesion_ready": bool(swing_debug.adhesion_ready),
                    "debug_contact_hold_satisfied": bool(swing_debug.contact_hold_satisfied),
                    "debug_reached_lift_target": bool(swing_debug.reached_lift_target),
                    "debug_reached_press_target": bool(swing_debug.reached_press_target),
                    "debug_phase_timed_out": bool(swing_debug.phase_timed_out),
                    "debug_phase_elapsed_s": round(float(swing_debug.phase_elapsed_s), 4),
                    "debug_measured_axis_force_n": round(float(swing_debug.measured_axis_force_n), 4),
                    "debug_compliant_force_estimate_n": round(float(swing_debug.compliant_force_estimate_n), 4),
                    "debug_compliant_axis_offset_m": round(float(swing_debug.compliant_axis_offset_m), 5),
                    "debug_compliant_axis_velocity_mps": round(float(swing_debug.compliant_axis_velocity_mps), 5),
                    "debug_command_axis_position_m": round(float(swing_debug.command_axis_position_m), 5),
                    "debug_command_axis_velocity_mps": round(float(swing_debug.command_axis_velocity_mps), 5),
                    "debug_lift_axis_from_nominal_m": round(float(swing_debug.lift_axis_from_nominal_m), 5),
                    "debug_press_axis_from_nominal_m": round(float(swing_debug.press_axis_from_nominal_m), 5),
                    "debug_preload_axis_from_nominal_m": round(float(swing_debug.preload_axis_from_nominal_m), 5),
                    "debug_attach_axis_from_nominal_m": round(float(swing_debug.attach_axis_from_nominal_m), 5),
                    "debug_active_axis_body": [round(value, 5) for value in active_axis],
                }
            )

        if swing is not None:
            data.update(
                {
                    "target_support_leg": bool(swing.support_leg),
                    "target_center": {
                        "x": round(float(swing.center.x), 5),
                        "y": round(float(swing.center.y), 5),
                        "z": round(float(swing.center.z), 5),
                    },
                    "target_center_velocity": {
                        "x": round(float(swing.center_velocity.x), 5),
                        "y": round(float(swing.center_velocity.y), 5),
                        "z": round(float(swing.center_velocity.z), 5),
                    },
                    "target_skirt_compression": round(float(swing.skirt_compression_target), 4),
                    "target_normal_force_limit_n": round(float(swing.desired_normal_force_limit), 4),
                }
            )
            data.update(self._target_axis_metrics(phase, swing, data["contact_hold_satisfied"], now))

        if est is not None:
            data.update(
                {
                    "measured_contact": bool(self._leg_value(est.measured_contact_mask, False)),
                    "wall_touch": bool(self._leg_value(est.wall_touch_mask, False)),
                    "ground_touch": bool(self._leg_value(est.wall_touch_mask, False)) if self.test_axis_mode == "ground_vertical" else bool(self._leg_value(est.wall_touch_mask, False)),
                    "early_contact": bool(self._leg_value(est.early_contact_mask, False)),
                    "attachment_ready": bool(self._leg_value(est.attachment_ready_mask, False)),
                    "adhesion_ready": bool(self._leg_value(est.adhesion_mask, False)),
                    "support_mask": bool(self._leg_value(est.support_mask, False)),
                    "contact_confidence": round(float(self._leg_value(est.contact_confidence, 0.0)), 4),
                    "seal_confidence": round(float(self._leg_value(est.seal_confidence, 0.0)), 4),
                    "fan_current_a": round(float(self._leg_value(est.fan_current, 0.0)), 4),
                    "skirt_compression_estimate_m": round(float(self._leg_value(est.skirt_compression_estimate, 0.0)), 5),
                    "leg_torque_sum_nm": round(float(self._leg_value(est.leg_torque_sum, 0.0)), 4),
                    "leg_torque_contact_confidence": round(float(self._leg_value(est.leg_torque_contact_confidence, 0.0)), 4),
                    "normal_force_limit_n": round(float(self._leg_value(est.normal_force_limit, 0.0)), 4),
                    "slip_risk": round(float(self._leg_value(est.slip_risk, 0.0)), 4),
                }
            )

        if adhesion is not None:
            data.update(
                {
                    "adhesion_command_mode": int(adhesion.mode),
                    "adhesion_command_rpm": round(float(adhesion.target_rpm), 2),
                    "adhesion_command_normal_force_limit_n": round(float(adhesion.normal_force_limit), 4),
                    "adhesion_command_required_adhesion_force_n": round(float(adhesion.required_adhesion_force), 4),
                }
            )

        joint_feedback = self._leg_joint_feedback()
        if joint_feedback is not None:
            data["joint_feedback"] = joint_feedback
            data.update(self._actual_center_metrics(joint_feedback, data))

        lift_command_target = data.get("debug_lift_axis_from_nominal_m", self.requested_trigger_lift_travel_m)
        press_command_target = data.get("debug_press_axis_from_nominal_m", self.requested_trigger_press_travel_m)
        command_axis_from_nominal = data.get("target_axis_from_nominal_m", data.get("target_normal_from_nominal_m"))
        actual_axis_from_nominal = data.get("actual_axis_from_nominal_m", data.get("actual_normal_from_nominal_m"))

        lift_command_reached = command_axis_from_nominal is not None and abs(float(command_axis_from_nominal) - float(lift_command_target)) <= self.command_reach_tolerance_m
        press_command_reached = command_axis_from_nominal is not None and abs(float(command_axis_from_nominal) - float(press_command_target)) <= self.command_reach_tolerance_m
        actual_lift_reached = actual_axis_from_nominal is not None and abs(float(actual_axis_from_nominal) - float(lift_command_target)) <= self.actual_reach_tolerance_m
        actual_press_reached = actual_axis_from_nominal is not None and abs(float(actual_axis_from_nominal) - float(press_command_target)) <= self.actual_reach_tolerance_m
        contact_detected = bool(data.get("measured_contact") or data.get("wall_touch") or data.get("early_contact") or data.get("attachment_ready"))
        admittance_started = phase == "COMPLIANT_SETTLE" and bool(data.get("contact_hold_satisfied"))
        admittance_response_detected = bool(data.get("debug_compliant_axis_offset_m", data.get("compliant_extra_axis_offset_m", 0.0)) > 1e-4)

        data["lift_command_reached"] = bool(lift_command_reached or data.get("debug_reached_lift_target", False))
        data["press_command_reached"] = bool(press_command_reached or data.get("debug_reached_press_target", False))
        data["actual_lift_reached"] = bool(actual_lift_reached)
        data["actual_press_reached"] = bool(actual_press_reached)
        data["contact_detected"] = bool(contact_detected)
        data["contact_detected_by"] = self._contact_detected_by(data)
        data["admittance_started"] = bool(admittance_started)
        data["admittance_response_detected"] = bool(admittance_response_detected or data.get("compliant_response_detected", False))
        data["admittance_response_latency_s"] = data.get("compliant_response_latency_s")
        data["admittance_peak_offset_m"] = data.get("peak_compliant_extra_axis_offset_m", data.get("peak_compliant_extra_normal_offset_m", 0.0))

        self._update_summary_flags(data)
        data["failure_stage"] = self._classify_failure_stage()

        return data

    def _write_log(self, data):
        with open(self.log_path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(data, ensure_ascii=False) + "\n")

    def _print_snapshot(self, data):
        print(
            "[{stamp:8.3f}] phase={phase:18s} fail={failure:28s} lift(cmd/act)={lift_cmd}/{lift_act} "
            "press(cmd/act)={press_cmd}/{press_act} contact={contact} by={contact_by:24s} "
            "adm={adm}/{adm_resp} hold={hold:5.2f}s axis={axis:+.3f} actual={actual_axis:+.3f} "
            "extra={extra:+.3f} force={force_est:+.2f}N torque={torque:6.2f}Nm".format(
                stamp=data["stamp"],
                phase=data.get("phase", data.get("inferred_phase", "UNKNOWN")),
                failure=data.get("failure_stage", "unknown"),
                lift_cmd=str(data.get("lift_command_reached", False)),
                lift_act=str(data.get("actual_lift_reached", False)),
                press_cmd=str(data.get("press_command_reached", False)),
                press_act=str(data.get("actual_press_reached", False)),
                contact=str(data.get("contact_detected", False)),
                contact_by=data.get("contact_detected_by", "none"),
                adm=str(data.get("admittance_started", False)),
                adm_resp=str(data.get("admittance_response_detected", False)),
                hold=data.get("contact_hold_elapsed_s", 0.0),
                axis=data.get("target_axis_from_nominal_m", data.get("target_normal_from_nominal_m", 0.0)),
                actual_axis=data.get("actual_axis_from_nominal_m", data.get("actual_normal_from_nominal_m", 0.0)),
                extra=data.get("debug_compliant_axis_offset_m", data.get("compliant_extra_axis_offset_m", 0.0)),
                force_est=data.get("debug_measured_axis_force_n", 0.0),
                torque=data.get("leg_torque_sum_nm", 0.0),
            )
        )
        sys.stdout.flush()

    def _print_summary(self):
        print("\nSummary:")
        print("  log_path=%s" % self.log_path)
        print("  debug_message_seen=%s" % str(self.debug_message_seen))
        print("  test_axis_name=%s" % self.test_axis_mode)
        print("  requested_trigger_lift_travel_m=%.4f" % self.requested_trigger_lift_travel_m)
        print("  requested_trigger_press_travel_m=%.4f" % self.requested_trigger_press_travel_m)
        print("  peak_target_axis_from_nominal_m=%.4f" % self.peak_target_axis_from_nominal_m)
        print("  peak_compliant_extra_axis_offset_m=%.4f" % self.peak_compliant_extra_axis_offset_m)
        if self.first_contact_hold_time is not None:
            print("  first_contact_hold_time_s=%.4f" % self.first_contact_hold_time)
        else:
            print("  first_contact_hold_time_s=not_reached")
        if self.first_compliant_response_time is not None and self.first_contact_hold_time is not None:
            print("  compliant_response_latency_s=%.4f" % (self.first_compliant_response_time - self.first_contact_hold_time))
        else:
            print("  compliant_response_latency_s=not_detected")
        print("  contact_detected_by=%s" % ("|".join(sorted(self.contact_detection_modes_seen)) if self.contact_detection_modes_seen else "none"))
        print("  final_failure_stage=%s" % self._classify_failure_stage())

    def spin(self):
        rate = rospy.Rate(max(self.args.rate_hz, 1.0))
        end_time = rospy.Time.now() + rospy.Duration.from_sec(self.args.duration_s) if self.args.duration_s > 0.0 else None

        try:
            while not rospy.is_shutdown():
                now = rospy.Time.now()
                if end_time is not None and now >= end_time:
                    break

                self._publish_trigger_reference(now)
                data = self._snapshot(now)
                self._write_log(data)
                self._maybe_print_missing_target_diagnostic(now)

                phase = data.get("phase", data.get("inferred_phase"))
                should_print = phase != self.last_printed_phase
                if (now - self.last_print_time).to_sec() >= self.args.print_period_s:
                    should_print = True
                if should_print:
                    self._print_snapshot(data)
                    self.last_printed_phase = phase
                    self.last_print_time = now
                    if phase not in self.phase_seen:
                        self.phase_seen.append(phase)

                rate.sleep()
        finally:
            self._clear_test_override_params()

        print("\nLog saved to %s" % self.log_path)
        print("Seen phases: %s" % ", ".join(self.phase_seen if self.phase_seen else ["none"]))
        self._print_summary()


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Observe and optionally trigger a single-leg swing state machine run while logging key tuning values."
    )
    parser.add_argument("--leg", default="rr", help="Leg to observe: lf, rf, rr, lr. Default: rr")
    parser.add_argument("--duration-s", type=float, default=20.0, help="Monitoring duration in seconds. Use <=0 to run until Ctrl+C.")
    parser.add_argument("--rate-hz", type=float, default=30.0, help="Loop rate for logging and optional trigger publishing.")
    parser.add_argument("--print-period-s", type=float, default=0.5, help="Minimum print interval while monitoring.")
    parser.add_argument("--output-dir", default="~/climbing_ws/test_logs", help="Directory for JSONL logs.")
    parser.add_argument(
        "--trigger-swing",
        action="store_true",
        help="Publish /control/body_reference to force the selected leg through the staged single-leg test. Use only when body_planner is disabled or not publishing.",
    )
    parser.add_argument(
        "--swing-duration-s",
        type=float,
        default=2.0,
        help="When --trigger-swing is enabled, keep the selected leg in swing for this long before restoring support.",
    )
    parser.add_argument(
        "--trigger-normal-travel-m",
        type=float,
        default=0.035,
        help="Requested lift travel along the selected test axis during the staged trigger test.",
    )
    parser.add_argument(
        "--trigger-press-normal-travel-m",
        type=float,
        default=-0.04,
        help="Requested press target relative to nominal along the selected test axis after the lift phase.",
    )
    parser.add_argument(
        "--test-axis",
        default="ground_vertical",
        choices=["ground_vertical", "wall_normal"],
        help="Axis used by the staged test. Default: ground_vertical.",
    )
    parser.add_argument(
        "--force-limit-tolerance-n",
        type=float,
        default=0.5,
        help="Tolerance used when inferring phases from desired_normal_force_limit.",
    )
    parser.add_argument(
        "--startup-grace-s",
        type=float,
        default=3.0,
        help="Wait this long before warning that /control/swing_leg_target has no messages.",
    )
    parser.add_argument(
        "--diagnostic-repeat-s",
        type=float,
        default=5.0,
        help="Minimum interval between repeated missing-target diagnostics.",
    )
    return parser


if __name__ == "__main__":
    arguments = build_arg_parser().parse_args()
    try:
        SwingLegStateMachineTester(arguments).spin()
    except rospy.ROSInterruptException:
        pass
    except Exception as exc:
        sys.stderr.write("test_swing_leg_state_machine.py failed: %s\n" % exc)
        sys.exit(1)