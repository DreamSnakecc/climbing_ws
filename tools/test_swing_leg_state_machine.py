#!/usr/bin/env python3

import argparse
import copy
import json
import math
import os
import sys

import rosgraph
import rospy
from climbing_msgs.msg import AdhesionCommand, BodyReference, EstimatedState, LegCenterCommand
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String


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
        self.preload_skirt_target = float(rospy.get_param("/swing_leg_controller/preload_skirt_compression_target", 0.85))
        self.light_contact_skirt_target = float(rospy.get_param("/swing_leg_controller/light_contact_skirt_compression_target", 0.20))
        self.force_limit_tolerance_n = max(float(args.force_limit_tolerance_n), 1e-3)
        self.body_position_gain = [float(value) for value in rospy.get_param("/swing_leg_controller/body_position_gain", [0.70, 0.70, 0.35])]
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
        self.nominal_normal_scalar = vector_dot(self.nominal_foot_center, self.wall_normal_body)
        self.nominal_body_center_by_leg = {
            leg_name: self._body_from_leg_local([
                self.nominal_x_m,
                self.nominal_y_m,
                self.nominal_z_m,
            ], leg_name)
            for leg_name in self.leg_names
        }
        self.max_trigger_normal_travel_m = sum([
            abs(float(self.wall_normal_body[index])) * float(self.max_position_offset[index])
            for index in range(min(len(self.wall_normal_body), len(self.max_position_offset)))
        ])
        self.requested_trigger_normal_travel_m = clamp(
            float(args.trigger_normal_travel_m),
            0.0,
            max(self.max_trigger_normal_travel_m, 1e-3),
        )
        self.requested_trigger_press_normal_travel_m = clamp(
            float(args.trigger_press_normal_travel_m),
            -max(self.max_trigger_normal_travel_m, 1e-3),
            max(self.max_trigger_normal_travel_m, 1e-3),
        )
        self.trigger_normal_scale = self._compute_trigger_normal_scale()

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
        self.last_swing_diag = None
        self.phase_seen = []
        self.last_printed_phase = None
        self.last_print_time = rospy.Time(0)
        self.last_missing_target_diagnostic_time = None
        self.contact_active_since = None
        self.trigger_started_at = None
        self.trigger_reference_seed = None
        self.preload_reference_normal_scalar = None
        self.first_contact_hold_time = None
        self.first_compliant_response_time = None
        self.peak_compliant_extra_normal_offset_m = 0.0
        self.peak_target_normal_from_nominal_m = 0.0
        self.min_target_normal_from_nominal_m = 0.0
        self.override_params_active = False
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
        ]
        self.controller_phase_id_to_name = {
            0: "SUPPORT",
            1: "TEST_LIFT_CLEARANCE",
            2: "TEST_PRESS_CONTACT",
            3: "DETACH_SLIDE",
            4: "TANGENTIAL_ALIGN",
            5: "PRELOAD_COMPRESS",
            6: "COMPLIANT_SETTLE",
            7: "ATTACHED_HOLD",
        }
        # Per-controller-phase aggregated statistics so the summary can pinpoint where things went wrong
        # without having to re-parse the full JSONL by hand.
        self.phase_stats = {}

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
        rospy.Subscriber(
            "/control/swing_leg_diag/" + self.leg_name,
            Float32MultiArray,
            self.swing_diag_callback,
            queue_size=50,
        )

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

    def swing_diag_callback(self, msg):
        data = list(msg.data) if msg.data is not None else []
        if len(data) < len(self.diagnostic_field_labels):
            data = data + [0.0] * (len(self.diagnostic_field_labels) - len(data))
        diag = {label: float(data[index]) for index, label in enumerate(self.diagnostic_field_labels)}
        diag["controller_phase_name"] = self.controller_phase_id_to_name.get(int(diag.get("phase_id", -1)), "UNKNOWN")
        self.last_swing_diag = diag

    def _leg_value(self, values, default=None):
        if values is None or self.leg_index >= len(values):
            return default
        return values[self.leg_index]

    def _support_mask_for_leg(self, support):
        mask = [True] * len(self.leg_names)
        mask[self.leg_index] = bool(support)
        return mask

    def _compute_trigger_normal_scale(self):
        weighted_gain = 0.0
        for index in range(min(len(self.wall_normal_body), len(self.body_position_gain))):
            weighted_gain += float(self.body_position_gain[index]) * float(self.wall_normal_body[index]) * float(self.wall_normal_body[index])
        return max(weighted_gain, 1e-3)

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

        if not support_leg:
            body_error_body = vector_scale(self.wall_normal_body, self.requested_trigger_normal_travel_m / self.trigger_normal_scale)
            body_error_world = rotate_vector_by_quaternion(body_error_body, self.trigger_reference_seed["orientation"])
            msg.pose.position.x += float(body_error_world[0])
            msg.pose.position.y += float(body_error_world[1])
            msg.pose.position.z += float(body_error_world[2])

        msg.support_mask = self._support_mask_for_leg(support_leg)
        return msg

    def _configure_test_override_params(self):
        rospy.set_param("/swing_leg_controller/test_trigger_leg_name", str(self.leg_name))
        rospy.set_param("/swing_leg_controller/test_trigger_normal_travel_m", float(self.requested_trigger_normal_travel_m))
        rospy.set_param("/swing_leg_controller/test_trigger_press_normal_travel_m", float(self.requested_trigger_press_normal_travel_m))
        # Push dwell times into the controller before the swing starts so the LIFT and PRESS targets are
        # held long enough for the observer (and a human) to verify each apex.
        rospy.set_param("/swing_leg_controller/test_lift_dwell_s", float(max(self.args.lift_dwell_s, 0.0)))
        rospy.set_param("/swing_leg_controller/test_press_dwell_s", float(max(self.args.press_dwell_s, 0.0)))
        self.override_params_active = True

    def _clear_test_override_params(self):
        if not self.override_params_active:
            return

        for param_name in [
            "/swing_leg_controller/test_trigger_leg_name",
            "/swing_leg_controller/test_trigger_normal_travel_m",
            "/swing_leg_controller/test_trigger_press_normal_travel_m",
            "/swing_leg_controller/test_lift_dwell_s",
            "/swing_leg_controller/test_press_dwell_s",
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
        state_publishers = self._topic_publishers("/state/estimated")
        body_reference_publishers = self._topic_publishers("/control/body_reference")
        print(
            "[{stamp:8.3f}] diagnostic: no /control/swing_leg_target message for leg={leg}. "
            "publishers swing_target={swing} state={state} body_reference={body}. "
            "received state={state_seen} body_reference={body_seen} trigger_swing={trigger}.".format(
                stamp=now.to_sec(),
                leg=self.leg_name,
                swing=swing_publishers if swing_publishers else ["none"],
                state=state_publishers if state_publishers else ["none"],
                body=body_reference_publishers if body_reference_publishers else ["none"],
                state_seen=str(self.last_estimated_state is not None),
                body_seen=str(self.last_body_reference is not None),
                trigger=str(bool(self.args.trigger_swing)),
            ),
            file=sys.stderr,
        )
        sys.stderr.flush()
        self.last_missing_target_diagnostic_time = now

    def _phase_from_outputs(self):
        if self.last_swing_target is None:
            return "NO_TARGET"

        support_leg = bool(self.last_swing_target.support_leg)
        normal_force_limit = float(self.last_swing_target.desired_normal_force_limit)
        skirt_target = float(self.last_swing_target.skirt_compression_target)
        target_normal_from_nominal = vector_dot(
            [
                float(self.last_swing_target.center.x),
                float(self.last_swing_target.center.y),
                float(self.last_swing_target.center.z),
            ],
            self.wall_normal_body,
        ) - self.nominal_normal_scalar

        if support_leg:
            if abs(normal_force_limit - self.attach_normal_force_limit_n) <= self.force_limit_tolerance_n and skirt_target >= self.preload_skirt_target:
                return "ATTACHED_HOLD"
            return "SUPPORT"
        if self.args.trigger_swing and abs(normal_force_limit) <= self.force_limit_tolerance_n:
            staged_midpoint = 0.5 * (self.requested_trigger_normal_travel_m + self.requested_trigger_press_normal_travel_m)
            if target_normal_from_nominal >= staged_midpoint:
                return "TEST_LIFT_CLEARANCE"
            return "TEST_PRESS_CONTACT"
        if abs(normal_force_limit - self.attach_normal_force_limit_n) <= self.force_limit_tolerance_n:
            return "COMPLIANT_SETTLE"
        if abs(normal_force_limit - self.preload_normal_force_limit_n) <= self.force_limit_tolerance_n:
            return "PRELOAD_COMPRESS"
        if skirt_target >= self.light_contact_skirt_target:
            return "DETACH_OR_TANGENTIAL"
        return "SWING_FREE"

    def _target_normal_metrics(self, phase, swing, contact_hold_satisfied, now):
        if swing is None:
            return {}

        target_center = [float(swing.center.x), float(swing.center.y), float(swing.center.z)]
        target_velocity = [float(swing.center_velocity.x), float(swing.center_velocity.y), float(swing.center_velocity.z)]
        target_normal_scalar = vector_dot(target_center, self.wall_normal_body)
        target_normal_velocity = vector_dot(target_velocity, self.wall_normal_body)
        target_normal_from_nominal = target_normal_scalar - self.nominal_normal_scalar
        self.peak_target_normal_from_nominal_m = max(self.peak_target_normal_from_nominal_m, target_normal_from_nominal)
        self.min_target_normal_from_nominal_m = min(self.min_target_normal_from_nominal_m, target_normal_from_nominal)

        if phase in ["PRELOAD_COMPRESS", "COMPLIANT_SETTLE"] and not contact_hold_satisfied:
            self.preload_reference_normal_scalar = target_normal_scalar

        if contact_hold_satisfied and self.first_contact_hold_time is None:
            self.first_contact_hold_time = float(now.to_sec())

        compliant_extra_normal_offset = 0.0
        compliant_response_detected = False
        compliant_response_latency_s = None
        preload_reference_from_nominal = None
        if self.preload_reference_normal_scalar is not None:
            preload_reference_from_nominal = self.preload_reference_normal_scalar - self.nominal_normal_scalar
            compliant_extra_normal_offset = target_normal_scalar - self.preload_reference_normal_scalar
            if compliant_extra_normal_offset > 0.0:
                self.peak_compliant_extra_normal_offset_m = max(self.peak_compliant_extra_normal_offset_m, compliant_extra_normal_offset)
            if contact_hold_satisfied and compliant_extra_normal_offset > 1e-4:
                compliant_response_detected = True
                if self.first_compliant_response_time is None:
                    self.first_compliant_response_time = float(now.to_sec())

        if self.first_contact_hold_time is not None and self.first_compliant_response_time is not None:
            compliant_response_latency_s = self.first_compliant_response_time - self.first_contact_hold_time

        return {
            "target_normal_position_m": round(target_normal_scalar, 5),
            "target_normal_velocity_mps": round(target_normal_velocity, 5),
            "target_normal_from_nominal_m": round(target_normal_from_nominal, 5),
            "preload_reference_normal_from_nominal_m": None if preload_reference_from_nominal is None else round(preload_reference_from_nominal, 5),
            "compliant_extra_normal_offset_m": round(compliant_extra_normal_offset, 5),
            "compliant_response_detected": bool(compliant_response_detected),
            "compliant_response_latency_s": None if compliant_response_latency_s is None else round(compliant_response_latency_s, 4),
            "peak_target_normal_from_nominal_m": round(self.peak_target_normal_from_nominal_m, 5),
            "peak_compliant_extra_normal_offset_m": round(self.peak_compliant_extra_normal_offset_m, 5),
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
        actual_normal_scalar = vector_dot(controller_frame_center, self.wall_normal_body)
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
            "actual_normal_position_m": round(actual_normal_scalar, 5),
            "actual_normal_from_nominal_m": round(actual_normal_scalar - self.nominal_normal_scalar, 5),
        }

        target_normal_position = data.get("target_normal_position_m")
        if target_normal_position is not None:
            metrics["actual_normal_tracking_error_m"] = round(actual_normal_scalar - float(target_normal_position), 5)
        return metrics

    def _snapshot(self, now):
        phase = self._phase_from_outputs()
        est = self.last_estimated_state
        swing = self.last_swing_target
        adhesion = self.last_adhesion_command
        contact_hold_elapsed = self._contact_hold_elapsed(now)

        data = {
            "stamp": now.to_sec(),
            "leg_name": self.leg_name,
            "mission_state": self.last_mission_state,
            "mission_active": bool(self.last_mission_active),
            "inferred_phase": phase,
            "contact_hold_elapsed_s": round(contact_hold_elapsed, 4),
            "contact_hold_satisfied": contact_hold_elapsed >= self.contact_hold_min_s,
            "trigger_normal_travel_request_m": round(self.requested_trigger_normal_travel_m, 5),
            "trigger_press_normal_travel_request_m": round(self.requested_trigger_press_normal_travel_m, 5),
        }

        if self.last_body_reference is not None and self.leg_index < len(self.last_body_reference.support_mask):
            data["body_reference_support"] = bool(self.last_body_reference.support_mask[self.leg_index])

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
            data.update(self._target_normal_metrics(phase, swing, data["contact_hold_satisfied"], now))

        if est is not None:
            data.update(
                {
                    "measured_contact": bool(self._leg_value(est.measured_contact_mask, False)),
                    "wall_touch": bool(self._leg_value(est.wall_touch_mask, False)),
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

        if self.last_swing_diag is not None:
            data["controller_diag"] = {
                key: round(value, 6) if isinstance(value, float) else value
                for key, value in self.last_swing_diag.items()
            }
            data["controller_phase_name"] = self.last_swing_diag.get("controller_phase_name", "UNKNOWN")

        self._update_phase_stats(data)
        return data

    def _update_phase_stats(self, data):
        # Use the controller-reported phase when available, otherwise the inferred phase, so per-phase
        # peaks always line up with the underlying state machine.
        phase = data.get("controller_phase_name") or data.get("inferred_phase") or "UNKNOWN"
        stats = self.phase_stats.setdefault(
            phase,
            {
                "first_seen_s": float(data.get("stamp", 0.0)),
                "last_seen_s": float(data.get("stamp", 0.0)),
                "samples": 0,
                "target_normal_from_nominal_min_m": None,
                "target_normal_from_nominal_max_m": None,
                "actual_normal_from_nominal_min_m": None,
                "actual_normal_from_nominal_max_m": None,
                "joint_torque_sum_max_nm": 0.0,
                "leg_torque_contact_confidence_max": 0.0,
                "compliant_force_estimate_max_n": 0.0,
                "compliant_normal_offset_max_m": 0.0,
                "estimated_leg_normal_force_max_n": 0.0,
                "measured_contact_seen": False,
                "wall_touch_seen": False,
                "attachment_ready_seen": False,
                "adhesion_ready_seen": False,
                "fan_current_max_a": 0.0,
            },
        )
        stats["last_seen_s"] = float(data.get("stamp", stats["last_seen_s"]))
        stats["samples"] += 1

        def _track_min(key, value):
            if value is None:
                return
            current = stats.get(key)
            stats[key] = float(value) if current is None else min(float(current), float(value))

        def _track_max(key, value):
            if value is None:
                return
            current = stats.get(key)
            stats[key] = float(value) if current is None else max(float(current), float(value))

        target_normal = data.get("target_normal_from_nominal_m")
        _track_min("target_normal_from_nominal_min_m", target_normal)
        _track_max("target_normal_from_nominal_max_m", target_normal)

        actual_normal = data.get("actual_normal_from_nominal_m")
        _track_min("actual_normal_from_nominal_min_m", actual_normal)
        _track_max("actual_normal_from_nominal_max_m", actual_normal)

        _track_max("joint_torque_sum_max_nm", abs(float(data.get("leg_torque_sum_nm", 0.0))))
        _track_max("leg_torque_contact_confidence_max", data.get("leg_torque_contact_confidence", 0.0))
        _track_max("fan_current_max_a", abs(float(data.get("fan_current_a", 0.0))))

        diag = data.get("controller_diag") or {}
        _track_max("compliant_force_estimate_max_n", abs(float(diag.get("compliant_force_estimate_n", 0.0))))
        _track_max("compliant_normal_offset_max_m", abs(float(diag.get("compliant_normal_offset_m", 0.0))))
        _track_max("estimated_leg_normal_force_max_n", abs(float(diag.get("estimated_leg_normal_force_n", 0.0))))

        if data.get("measured_contact"):
            stats["measured_contact_seen"] = True
        if data.get("wall_touch"):
            stats["wall_touch_seen"] = True
        if data.get("attachment_ready"):
            stats["attachment_ready_seen"] = True
        if data.get("adhesion_ready"):
            stats["adhesion_ready_seen"] = True

    def _write_log(self, data):
        with open(self.log_path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(data, ensure_ascii=False) + "\n")

    def _print_snapshot(self, data):
        center = data.get("target_center", {})
        diag = data.get("controller_diag") or {}
        adm_force = float(diag.get("compliant_force_estimate_n", 0.0))
        adm_offset = float(diag.get("compliant_normal_offset_m", 0.0))
        ctrl_phase = data.get("controller_phase_name", "UNKNOWN")
        print(
            "[{stamp:8.3f}] inf={phase:18s} ctrl={ctrl_phase:18s} support={support} measured={measured} wall={wall} hold={hold:5.2f}s "
            "attach={attach} adhesion={adhesion} fan={fan:6.2f}A rpm={rpm:6.0f} force_lim={force:5.1f} "
            "cmd=({x:+.3f},{y:+.3f},{z:+.3f}) act=({ax:+.3f},{ay:+.3f},{az:+.3f}) "
            "tgt_n={normal:+.4f} act_n={actual_normal:+.4f} adm_F={adm_force:+6.2f}N adm_off={adm_offset:+0.4f}m torque={torque:5.2f}Nm".format(
                stamp=data["stamp"],
                phase=data.get("inferred_phase", "UNKNOWN"),
                ctrl_phase=ctrl_phase,
                adm_force=adm_force,
                adm_offset=adm_offset,
                support=str(data.get("target_support_leg", False)),
                measured=str(data.get("measured_contact", False)),
                wall=str(data.get("wall_touch", False)),
                hold=data.get("contact_hold_elapsed_s", 0.0),
                attach=str(data.get("attachment_ready", False)),
                adhesion=str(data.get("adhesion_ready", False)),
                fan=data.get("fan_current_a", 0.0),
                rpm=data.get("adhesion_command_rpm", 0.0),
                force=data.get("target_normal_force_limit_n", 0.0),
                x=center.get("x", 0.0),
                y=center.get("y", 0.0),
                z=center.get("z", 0.0),
                ax=(data.get("actual_center_estimate") or {}).get("x", 0.0),
                ay=(data.get("actual_center_estimate") or {}).get("y", 0.0),
                az=(data.get("actual_center_estimate") or {}).get("z", 0.0),
                normal=data.get("target_normal_from_nominal_m", 0.0),
                actual_normal=data.get("actual_normal_from_nominal_m", 0.0),
                torque=data.get("leg_torque_sum_nm", 0.0),
            )
        )
        sys.stdout.flush()

    def _print_summary(self):
        print("\nSummary:")
        print("  log_path=%s" % self.log_path)
        print("  requested_trigger_lift_normal_travel_m=%.4f" % self.requested_trigger_normal_travel_m)
        print("  requested_trigger_press_normal_travel_m=%.4f" % self.requested_trigger_press_normal_travel_m)
        print("  peak_target_normal_from_nominal_m=%.4f" % self.peak_target_normal_from_nominal_m)
        print("  min_target_normal_from_nominal_m=%.4f" % self.min_target_normal_from_nominal_m)
        print("  peak_compliant_extra_normal_offset_m=%.4f" % self.peak_compliant_extra_normal_offset_m)
        if self.first_contact_hold_time is not None:
            print("  first_contact_hold_time_s=%.4f" % self.first_contact_hold_time)
        else:
            print("  first_contact_hold_time_s=not_reached")
        if self.first_compliant_response_time is not None and self.first_contact_hold_time is not None:
            print("  compliant_response_latency_s=%.4f" % (self.first_compliant_response_time - self.first_contact_hold_time))
        else:
            print("  compliant_response_latency_s=not_detected")

        print("\nPer-phase summary (controller-reported phase when available):")
        for phase_name, stats in self.phase_stats.items():
            duration = max(0.0, float(stats.get("last_seen_s", 0.0)) - float(stats.get("first_seen_s", 0.0)))
            print(
                "  [{phase:<22s}] dur={dur:5.2f}s n={n:4d} "
                "tgt_norm[min={tmin}, max={tmax}] act_norm[min={amin}, max={amax}] "
                "torque_max={tq:6.3f}Nm est_force_max={fest:6.3f}N "
                "adm_force_max={af:6.3f}N adm_offset_max={ao:6.4f}m "
                "fan_max={fan:6.3f}A measured={mc} wall={wt} attach={at} adhesion={ad}".format(
                    phase=phase_name,
                    dur=duration,
                    n=int(stats.get("samples", 0)),
                    tmin=self._fmt_optional(stats.get("target_normal_from_nominal_min_m")),
                    tmax=self._fmt_optional(stats.get("target_normal_from_nominal_max_m")),
                    amin=self._fmt_optional(stats.get("actual_normal_from_nominal_min_m")),
                    amax=self._fmt_optional(stats.get("actual_normal_from_nominal_max_m")),
                    tq=float(stats.get("joint_torque_sum_max_nm", 0.0)),
                    fest=float(stats.get("estimated_leg_normal_force_max_n", 0.0)),
                    af=float(stats.get("compliant_force_estimate_max_n", 0.0)),
                    ao=float(stats.get("compliant_normal_offset_max_m", 0.0)),
                    fan=float(stats.get("fan_current_max_a", 0.0)),
                    mc=str(stats.get("measured_contact_seen", False)),
                    wt=str(stats.get("wall_touch_seen", False)),
                    at=str(stats.get("attachment_ready_seen", False)),
                    ad=str(stats.get("adhesion_ready_seen", False)),
                )
            )

        diagnosis = self._build_diagnosis()
        print("\nDiagnosis (PASS/FAIL per check):")
        for label, passed, detail in diagnosis:
            tag = "PASS" if passed else "FAIL"
            print("  [{tag}] {label}: {detail}".format(tag=tag, label=label, detail=detail))

    @staticmethod
    def _fmt_optional(value):
        if value is None:
            return "  n/a "
        return "%+0.4f" % float(value)

    def _build_diagnosis(self):
        results = []
        lift_request = float(self.requested_trigger_normal_travel_m)
        press_request = float(self.requested_trigger_press_normal_travel_m)
        actual_lift_threshold_m = max(0.5 * lift_request, 0.005)
        actual_press_threshold_m = min(0.5 * press_request, -0.005)

        # 1. Did any swing target arrive at all?
        target_seen = self.peak_target_normal_from_nominal_m != 0.0 or self.min_target_normal_from_nominal_m != 0.0
        results.append(
            (
                "swing_target_received",
                bool(target_seen),
                "/control/swing_leg_target {} during the test window".format(
                    "carried non-zero normal motion" if target_seen else "never arrived or stayed at nominal"
                ),
            )
        )

        # 2. Did the commanded trajectory reach the lift apex?
        reached_lift_cmd = self.peak_target_normal_from_nominal_m >= 0.9 * lift_request
        results.append(
            (
                "command_reached_lift",
                bool(reached_lift_cmd),
                "peak_cmd_normal_from_nominal=%.4fm vs target=%+0.4fm" % (self.peak_target_normal_from_nominal_m, lift_request),
            )
        )

        # 3. Did the commanded trajectory reach the press apex?
        reached_press_cmd = self.min_target_normal_from_nominal_m <= 0.9 * press_request
        results.append(
            (
                "command_reached_press",
                bool(reached_press_cmd),
                "min_cmd_normal_from_nominal=%.4fm vs target=%+0.4fm" % (self.min_target_normal_from_nominal_m, press_request),
            )
        )

        # 4. Did the leg actually move during LIFT according to forward kinematics?
        lift_stats = self.phase_stats.get("TEST_LIFT_CLEARANCE", {})
        actual_lift_max = lift_stats.get("actual_normal_from_nominal_max_m")
        actual_lift_ok = actual_lift_max is not None and float(actual_lift_max) >= actual_lift_threshold_m
        results.append(
            (
                "actual_motion_during_lift",
                bool(actual_lift_ok),
                "FK actual_normal_from_nominal_max=%s vs threshold=%+0.4fm" % (
                    self._fmt_optional(actual_lift_max),
                    actual_lift_threshold_m,
                ),
            )
        )

        # 5. Did the leg actually move during PRESS according to forward kinematics?
        press_stats = self.phase_stats.get("TEST_PRESS_CONTACT", {})
        actual_press_min = press_stats.get("actual_normal_from_nominal_min_m")
        actual_press_ok = actual_press_min is not None and float(actual_press_min) <= actual_press_threshold_m
        results.append(
            (
                "actual_motion_during_press",
                bool(actual_press_ok),
                "FK actual_normal_from_nominal_min=%s vs threshold=%+0.4fm" % (
                    self._fmt_optional(actual_press_min),
                    actual_press_threshold_m,
                ),
            )
        )

        # 6. Did the joint torques rise during PRESS, suggesting actual ground reaction?
        press_torque_max = float(press_stats.get("joint_torque_sum_max_nm", 0.0)) if press_stats else 0.0
        press_force_max = float(press_stats.get("estimated_leg_normal_force_max_n", 0.0)) if press_stats else 0.0
        torque_ok = press_torque_max >= max(self.args.press_torque_threshold_nm, 0.0)
        results.append(
            (
                "press_phase_torque_evidence",
                bool(torque_ok),
                "max_joint_torque_sum_during_press=%.3fNm; controller force_estimate_max=%.3fN; threshold=%.3fNm" % (
                    press_torque_max,
                    press_force_max,
                    self.args.press_torque_threshold_nm,
                ),
            )
        )

        # 7. Did the estimator ever flag contact for this leg?
        contact_seen = any(
            stats.get("measured_contact_seen") or stats.get("wall_touch_seen")
            for stats in self.phase_stats.values()
        )
        results.append(
            (
                "estimator_contact_flagged",
                bool(contact_seen),
                "measured_contact or wall_touch ever True during the test" if contact_seen else
                "measured_contact and wall_touch never True; check current/torque thresholds in state_estimator",
            )
        )

        # 8. Did the admittance respond once we entered COMPLIANT_SETTLE?
        settle_stats = self.phase_stats.get("COMPLIANT_SETTLE", {})
        adm_offset_max = float(settle_stats.get("compliant_normal_offset_max_m", 0.0)) if settle_stats else 0.0
        adm_force_max = float(settle_stats.get("compliant_force_estimate_max_n", 0.0)) if settle_stats else 0.0
        admittance_active = adm_offset_max > 1e-4 or adm_force_max > self.args.admittance_force_min_n
        results.append(
            (
                "admittance_engaged_in_settle",
                bool(admittance_active),
                "compliant_normal_offset_max=%.5fm, compliant_force_estimate_max=%.3fN" % (adm_offset_max, adm_force_max),
            )
        )

        return results

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

                phase = data.get("inferred_phase")
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
        help="Publish /control/body_reference to force the selected leg into a pure wall-normal swing-and-attach test. Use only when body_planner is disabled or not publishing.",
    )
    parser.add_argument(
        "--swing-duration-s",
        type=float,
        default=8.0,
        help="When --trigger-swing is enabled, keep the selected leg in swing for this long before restoring support.",
    )
    parser.add_argument(
        "--trigger-normal-travel-m",
        type=float,
        default=0.035,
        help="Requested lift travel along wall normal during the staged trigger test. It is clamped by swing_leg_controller max_position_offset.",
    )
    parser.add_argument(
        "--trigger-press-normal-travel-m",
        type=float,
        default=-0.040,
        help="Requested press target relative to nominal along wall normal after the lift phase.",
    )
    parser.add_argument(
        "--lift-dwell-s",
        type=float,
        default=1.0,
        help="Hold the LIFT apex for at least this long before transitioning to PRESS. Pushed to /swing_leg_controller/test_lift_dwell_s.",
    )
    parser.add_argument(
        "--press-dwell-s",
        type=float,
        default=1.0,
        help="Hold the PRESS apex for at least this long before transitioning to COMPLIANT_SETTLE. Pushed to /swing_leg_controller/test_press_dwell_s.",
    )
    parser.add_argument(
        "--press-torque-threshold-nm",
        type=float,
        default=0.5,
        help="Joint-torque-sum (Nm) threshold considered evidence of ground contact during PRESS for the diagnosis summary.",
    )
    parser.add_argument(
        "--admittance-force-min-n",
        type=float,
        default=2.0,
        help="Minimum compliant_force_estimate (N) considered as evidence the admittance engaged during COMPLIANT_SETTLE.",
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