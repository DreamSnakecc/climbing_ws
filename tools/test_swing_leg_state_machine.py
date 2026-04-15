#!/usr/bin/env python3

import argparse
import copy
import json
import os
import sys

import rosgraph
import rospy
from climbing_msgs.msg import AdhesionCommand, BodyReference, EstimatedState, LegCenterCommand
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String


def clamp(value, lower_bound, upper_bound):
    return max(lower_bound, min(upper_bound, value))


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

        self.preload_normal_force_limit_n = float(rospy.get_param("/swing_leg_controller/preload_normal_force_limit_n", 15.0))
        self.attach_normal_force_limit_n = float(rospy.get_param("/swing_leg_controller/attach_normal_force_limit_n", 25.0))
        self.contact_hold_min_s = float(rospy.get_param("/swing_leg_controller/contact_hold_min_s", 0.04))
        self.preload_skirt_target = float(rospy.get_param("/swing_leg_controller/preload_skirt_compression_target", 0.85))
        self.light_contact_skirt_target = float(rospy.get_param("/swing_leg_controller/light_contact_skirt_compression_target", 0.20))
        self.force_limit_tolerance_n = max(float(args.force_limit_tolerance_n), 1e-3)

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
        self.phase_seen = []
        self.last_printed_phase = None
        self.last_print_time = rospy.Time(0)
        self.last_missing_target_diagnostic_time = None
        self.contact_active_since = None
        self.trigger_started_at = None

        self.body_reference_pub = None
        if self.args.trigger_swing:
            self.body_reference_pub = rospy.Publisher("/control/body_reference", BodyReference, queue_size=20)

        rospy.Subscriber("/control/body_reference", BodyReference, self.body_reference_callback, queue_size=20)
        rospy.Subscriber("/state/estimated", EstimatedState, self.estimated_state_callback, queue_size=20)
        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self.swing_target_callback, queue_size=50)
        rospy.Subscriber("/jetson/fan_serial_bridge/adhesion_command", AdhesionCommand, self.adhesion_command_callback, queue_size=20)
        rospy.Subscriber("/control/mission_state", String, self.mission_state_callback, queue_size=20)
        rospy.Subscriber("/control/mission_active", Bool, self.mission_active_callback, queue_size=20)
        rospy.Subscriber("/jetson/dynamixel_bridge/joint_state", JointState, self.joint_state_callback, queue_size=20)

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

    def _leg_value(self, values, default=None):
        if values is None or self.leg_index >= len(values):
            return default
        return values[self.leg_index]

    def _support_mask_for_leg(self, support):
        mask = [True] * len(self.leg_names)
        mask[self.leg_index] = bool(support)
        return mask

    def _make_trigger_body_reference(self, support_leg):
        msg = BodyReference()
        msg.header.stamp = rospy.Time.now()
        if self.last_body_reference is not None:
            msg.pose = copy.deepcopy(self.last_body_reference.pose)
            msg.twist = copy.deepcopy(self.last_body_reference.twist)
            msg.gait_mode = int(self.last_body_reference.gait_mode)
        else:
            msg.pose = Pose()
            msg.pose.orientation.w = 1.0
            msg.twist = Twist()
            msg.gait_mode = 0
        msg.support_mask = self._support_mask_for_leg(support_leg)
        return msg

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

        if support_leg:
            if abs(normal_force_limit - self.attach_normal_force_limit_n) <= self.force_limit_tolerance_n and skirt_target >= self.preload_skirt_target:
                return "ATTACHED_HOLD"
            return "SUPPORT"
        if abs(normal_force_limit - self.attach_normal_force_limit_n) <= self.force_limit_tolerance_n:
            return "COMPLIANT_SETTLE"
        if abs(normal_force_limit - self.preload_normal_force_limit_n) <= self.force_limit_tolerance_n:
            return "PRELOAD_COMPRESS"
        if skirt_target >= self.light_contact_skirt_target:
            return "DETACH_OR_TANGENTIAL"
        return "SWING_FREE"

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

        return data

    def _write_log(self, data):
        with open(self.log_path, "a", encoding="utf-8") as stream:
            stream.write(json.dumps(data, ensure_ascii=False) + "\n")

    def _print_snapshot(self, data):
        center = data.get("target_center", {})
        print(
            "[{stamp:8.3f}] phase={phase:18s} support={support} measured={measured} wall={wall} hold={hold:5.2f}s "
            "attach={attach} adhesion={adhesion} fan={fan:8.2f}A rpm={rpm:8.1f} force_lim={force:6.2f} "
            "center=({x:+.3f},{y:+.3f},{z:+.3f}) torque={torque:6.2f}Nm seal={seal:4.2f}".format(
                stamp=data["stamp"],
                phase=data.get("inferred_phase", "UNKNOWN"),
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
                torque=data.get("leg_torque_sum_nm", 0.0),
                seal=data.get("seal_confidence", 0.0),
            )
        )
        sys.stdout.flush()

    def spin(self):
        rate = rospy.Rate(max(self.args.rate_hz, 1.0))
        end_time = rospy.Time.now() + rospy.Duration.from_sec(self.args.duration_s) if self.args.duration_s > 0.0 else None

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

        print("\nLog saved to %s" % self.log_path)
        print("Seen phases: %s" % ", ".join(self.phase_seen if self.phase_seen else ["none"]))


def build_arg_parser():
    parser = argparse.ArgumentParser(
        description="Observe and optionally trigger a single-leg swing state machine run while logging key tuning values."
    )
    parser.add_argument("--leg", default="rr", help="Leg to observe: lf, rf, rr, lr. Default: rr")
    parser.add_argument("--duration-s", type=float, default=20.0, help="Monitoring duration in seconds. Use <=0 to run until Ctrl+C.")
    parser.add_argument("--rate-hz", type=float, default=10.0, help="Loop rate for logging and optional trigger publishing.")
    parser.add_argument("--print-period-s", type=float, default=0.5, help="Minimum print interval while monitoring.")
    parser.add_argument("--output-dir", default="~/climbing_ws/test_logs", help="Directory for JSONL logs.")
    parser.add_argument(
        "--trigger-swing",
        action="store_true",
        help="Publish /control/body_reference to force the selected leg into swing. Use only when body_planner is disabled or not publishing.",
    )
    parser.add_argument(
        "--swing-duration-s",
        type=float,
        default=2.0,
        help="When --trigger-swing is enabled, keep the selected leg in swing for this long before restoring support.",
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