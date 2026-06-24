#!/usr/bin/env python3
"""Single-leg swing test.

Drives one leg through the full swing_leg_controller swing sequence
by publishing BodyReference with the target leg's support_mask=False
for one control-tick (triggering _start_swing), then immediately
switching to all-support=True so the leg completes its swing phases
and returns to SUPPORT via the use_contact_feedback=False timeout.

Usage:
  # Jetson: roslaunch climbing_bringup jetson_bringup.launch enable_auto_current_control:=false
  # PC:     roslaunch climbing_bringup test_crawl_gait.launch
  rosrun climbing_control_core test_single_leg_swing.py [--leg lf] [--cycles 3]
"""

from __future__ import print_function

import argparse
import csv
import datetime
import os
import subprocess
import time

import rospy
from climbing_msgs.msg import BodyReference, EstimatedState, LegCenterCommand
from geometry_msgs.msg import Pose, Twist
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String, Int32MultiArray

LEG_NAMES = ["lf", "rf", "rr", "lr"]
STATE_FAULT = "FAULT"
STATE_CLIMB = "CLIMB"
SERVO_ENDPOINT_PHASES = {5: "LIFT", 6: "TRANSFER", 2: "PRELOAD"}


def _default_test_logs_dir():
    here = os.path.dirname(os.path.abspath(__file__))
    path = here
    for _ in range(12):
        if os.path.isdir(os.path.join(path, "test_logs")):
            return os.path.join(path, "test_logs")
        if os.path.isdir(os.path.join(path, "src", "climbing_control_core")):
            return os.path.join(path, "test_logs")
        parent = os.path.dirname(path)
        if parent == path:
            break
        path = parent
    ws = os.environ.get("CLIMBING_WS", os.path.join(os.path.expanduser("~"), "climbing_ws"))
    return os.path.join(ws, "test_logs")


class SingleLegSwingTest(object):
    def __init__(self, args):
        rospy.init_node("test_single_leg_swing", anonymous=False)

        self.leg_name = args.leg
        self.leg_index = LEG_NAMES.index(self.leg_name)
        self.cycles = max(1, args.cycles)
        self.swing_wait_s = max(5.0, args.swing_wait)
        self.hold_before_s = max(0.5, args.hold_before)
        self.hold_after_s = max(0.5, args.hold_after)
        self.output_dir = args.output_dir
        self.require_servo_gate = not args.allow_servo_gate_disabled

        self.latest_estimated = None
        self.latest_mission_state = "INIT"
        self.latest_swing_target = {}  # leg_name -> LegCenterCommand
        self.latest_ticks_cmd = None   # JointState from leg_ik_executor
        self.latest_ticks_actual = None  # Int32MultiArray from dynamixel_bridge/joint_ticks
        self.latest_servo_tracking = None
        self.latest_transfer_path = None
        self._cycle_servo_passes = {}
        self._cycle_servo_timeout = False

        self.body_ref_pub = rospy.Publisher("/control/body_reference", BodyReference, queue_size=10)
        self.mission_start_pub = rospy.Publisher("/control/mission_start", Bool, queue_size=10)
        self.mission_pause_pub = rospy.Publisher("/control/mission_pause", Bool, queue_size=10)
        self.swing_reset_pub = rospy.Publisher("/control/swing_leg_reset", Bool, queue_size=2)

        rospy.Subscriber("/state/estimated", EstimatedState, self._est_cb, queue_size=10)
        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self._swing_cb, queue_size=10)
        rospy.Subscriber("/control/mission_state", String, self._state_cb, queue_size=10)
        rospy.Subscriber("/jetson/joint_position_ticks_cmd", JointState, self._ticks_cmd_cb, queue_size=10)
        rospy.Subscriber("/jetson/dynamixel_bridge/joint_ticks", Int32MultiArray, self._ticks_actual_cb, queue_size=10)
        rospy.Subscriber(
            "/control/swing_leg_servo_tracking/" + self.leg_name,
            Float32MultiArray,
            self._servo_tracking_cb,
            queue_size=10,
        )
        rospy.Subscriber(
            "/control/swing_leg_transfer_path/" + self.leg_name,
            Float32MultiArray,
            self._transfer_path_cb,
            queue_size=10,
        )

        rospy.sleep(0.5)

    def _est_cb(self, msg):
        self.latest_estimated = msg

    def _swing_cb(self, msg):
        self.latest_swing_target[msg.leg_name] = msg

    def _state_cb(self, msg):
        self.latest_mission_state = str(msg.data)

    def _ticks_cmd_cb(self, msg):
        self.latest_ticks_cmd = msg

    def _ticks_actual_cb(self, msg):
        self.latest_ticks_actual = msg

    def _servo_tracking_cb(self, msg):
        self.latest_servo_tracking = list(msg.data)

    def _transfer_path_cb(self, msg):
        self.latest_transfer_path = list(msg.data)

    def _servo_tracking_values(self):
        values = list(self.latest_servo_tracking or [])
        while len(values) < 15:
            values.append(0.0)
        return values[:15]

    def _transfer_path_values(self):
        values = list(self.latest_transfer_path or [])
        while len(values) < 12:
            values.append(0.0)
        return values[:12]

    def _capture_servo_endpoint_pass(self, start_sequence):
        values = self._servo_tracking_values()
        if values[1] < 0.5:
            return
        if values[6] >= 0.5:
            self._cycle_servo_timeout = True
        phase_id = int(round(values[7]))
        sequence = int(round(values[8]))
        if sequence > start_sequence and phase_id in SERVO_ENDPOINT_PHASES:
            self._cycle_servo_passes[phase_id] = list(values[12:15])

    def _make_body_ref(self, support_mask):
        msg = BodyReference()
        msg.header.stamp = rospy.Time.now()
        msg.pose = Pose()
        msg.twist = Twist()
        msg.gait_mode = 3
        msg.support_mask = [bool(v) for v in support_mask]
        return msg

    def _publish_ref(self, support_mask, n_ticks=1):
        """Publish BodyReference support_mask for n_ticks (at 50Hz)."""
        for _ in range(n_ticks):
            self.body_ref_pub.publish(self._make_body_ref(support_mask))
            rospy.sleep(0.02)

    def _reset_swing_controller(self):
        for _ in range(3):
            self.swing_reset_pub.publish(Bool(data=True))
            rospy.sleep(0.05)

    def _swing_target_for_leg(self, leg_name):
        cmd = self.latest_swing_target.get(leg_name)
        if cmd is None:
            return None, None, None, None
        return float(cmd.center.x), float(cmd.center.y), float(cmd.center.z), bool(cmd.support_leg)

    def run(self):
        # Wait for streams
        deadline = time.time() + 5.0
        while not rospy.is_shutdown() and time.time() < deadline:
            if self.latest_estimated is not None:
                break
            rospy.sleep(0.1)
        if self.latest_estimated is None:
            rospy.logerr("No EstimatedState after 5s — is state_estimator running?")
            return 1
        if self.require_servo_gate and not rospy.get_param(
                "/swing_leg_controller/servo_tracking_gate_enabled", False):
            rospy.logerr(
                "servo_tracking_gate_enabled is false. Enable it for the endpoint-accuracy test, "
                "or pass --allow-servo-gate-disabled for a motion-only run."
            )
            return 3

        # Kill body_planner to prevent conflicting BodyReference publications
        # (body_planner's sequential gait advances other legs during test)
        subprocess.call(["rosnode", "kill", "/body_planner"], stderr=subprocess.DEVNULL)
        rospy.sleep(0.3)
        rospy.loginfo("body_planner killed (test-only mode)")
        self._reset_swing_controller()

        # Open CSV
        ts = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        csv_path = os.path.join(self.output_dir, "single_swing_%s_%s.csv" % (self.leg_name, ts))
        csv_f = open(csv_path, "w")
        csv_w = csv.writer(csv_f)
        csv_w.writerow([
            "wall_time", "elapsed_s", "cycle", "leg",
            "cmd_x", "cmd_y", "cmd_z", "cmd_support",
            "ujc_x", "ujc_y", "ujc_z",
            "body_x", "body_y", "body_z",
            "mission_state",
            "servo_phase_id", "servo_gate_enabled", "servo_ready",
            "servo_max_abs_error_tick", "servo_within_tolerance_s", "servo_wait_s", "servo_timed_out",
            "servo_last_pass_phase_id", "servo_last_pass_sequence",
            "servo_error_joint1_tick", "servo_error_joint2_tick", "servo_error_joint3_tick",
            "servo_last_pass_error_joint1_tick", "servo_last_pass_error_joint2_tick", "servo_last_pass_error_joint3_tick",
            "transfer_phase_id", "transfer_path_valid", "transfer_path_index", "transfer_path_size",
            "transfer_cmd_x_m", "transfer_cmd_y_m", "transfer_cmd_z_m",
            "transfer_q1_deg", "transfer_q2_deg", "transfer_q3_deg", "transfer_q23_sum_deg", "transfer_lateral_offset_m",
            "ticks_cmd_11", "ticks_cmd_1", "ticks_cmd_2",
            "ticks_cmd_12", "ticks_cmd_3", "ticks_cmd_4",
            "ticks_cmd_13", "ticks_cmd_5", "ticks_cmd_6",
            "ticks_cmd_14", "ticks_cmd_7", "ticks_cmd_8",
            "ticks_act_11", "ticks_act_1", "ticks_act_2",
            "ticks_act_12", "ticks_act_3", "ticks_act_4",
            "ticks_act_13", "ticks_act_5", "ticks_act_6",
            "ticks_act_14", "ticks_act_7", "ticks_act_8",
        ])
        t0 = time.time()
        rate = rospy.Rate(50)

        rospy.loginfo("=" * 60)
        rospy.loginfo("Single-leg swing test: leg=%s, cycles=%d", self.leg_name, self.cycles)
        rospy.loginfo("=" * 60)

        # Transition to CLIMB
        rospy.loginfo("Publishing mission_start...")
        for _ in range(3):
            self.mission_start_pub.publish(Bool(data=True))
            rospy.sleep(0.05)
        rospy.sleep(1.5)
        if self.latest_mission_state == STATE_FAULT:
            rospy.logerr("Mission in FAULT — aborting.")
            csv_f.close()
            return 2
        rospy.loginfo("Mission state: %s", self.latest_mission_state)

        # Initial all-support hold
        rospy.loginfo("Initial all-support hold (%.1f s)...", self.hold_before_s)
        self._publish_ref([True] * 4, int(self.hold_before_s * 50))

        all_cycles_passed = True

        for cycle in range(1, self.cycles + 1):
            rospy.loginfo("")
            rospy.loginfo("--- Cycle %d/%d: swing %s ---", cycle, self.cycles, self.leg_name)

            # Step 1: Trigger swing by publishing one tick of support_mask=False
            mask_swing = [True] * 4
            mask_swing[self.leg_index] = False
            rospy.loginfo("  Trigger swing: support_mask=%s", mask_swing)
            self._publish_ref(mask_swing, 3)

            start_sequence = int(round(self._servo_tracking_values()[8]))
            self._cycle_servo_passes = {}
            self._cycle_servo_timeout = False

            # Step 2: Immediately switch to all-support so swing_leg_controller
            #   can complete the swing and return via use_contact_feedback=False
            #   timeout in ATTACHED_HOLD.
            wait_end = time.time() + self.swing_wait_s
            swing_started = False
            swing_completed = False
            last_log = 0.0

            while not rospy.is_shutdown() and time.time() < wait_end:
                self.body_ref_pub.publish(self._make_body_ref([True] * 4))
                self._capture_servo_endpoint_pass(start_sequence)

                cmd_x, cmd_y, cmd_z, cmd_support = self._swing_target_for_leg(self.leg_name)
                servo_values = self._servo_tracking_values()
                transfer_values = self._transfer_path_values()

                # Log
                est = self.latest_estimated
                ujc_x = ujc_y = ujc_z = 0.0
                body_x = body_y = body_z = 0.0
                if est is not None:
                    if self.leg_index < len(est.universal_joint_center_positions):
                        p = est.universal_joint_center_positions[self.leg_index]
                        ujc_x = float(p.x)
                        ujc_y = float(p.y)
                        ujc_z = float(p.z)
                    bp = est.pose
                    body_x = float(bp.position.x)
                    body_y = float(bp.position.y)
                    body_z = float(bp.position.z)

                csv_w.writerow([
                    "%.4f" % time.time(), "%.4f" % (time.time() - t0),
                    cycle, self.leg_name,
                    "%.4f" % (cmd_x or 0), "%.4f" % (cmd_y or 0), "%.4f" % (cmd_z or 0),
                    int(cmd_support) if cmd_support is not None else 0,
                    "%.4f" % ujc_x, "%.4f" % ujc_y, "%.4f" % ujc_z,
                    "%.4f" % body_x, "%.4f" % body_y, "%.4f" % body_z,
                    self.latest_mission_state,
                    "%.0f" % servo_values[0], "%.0f" % servo_values[1], "%.0f" % servo_values[2],
                    "%.1f" % servo_values[3], "%.3f" % servo_values[4], "%.3f" % servo_values[5], "%.0f" % servo_values[6],
                    "%.0f" % servo_values[7], "%.0f" % servo_values[8],
                    "%.1f" % servo_values[9], "%.1f" % servo_values[10], "%.1f" % servo_values[11],
                    "%.1f" % servo_values[12], "%.1f" % servo_values[13], "%.1f" % servo_values[14],
                    "%.0f" % transfer_values[0], "%.0f" % transfer_values[1], "%.0f" % transfer_values[2], "%.0f" % transfer_values[3],
                    "%.4f" % transfer_values[4], "%.4f" % transfer_values[5], "%.4f" % transfer_values[6],
                    "%.2f" % transfer_values[7], "%.2f" % transfer_values[8], "%.2f" % transfer_values[9],
                    "%.2f" % transfer_values[10], "%.4f" % transfer_values[11],
                    # cmd ticks (leg_ik_executor output): order lf->rf->rr->lr
                    # lf: 11,1,2 | rf: 12,3,4 | rr: 13,5,6 | lr: 14,7,8
                    "%.0f" % (self.latest_ticks_cmd.position[0] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[1] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[2] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[3] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[4] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[5] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[6] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[7] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[8] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[9] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[10] if self.latest_ticks_cmd else 0),
                    "%.0f" % (self.latest_ticks_cmd.position[11] if self.latest_ticks_cmd else 0),
                    # actual ticks (dynamixel_bridge joint_ticks raw values)
                    "%.0f" % (self.latest_ticks_actual.data[0] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[1] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[2] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[3] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[4] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[5] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[6] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[7] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[8] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[9] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[10] if self.latest_ticks_actual else 0),
                    "%.0f" % (self.latest_ticks_actual.data[11] if self.latest_ticks_actual else 0),
                ])

                # Detect swing started (cmd_support transitions False)
                if cmd_support is not None and not cmd_support:
                    swing_started = True

                # Detect swing completed (cmd_support transitions back to True)
                if swing_started and cmd_support is not None and cmd_support:
                    swing_completed = True
                    rospy.loginfo("  -> %s returned to SUPPORT!", self.leg_name)
                    break

                now = time.time()
                if now - last_log > 1.0 and swing_started:
                    rospy.loginfo("  swinging... (%.1f/%.1f s)", now - (wait_end - self.swing_wait_s), self.swing_wait_s)
                    last_log = now

                rate.sleep()

            if swing_completed:
                self._capture_servo_endpoint_pass(start_sequence)
                missing_phases = [
                    phase_name for phase_id, phase_name in SERVO_ENDPOINT_PHASES.items()
                    if phase_id not in self._cycle_servo_passes
                ]
                cycle_passed = (not self.require_servo_gate) or (
                    not self._cycle_servo_timeout and not missing_phases
                )
                if cycle_passed:
                    if self.require_servo_gate:
                        rospy.loginfo(
                            "  Cycle %d PASS: LIFT=%s TRANSFER=%s PRELOAD=%s tick",
                            cycle,
                            ["%.1f" % value for value in self._cycle_servo_passes[5]],
                            ["%.1f" % value for value in self._cycle_servo_passes[6]],
                            ["%.1f" % value for value in self._cycle_servo_passes[2]],
                        )
                    else:
                        rospy.loginfo("  Cycle %d complete (servo gate disabled).", cycle)
                else:
                    rospy.logerr(
                        "  Cycle %d FAIL: timed_out=%s missing=%s endpoint_errors=%s",
                        cycle,
                        self._cycle_servo_timeout,
                        missing_phases,
                        self._cycle_servo_passes,
                    )
                    all_cycles_passed = False
            else:
                rospy.loginfo("  Cycle %d: swing wait timeout (%.1f s).", cycle, self.swing_wait_s)
                all_cycles_passed = False

            # Post-swing all-support hold
            rospy.loginfo("  Post-swing hold (%.1f s)...", self.hold_after_s)
            self._publish_ref([True] * 4, int(self.hold_after_s * 50))

        csv_f.close()
        rospy.loginfo("")
        rospy.loginfo("=" * 60)
        rospy.loginfo("Test complete. CSV -> %s", csv_path)
        rospy.loginfo("=" * 60)

        for _ in range(3):
            self.mission_pause_pub.publish(Bool(data=True))
            rospy.sleep(0.05)

        # Print per-cycle summary
        rospy.loginfo("Summary: leg=%s, cycles=%d", self.leg_name, self.cycles)
        rospy.loginfo("Test finished. Restart the roslaunch to resume body_planner.")
        return 0 if all_cycles_passed else 4


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Single-leg swing test")
    parser.add_argument("--leg", default="lf", choices=LEG_NAMES, help="Leg to swing (default lf)")
    parser.add_argument("--cycles", type=int, default=3, help="Number of swing cycles (default 3)")
    parser.add_argument("--swing-wait", type=float, default=10.0, help="Max wait per swing (s, default 10)")
    parser.add_argument("--hold-before", type=float, default=1.0, help="All-support hold before first swing (s, default 1)")
    parser.add_argument("--hold-after", type=float, default=1.0, help="All-support hold after each swing (s, default 1)")
    parser.add_argument("--output-dir", default=_default_test_logs_dir(), help="CSV output directory")
    parser.add_argument(
        "--allow-servo-gate-disabled",
        action="store_true",
        help="Run without enforcing the +/-10 tick endpoint gate (motion-only test)",
    )

    args = parser.parse_args()
    try:
        exit(SingleLegSwingTest(args).run())
    except rospy.ROSInterruptException:
        pass
