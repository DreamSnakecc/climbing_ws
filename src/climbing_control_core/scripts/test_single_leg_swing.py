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
import time

import rospy
from climbing_msgs.msg import BodyReference, EstimatedState, LegCenterCommand
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool, String

LEG_NAMES = ["lf", "rf", "rr", "lr"]
STATE_FAULT = "FAULT"
STATE_CLIMB = "CLIMB"


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

        self.latest_estimated = None
        self.latest_mission_state = "INIT"
        self.latest_swing_target = {}  # leg_name -> LegCenterCommand

        self.body_ref_pub = rospy.Publisher("/control/body_reference", BodyReference, queue_size=10)
        self.mission_start_pub = rospy.Publisher("/control/mission_start", Bool, queue_size=10)
        self.mission_pause_pub = rospy.Publisher("/control/mission_pause", Bool, queue_size=10)

        rospy.Subscriber("/state/estimated", EstimatedState, self._est_cb, queue_size=10)
        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self._swing_cb, queue_size=10)
        rospy.Subscriber("/control/mission_state", String, self._state_cb, queue_size=10)

        rospy.sleep(0.5)

    def _est_cb(self, msg):
        self.latest_estimated = msg

    def _swing_cb(self, msg):
        self.latest_swing_target[msg.leg_name] = msg

    def _state_cb(self, msg):
        self.latest_mission_state = str(msg.data)

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

        for cycle in range(1, self.cycles + 1):
            rospy.loginfo("")
            rospy.loginfo("--- Cycle %d/%d: swing %s ---", cycle, self.cycles, self.leg_name)

            # Step 1: Trigger swing by publishing one tick of support_mask=False
            mask_swing = [True] * 4
            mask_swing[self.leg_index] = False
            rospy.loginfo("  Trigger swing: support_mask=%s", mask_swing)
            self._publish_ref(mask_swing, 3)

            # Step 2: Immediately switch to all-support so swing_leg_controller
            #   can complete the swing and return via use_contact_feedback=False
            #   timeout in ATTACHED_HOLD.
            wait_end = time.time() + self.swing_wait_s
            swing_started = False
            swing_completed = False
            last_log = 0.0

            while not rospy.is_shutdown() and time.time() < wait_end:
                self.body_ref_pub.publish(self._make_body_ref([True] * 4))

                cmd_x, cmd_y, cmd_z, cmd_support = self._swing_target_for_leg(self.leg_name)

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
                rospy.loginfo("  Cycle %d complete.", cycle)
            else:
                rospy.loginfo("  Cycle %d: swing wait timeout (%.1f s).", cycle, self.swing_wait_s)

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
        return 0


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Single-leg swing test")
    parser.add_argument("--leg", default="lf", choices=LEG_NAMES, help="Leg to swing (default lf)")
    parser.add_argument("--cycles", type=int, default=3, help="Number of swing cycles (default 3)")
    parser.add_argument("--swing-wait", type=float, default=10.0, help="Max wait per swing (s, default 10)")
    parser.add_argument("--hold-before", type=float, default=1.0, help="All-support hold before first swing (s, default 1)")
    parser.add_argument("--hold-after", type=float, default=1.0, help="All-support hold after each swing (s, default 1)")
    parser.add_argument("--output-dir", default=_default_test_logs_dir(), help="CSV output directory")

    args = parser.parse_args()
    try:
        exit(SingleLegSwingTest(args).run())
    except rospy.ROSInterruptException:
        pass
