#!/usr/bin/env python3
"""
Individual fan channel test.

Tests each fan channel in isolation: turn ONE fan ON (others OFF),
record response, then turn it OFF.

Also includes a "reverse mapping" test to check if the leg_index→fan
assignment in the STM32 matches what the bridge expects.

Usage:
  rosrun climbing_control_core test_fan_individual.py
  rosrun climbing_control_core test_fan_individual.py --target-rpm 25000
  rosrun climbing_control_core test_fan_individual.py --reverse      # also test reversed leg order
"""

from __future__ import print_function

import argparse
import sys
import threading
import time

import rospy
from std_msgs.msg import Float32MultiArray
from climbing_msgs.msg import AdhesionCommand
from climbing_msgs.srv import SetFanSpeed, SetFanSpeedRequest

LEG_NAMES = ["lf", "rf", "rr", "lr"]
LEG_INDEX = {name: idx for idx, name in enumerate(LEG_NAMES)}

MODE_RELEASE = 0
MODE_NORMAL = 1

SERVICE_TIMEOUT = 3.0  # seconds


class FanIndividualTester(object):
    def __init__(self, args):
        self.args = args
        self._leg_names = LEG_NAMES

        self._lock = threading.Lock()
        self._latest_rpm = [0.0, 0.0, 0.0, 0.0]
        self._latest_currents = [0.0, 0.0, 0.0, 0.0]
        self._msg_count = 0

        self._adhesion_pub = rospy.Publisher(
            "/jetson/fan_serial_bridge/adhesion_command",
            AdhesionCommand,
            queue_size=20,
        )

        rospy.Subscriber(
            "/jetson/fan_serial_bridge/leg_rpm", Float32MultiArray,
            self._cb_rpm, queue_size=10,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/fan_currents", Float32MultiArray,
            self._cb_currents, queue_size=10,
        )

        rospy.loginfo("test_fan_individual: initialized")

    # --- callbacks ---

    def _cb_rpm(self, msg):
        values = [float(v) for v in msg.data[:4]]
        while len(values) < 4:
            values.append(0.0)
        with self._lock:
            self._latest_rpm = values
            self._msg_count += 1

    def _cb_currents(self, msg):
        values = [float(v) for v in msg.data[:4]]
        while len(values) < 4:
            values.append(0.0)
        with self._lock:
            self._latest_currents = values

    # --- helpers ---

    def _wait_for_rpm(self, timeout_s=3.0):
        """Wait until at least one RPM message has arrived. Returns True if data received."""
        start = time.time()
        while time.time() - start < timeout_s:
            with self._lock:
                if self._msg_count > 0:
                    return True
            rospy.sleep(0.05)
        return False

    def _get_snapshot(self):
        with self._lock:
            return (
                list(self._latest_rpm),
                list(self._latest_currents),
            )

    def _publish_all(self, rpm_map):
        """Publish AdhesionCommand for all 4 legs with given RPMs (dict leg_name→rpm)."""
        for leg in self._leg_names:
            target = rpm_map.get(leg, 0.0)
            is_on = target > 0.0
            msg = AdhesionCommand()
            msg.header.stamp = rospy.Time.now()
            msg.leg_index = LEG_INDEX[leg]
            msg.mode = MODE_NORMAL if is_on else MODE_RELEASE
            msg.target_rpm = float(target)
            msg.normal_force_limit = 150.0
            msg.required_adhesion_force = 100.0
            self._adhesion_pub.publish(msg)
        # brief sleep to let the bridge process
        rospy.sleep(0.05)
        # re-publish to ensure delivery
        for leg in self._leg_names:
            target = rpm_map.get(leg, 0.0)
            is_on = target > 0.0
            msg = AdhesionCommand()
            msg.header.stamp = rospy.Time.now()
            msg.leg_index = LEG_INDEX[leg]
            msg.mode = MODE_NORMAL if is_on else MODE_RELEASE
            msg.target_rpm = float(target)
            msg.normal_force_limit = 150.0
            msg.required_adhesion_force = 100.0
            self._adhesion_pub.publish(msg)

    def _print_state(self, label, rpm, currents):
        parts = []
        for i, leg in enumerate(self._leg_names):
            parts.append("%s: rpm=%6.0f  cur=%+.4f" % (
                leg, rpm[i], currents[i]))
        rospy.loginfo("  %s: %s", label, " | ".join(parts))

    def _test_leg(self, leg_name, dwell_on=4.0, dwell_off=2.0, cycle_idx=0):
        """Test one leg: turn only this leg ON, then OFF."""
        target = self.args.target_rpm

        # --- Turn ON only this leg ---
        rospy.loginfo("  >> %s ON (%d RPM, others OFF) ...", leg_name, target)
        rpm_map = {l: (target if l == leg_name else 0.0) for l in self._leg_names}
        self._publish_all(rpm_map)
        rospy.sleep(0.2)

        # Record during ON phase
        end_t = time.time() + dwell_on
        while time.time() < end_t:
            rpm, currents = self._get_snapshot()
            self._print_state("%s ON  %.1fs" % (leg_name, end_t - time.time()), rpm, currents)
            rospy.sleep(0.5)

        rpm_on, cur_on = self._get_snapshot()
        leg_idx = LEG_INDEX[leg_name]
        measured_rpm = rpm_on[leg_idx]

        # --- Turn OFF ---
        rospy.loginfo("  >> %s OFF ...", leg_name)
        rpm_map = {l: 0.0 for l in self._leg_names}
        self._publish_all(rpm_map)
        rospy.sleep(0.2)

        end_t = time.time() + dwell_off
        while time.time() < end_t:
            rpm, currents = self._get_snapshot()
            self._print_state("%s OFF %.1fs" % (leg_name, end_t - time.time()), rpm, currents)
            rospy.sleep(0.5)

        rpm_off, cur_off = self._get_snapshot()
        measured_rpm_off = rpm_off[leg_idx]

        # --- Assess ---
        passed_on = measured_rpm > target * 0.5
        passed_off = abs(measured_rpm_off) < target * 0.1

        status = "PASS" if passed_on and passed_off else "FAIL"
        if not passed_on:
            status += " (ON fail: expected ~%d, got %.0f)" % (target, measured_rpm)
        if not passed_off:
            status += " (OFF fail: expected ~0, got %.0f)" % measured_rpm_off

        rospy.loginfo("  ===> %s: %s", leg_name, status)
        return status, measured_rpm, measured_rpm_off

    def _test_reverse_mapping(self):
        """
        Alternative mapping test: command each leg_index to 35000 while
        others are 0, but *print which serial fan actually responds*.
        This helps detect if STM32 fan IDs are swapped.
        """
        rospy.loginfo("\n=== Reverse mapping test (alternate leg order) ===")

        results = {}
        for leg in self._leg_names:
            rospy.loginfo("\n--- Command: leg=%s (leg_index=%d) = ON, others OFF ---",
                          leg, LEG_INDEX[leg])
            rpm_map = {l: (self.args.target_rpm if l == leg else 0.0) for l in self._leg_names}
            self._publish_all(rpm_map)
            rospy.sleep(2.0)

            rpm, currents = self._get_snapshot()
            self._print_state("Snapshot", rpm, currents)
            results[leg] = rpm

            # Turn off
            self._publish_all({l: 0.0 for l in self._leg_names})
            rospy.sleep(1.0)

        rospy.loginfo("\n=== Reverse mapping summary (which fan actually ran?) ===")
        for cmd_leg, rpm in results.items():
            for i, actual_leg in enumerate(self._leg_names):
                if abs(rpm[i]) > self.args.target_rpm * 0.5:
                    rospy.loginfo("  cmd=%s → physical fan that responded: %s (rpm=%.0f)",
                                  cmd_leg, actual_leg, rpm[i])
        rospy.loginfo("  (if cmd_leg matches physical_leg → mapping is correct)")

    # --- main ---

    def run(self):
        rospy.loginfo("test_fan_individual: starting")
        rospy.loginfo("  target_rpm=%.0f  dwell_on=%.1fs  dwell_off=%.1fs",
                      self.args.target_rpm, self.args.dwell_on, self.args.dwell_off)

        # Wait for telemetry data
        rospy.loginfo("Waiting for fan telemetry data...")
        if not self._wait_for_rpm(10.0):
            rospy.logwarn("No RPM data received within 10s; continuing anyway")

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("=== STEP 1: All OFF (baseline) ===")
        rospy.loginfo("=" * 60)
        self._publish_all({l: 0.0 for l in self._leg_names})
        rospy.sleep(2.0)
        rpm, currents = self._get_snapshot()
        self._print_state("Baseline (all OFF)", rpm, currents)

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("=== STEP 2: Individual leg test (standard order) ===")
        rospy.loginfo("=" * 60)
        results = []
        for leg in self._leg_names:
            rospy.loginfo("\n--- Testing leg %s (leg_index=%d) ---", leg, LEG_INDEX[leg])
            # Ensure all are OFF first
            self._publish_all({l: 0.0 for l in self._leg_names})
            rospy.sleep(1.0)
            status, rpm_on, rpm_off = self._test_leg(leg, self.args.dwell_on, self.args.dwell_off)
            results.append((leg, status, rpm_on, rpm_off))

        rospy.loginfo("\n" + "=" * 60)
        rospy.loginfo("=== STEP 3: Results Summary ===")
        rospy.loginfo("=" * 60)
        for leg, status, rpm_on, rpm_off in results:
            rospy.loginfo("  %s: ON=%.0f RPM  OFF=%.0f RPM  → %s", leg, rpm_on, rpm_off, status)

        # Reverse mapping test
        if self.args.reverse:
            rospy.loginfo("\n" + "=" * 60)
            rospy.loginfo("=== STEP 4: Reverse mapping test ===")
            rospy.loginfo("=" * 60)
            self._test_reverse_mapping()

        rospy.loginfo("\ntest_fan_individual: done")


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Individual fan channel test: test one leg at a time",
    )
    parser.add_argument(
        "--target-rpm", type=float, default=30000.0,
        help="Fan target RPM (default 30000)",
    )
    parser.add_argument(
        "--dwell-on", type=float, default=4.0,
        help="Duration to keep fan ON (s, default 4)",
    )
    parser.add_argument(
        "--dwell-off", type=float, default=3.0,
        help="Duration to observe OFF state (s, default 3)",
    )
    parser.add_argument(
        "--reverse", action="store_true",
        help="Also run reverse mapping test (extra ~30s)",
    )
    return parser.parse_args(argv)


def main():
    argv = rospy.myargv(sys.argv)[1:]
    args = parse_args(argv)

    rospy.init_node("test_fan_individual", anonymous=False)

    tester = FanIndividualTester(args)
    try:
        tester.run()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
