#!/usr/bin/env python3
"""
Fan-only cycle test: sequentially turn off fans in gait order.

Test flow per cycle:
  0. All OFF  (2s settle, baseline recording)
  1. ALL ON   (all 4 fans at --target-rpm for --all-on-duration s)
  2. SEQ OFF  (for each leg in [lf, rf, rr, lr]:
                turn that leg OFF, others remain ON;
                wait --leg-dwell s)
  3. All OFF  (2s settle)
  → Repeat for --cycles times.

Purpose: Validate fan serial bridge response, RPM settling time,
         current draw profile, and cross-coupling between legs.

Publishes:  /jetson/fan_serial_bridge/adhesion_command (AdhesionCommand)
Subscribes: /jetson/fan_serial_bridge/leg_rpm (Float32MultiArray)
            /jetson/fan_serial_bridge/fan_currents (Float32MultiArray)

Prerequisites:
  - Jetson: roslaunch climbing_bringup jetson_bringup.launch
  - PC:     roslaunch climbing_bringup test_crawl_gait.launch ...
            (mission_supervisor must have enable_auto_adhesion_commands=false
             to avoid conflicting fan commands during this test.)
"""

from __future__ import print_function

import argparse
import csv
import datetime as _dt
import os
import sys
import threading
import time

import rospy
from std_msgs.msg import Float32MultiArray
from climbing_msgs.msg import AdhesionCommand


LEG_NAMES = ["lf", "rf", "rr", "lr"]  # sequential off order

# AdhesionCommand mode constants
MODE_RELEASE = 0
MODE_NORMAL = 1

# Index in Float32MultiArray data (0=lf, 1=rf, 2=rr, 3=lr)
LEG_INDEX = {name: idx for idx, name in enumerate(LEG_NAMES)}


class FanCycleTester(object):
    def __init__(self, args):
        self.args = args
        self._leg_names = LEG_NAMES

        # latest sensor data, protected by lock
        self._lock = threading.Lock()
        self._latest_fan_rpm = [0.0, 0.0, 0.0, 0.0]
        self._latest_fan_currents = [0.0, 0.0, 0.0, 0.0]

        # publisher
        self._adhesion_pub = rospy.Publisher(
            "/jetson/fan_serial_bridge/adhesion_command",
            AdhesionCommand,
            queue_size=20,
        )

        # subscribers
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/leg_rpm", Float32MultiArray,
            self._cb_fan_rpm, queue_size=10,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/fan_currents", Float32MultiArray,
            self._cb_fan_currents, queue_size=10,
        )

        # CSV state
        self._csv_path = None
        self._csv_file = None
        self._csv_writer = None

        # timing
        self._t_zero_wall = time.time()
        self._t_zero_ros = rospy.Time.now()

        # publish rate for fan commands during logging
        self._pub_rate_hz = 20.0

    # --- callbacks ---
    def _cb_fan_rpm(self, msg):
        values = [float(v) for v in msg.data[:4]]
        while len(values) < 4:
            values.append(0.0)
        with self._lock:
            self._latest_fan_rpm = values

    def _cb_fan_currents(self, msg):
        values = [float(v) for v in msg.data[:4]]
        while len(values) < 4:
            values.append(0.0)
        with self._lock:
            self._latest_fan_currents = values

    # --- fan command helpers ---
    def _send_adhesion(self, leg_name, mode, target_rpm):
        """Publish a single-leg AdhesionCommand."""
        msg = AdhesionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.leg_index = LEG_INDEX[leg_name]
        msg.mode = mode
        msg.target_rpm = float(target_rpm)
        msg.normal_force_limit = 150.0
        msg.required_adhesion_force = 100.0
        self._adhesion_pub.publish(msg)

    def _all_fans(self, target_rpm):
        """Set all legs to the same RPM."""
        mode = MODE_NORMAL if target_rpm > 0.0 else MODE_RELEASE
        for leg in self._leg_names:
            self._send_adhesion(leg, mode, target_rpm)

    def _leg_seq_off(self, on_legs, off_leg):
        """Turn one leg OFF while keeping others ON at target_rpm."""
        for leg in self._leg_names:
            if leg == off_leg:
                self._send_adhesion(leg, MODE_RELEASE, 0.0)
            else:
                self._send_adhesion(leg, MODE_NORMAL, self.args.target_rpm)

    def _publish_phase(self, phase, elapsed_s, rpm_map):
        pub_rate = rospy.Rate(self._pub_rate_hz)
        for leg in self._leg_names:
            self._send_adhesion(leg, MODE_NORMAL if rpm_map[leg] > 0.0 else MODE_RELEASE, rpm_map[leg])
        pub_rate.sleep()
        # Send a second time to ensure the bridge receives it
        for leg in self._leg_names:
            self._send_adhesion(leg, MODE_NORMAL if rpm_map[leg] > 0.0 else MODE_RELEASE, rpm_map[leg])

    def _create_rpm_map(self, target_rpm):
        """Create a dict mapping leg_name -> RPM with all legs at target."""
        return {leg: target_rpm for leg in self._leg_names}

    # --- CSV ---
    @property
    def _csv_columns(self):
        cols = [
            "wall_time", "ros_time", "elapsed_s",
            "test_phase", "cycle_idx", "step_idx",
        ]
        for leg in LEG_NAMES:
            cols.extend([
                "%s_cmd_rpm" % leg,
                "%s_leg_rpm" % leg,
                "%s_fan_current" % leg,
            ])
        return cols

    def _open_csv(self):
        out_dir = os.path.expanduser(str(self.args.output_dir))
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, "fan_cycle_%s.csv" % timestamp)
        f = open(path, "w")
        writer = csv.writer(f)
        writer.writerow(self._csv_columns)
        self._csv_path = path
        self._csv_file = f
        self._csv_writer = writer
        rospy.loginfo("test_fan_cycle: writing CSV -> %s", path)

    def _close_csv(self):
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except (IOError, OSError):
                pass
            self._csv_file = None
            self._csv_writer = None

    def _sample_row(self, test_phase, cycle_idx, step_idx, rpm_map):
        """Build a CSV row from latest data."""
        with self._lock:
            fan_rpm = list(self._latest_fan_rpm)
            fan_currents = list(self._latest_fan_currents)

        now_wall = time.time()
        now_ros = rospy.Time.now()
        elapsed_s = now_ros.to_sec() - self._t_zero_ros.to_sec()
        if elapsed_s < 0:
            elapsed_s = now_wall - self._t_zero_wall

        row = [
            "%.6f" % now_wall,
            "%.6f" % now_ros.to_sec(),
            "%.6f" % elapsed_s,
            test_phase,
            cycle_idx,
            step_idx,
        ]
        for idx, leg in enumerate(LEG_NAMES):
            row.extend([
                "%.1f" % rpm_map.get(leg, 0.0),
                "%.1f" % fan_rpm[idx],
                "%.4f" % fan_currents[idx],
            ])
        return row

    # --- record loop ---
    def _record(self, duration_s, test_phase, cycle_idx, step_idx, rpm_map):
        """Record for a fixed duration, publishing fan commands at the log rate."""
        rate = rospy.Rate(max(1.0, self._pub_rate_hz))
        end_t = time.time() + max(0.0, duration_s)

        while not rospy.is_shutdown() and time.time() < end_t:
            # re-publish fan state to keep bridge active
            for leg in self._leg_names:
                self._send_adhesion(
                    leg,
                    MODE_NORMAL if rpm_map.get(leg, 0.0) > 0.0 else MODE_RELEASE,
                    rpm_map.get(leg, 0.0),
                )
            row = self._sample_row(test_phase, cycle_idx, step_idx, rpm_map)
            try:
                self._csv_writer.writerow(row)
            except (IOError, ValueError):
                pass
            self._sample_count += 1
            rate.sleep()

    # --- main test flow ---
    def run(self):
        self._t_zero_wall = time.time()
        self._t_zero_ros = rospy.Time.now()
        self._sample_count = 0

        rospy.loginfo("test_fan_cycle: starting fan cycle test")
        rospy.loginfo(
            "  target_rpm=%.0f  all_on=%.1fs  leg_dwell=%.1fs  cycles=%d",
            self.args.target_rpm,
            self.args.all_on_duration,
            self.args.leg_dwell,
            self.args.cycles,
        )

        # Wait for first fan RPM data (indicates fan_serial_bridge is alive)
        rospy.loginfo("test_fan_cycle: waiting for fan_serial_bridge data...")
        deadline = time.time() + 10.0
        while not rospy.is_shutdown() and time.time() < deadline:
            with self._lock:
                if any(abs(v) > 0.01 for v in self._latest_fan_rpm):
                    break
            rospy.sleep(0.2)
        if rospy.is_shutdown():
            return 0
        with self._lock:
            has_data = any(abs(v) > 0.01 for v in self._latest_fan_rpm)
        if not has_data:
            rospy.logwarn("test_fan_cycle: no fan RPM data received within 10s; continuing anyway")

        self._open_csv()

        for cycle in range(self.args.cycles):
            rospy.loginfo("=== test_fan_cycle: cycle %d/%d ===", cycle + 1, self.args.cycles)

            # Phase 0: All OFF (baseline)
            rospy.loginfo("  PHASE all_off (baseline)...")
            off_map = {leg: 0.0 for leg in self._leg_names}
            self._all_fans(0.0)
            rospy.sleep(0.1)  # let command arrive
            self._record(2.0, "all_off", cycle + 1, 0, off_map)

            # Phase 1: ALL ON
            rospy.loginfo("  PHASE all_on (all at %.0f RPM)...", self.args.target_rpm)
            on_map = {leg: self.args.target_rpm for leg in self._leg_names}
            self._all_fans(self.args.target_rpm)
            rospy.sleep(0.1)
            self._record(self.args.all_on_duration, "all_on", cycle + 1, 0, on_map)

            # Phase 2: Sequential OFF
            for step_idx, leg in enumerate(self._leg_names):
                rospy.loginfo("  PHASE seq_off: turning %s OFF", leg)
                for l in self._leg_names:
                    if l == leg:
                        self._send_adhesion(l, MODE_RELEASE, 0.0)
                    else:
                        self._send_adhesion(l, MODE_NORMAL, self.args.target_rpm)
                rospy.sleep(0.1)

                # Build the RPM map for this step: only 'leg' is 0
                step_map = {l: (0.0 if l == leg else self.args.target_rpm) for l in self._leg_names}
                self._record(self.args.leg_dwell, "seq_off", cycle + 1, step_idx + 1, step_map)

            # Phase 3: All OFF (settle)
            rospy.loginfo("  PHASE all_off (settle)...")
            self._all_fans(0.0)
            rospy.sleep(0.1)
            self._record(2.0, "all_off", cycle + 1, len(self._leg_names) + 1, off_map)

        self._close_csv()
        rospy.loginfo("test_fan_cycle: done, %d samples written to %s", self._sample_count, self._csv_path)
        return 0


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Fan-only cycle test: all-on then sequential off in gait order",
    )
    parser.add_argument(
        "--target-rpm", type=float, default=35000.0,
        help="Fan target RPM when ON (default 35000)",
    )
    parser.add_argument(
        "--all-on-duration", type=float, default=5.0,
        help="Duration to keep all fans on (s, default 5)",
    )
    parser.add_argument(
        "--leg-dwell", type=float, default=4.0,
        help="Duration after each leg is turned off (s, default 4)",
    )
    parser.add_argument(
        "--cycles", type=int, default=3,
        help="Number of test cycles (default 3)",
    )
    parser.add_argument(
        "--output-dir", type=str, default="test_logs",
        help="CSV output directory (default test_logs)",
    )
    return parser.parse_args(argv)


def main():
    argv = rospy.myargv(sys.argv)[1:]
    args = parse_args(argv)

    rospy.init_node("test_fan_cycle", anonymous=False)

    tester = FanCycleTester(args)
    try:
        ret = tester.run()
    except rospy.ROSInterruptException:
        ret = 0
    sys.exit(ret)


if __name__ == "__main__":
    main()
