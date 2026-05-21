#!/usr/bin/env python3
"""
Whole-body crawl gait with integrated fan-motion coordination test.

Monitors the existing gait controller and mission supervisor to validate
that fan suction states properly coordinate with gait phases during
suction-based crawling.

Test flow:
  1. Wait for all ROS topics
  2. INIT hold check (all legs at operating UJC, no support_leg)
  3. User confirm robot ready (unless --no-confirm)
  4. Publish mission_start -> wait STICK -> wait CLIMB
  5. During CLIMB: log unified CSV at --log-rate-hz
     - Detect swing leg transitions via /control/swing_leg_target
     - Record fan RPM/current, body_vx, attachment state for correlation
  6. Pause, save CSV, print summary

CSV fields (25 cols):
  Common  (5): wall_time, elapsed_s, mission_state, mission_active,
               swing_leg, swing_cycle_idx
  Body    (1): body_vx
  Legx4 (5 each, 20): {leg}_cmd_support, {leg}_fan_rpm,
               {leg}_fan_current_a, {leg}_attachment_ready, {leg}_ujc_z

Prerequisites:
  - Jetson: roslaunch climbing_bringup jetson_bringup.launch
  - PC:     roslaunch climbing_bringup test_crawl_gait.launch
            (mission_supervisor enable_auto_adhesion_commands=true)
  - Robot at operating UJC in INIT state
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
from std_msgs.msg import Bool, Float32MultiArray, String

from climbing_msgs.msg import (
    AdhesionCommand,
    BodyReference,
    EstimatedState,
    LegCenterCommand,
)


LEG_NAMES = ["lf", "rf", "rr", "lr"]

STATE_INIT = "INIT"
STATE_STICK = "STICK"
STATE_CLIMB = "CLIMB"
STATE_PAUSE = "PAUSE"
STATE_FAULT = "FAULT"


class CrawlGaitWithFanTester(object):
    def __init__(self, args):
        self.args = args

        # latest data, protected by lock
        self._lock = threading.Lock()
        self._latest_mission_state = None
        self._latest_mission_active = False
        self._latest_body_reference = None
        self._latest_estimated_state = None
        self._latest_swing_targets = {leg: None for leg in LEG_NAMES}
        self._latest_fan_rpm = [0.0, 0.0, 0.0, 0.0]
        self._latest_fan_currents = [0.0, 0.0, 0.0, 0.0]

        # state timeline for summary
        self._state_history = []  # list of (rel_time, state)
        self._fault_seen = False

        # operating UJC for INIT hold check
        operating_z_mm = float(rospy.get_param(
            "/gait_controller/operating_universal_joint_center_z",
            -100.0,
        ))
        self._operating_z_m = operating_z_mm / 1000.0

        # publishers, latch=False
        self._mission_start_pub = rospy.Publisher(
            "/control/mission_start", Bool, queue_size=2,
        )
        self._mission_pause_pub = rospy.Publisher(
            "/control/mission_pause", Bool, queue_size=2,
        )
        self._adhesion_pub = rospy.Publisher(
            "/jetson/fan_serial_bridge/adhesion_command",
            AdhesionCommand,
            queue_size=20,
        )

        # Initialize timing markers BEFORE creating subscribers so that
        # callbacks arriving immediately do not see uninitialized attributes.
        self._t_zero_wall = None
        self._t_zero_ros = None

        # subscribers
        rospy.Subscriber(
            "/control/mission_state", String,
            self._cb_mission_state, queue_size=10,
        )
        rospy.Subscriber(
            "/control/mission_active", Bool,
            self._cb_mission_active, queue_size=10,
        )
        rospy.Subscriber(
            "/control/body_reference", BodyReference,
            self._cb_body_reference, queue_size=20,
        )
        rospy.Subscriber(
            "/state/estimated", EstimatedState,
            self._cb_estimated_state, queue_size=20,
        )
        rospy.Subscriber(
            "/control/swing_leg_target", LegCenterCommand,
            self._cb_swing_target, queue_size=50,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/leg_rpm", Float32MultiArray,
            self._cb_fan_rpm, queue_size=10,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/fan_currents", Float32MultiArray,
            self._cb_fan_currents, queue_size=10,
        )

        # CSV setup, initialized in _open_csv
        self._csv_path = None
        self._csv_file = None
        self._csv_writer = None
        self._csv_columns = self._build_csv_columns()
        self._sample_count = 0

        # swing cycle tracking
        self._prev_support = {leg: None for leg in LEG_NAMES}
        self._swing_count = {leg: 0 for leg in LEG_NAMES}
        self._current_swing_leg = ""    # which leg is currently swinging
        self._swing_cycle_idx = 0       # total completed swing cycles
        # timestamps of swing events for summary
        self._swing_events = []         # list of (rel_time, leg, "start"|"end")

        # body vx accumulation per swing cycle for summary
        self._body_vx_samples = []      # list of vx values for current swing
        self._cycle_vx_stats = []       # list of dicts per completed cycle

    # ------------------------------------------------------------------ #
    # ROS callbacks
    # ------------------------------------------------------------------ #
    def _cb_mission_state(self, msg):
        try:
            value = str(msg.data)
        except (TypeError, ValueError):
            return
        with self._lock:
            previous = self._latest_mission_state
            self._latest_mission_state = value
            if previous != value:
                rel = self._rel_now()
                self._state_history.append((rel, value))
                if value == STATE_FAULT:
                    self._fault_seen = True

    def _cb_mission_active(self, msg):
        with self._lock:
            self._latest_mission_active = bool(msg.data)

    def _cb_body_reference(self, msg):
        with self._lock:
            self._latest_body_reference = msg

    def _cb_estimated_state(self, msg):
        with self._lock:
            self._latest_estimated_state = msg

    def _cb_swing_target(self, msg):
        if msg.leg_name not in LEG_NAMES:
            return
        with self._lock:
            self._latest_swing_targets[msg.leg_name] = msg

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

    # ------------------------------------------------------------------ #
    # timing / utilities
    # ------------------------------------------------------------------ #
    def _rel_now(self):
        if self._t_zero_ros is None:
            return 0.0
        return (rospy.Time.now() - self._t_zero_ros).to_sec()

    def _wait_streams(self, timeout_s):
        """Block until all subscriptions received once. Returns False on timeout"""
        deadline = time.time() + timeout_s
        rospy.loginfo("test_crawl_gait_with_fan: waiting for upstream streams (timeout=%.1fs)...",
                      timeout_s)
        while not rospy.is_shutdown():
            with self._lock:
                ok_state = self._latest_mission_state is not None
                ok_body = self._latest_body_reference is not None
                ok_est = self._latest_estimated_state is not None
                ok_swing = all(
                    self._latest_swing_targets[leg] is not None
                    for leg in LEG_NAMES
                )
            if ok_state and ok_body and ok_est and ok_swing:
                rospy.loginfo("test_crawl_gait_with_fan: all streams ready.")
                return True
            if time.time() >= deadline:
                rospy.logerr(
                    "test_crawl_gait_with_fan: stream readiness timeout. "
                    "state=%s body=%s estimated=%s swing=%s",
                    ok_state, ok_body, ok_est, ok_swing,
                )
                return False
            rospy.sleep(0.1)
        return False

    # ------------------------------------------------------------------ #
    # INIT hold check  (same logic as test_crawl_gait.py)
    # ------------------------------------------------------------------ #
    def _snapshot_init_state(self):
        """One-shot snapshot of state, targets, UJC for diagnostic printout"""
        with self._lock:
            state = self._latest_mission_state
            est = self._latest_estimated_state
            swing = dict(self._latest_swing_targets)
        legs_info = []
        for leg_index, leg in enumerate(LEG_NAMES):
            cmd = swing.get(leg)
            cmd_xyz = (None, None, None)
            cmd_support = None
            if cmd is not None:
                cmd_xyz = (float(cmd.center.x), float(cmd.center.y), float(cmd.center.z))
                cmd_support = bool(cmd.support_leg)
            ujc_xyz = (None, None, None)
            if est is not None and len(est.universal_joint_center_positions) > leg_index:
                p = est.universal_joint_center_positions[leg_index]
                ujc_xyz = (float(p.x), float(p.y), float(p.z))
            legs_info.append({
                "leg": leg,
                "cmd_xyz": cmd_xyz,
                "cmd_support": cmd_support,
                "ujc_xyz": ujc_xyz,
            })
        return state, legs_info

    def _format_init_table(self, legs_info, tol_m):
        """Generate readable INIT state table (one row per leg)"""
        lines = [
            "  leg  cmd_support |     cmd_xyz (m)            |     ujc_xyz (m)            |  dz_to_op (mm)  status",
        ]
        for info in legs_info:
            cmd_xyz = info["cmd_xyz"]
            ujc_xyz = info["ujc_xyz"]
            cmd_support = info["cmd_support"]

            cmd_str = (
                "(%+.4f, %+.4f, %+.4f)" % cmd_xyz
                if cmd_xyz[0] is not None else "(    n/a              )"
            )
            ujc_str = (
                "(%+.4f, %+.4f, %+.4f)" % ujc_xyz
                if ujc_xyz[0] is not None else "(    n/a              )"
            )
            if ujc_xyz[2] is not None:
                dz_mm = (ujc_xyz[2] - self._operating_z_m) * 1000.0
                status = "OK" if abs(ujc_xyz[2] - self._operating_z_m) <= tol_m else "BAD"
                dz_str = "%+8.2f" % dz_mm
            else:
                dz_str = "    n/a "
                status = "?"
            sup_str = "False" if cmd_support is False else (
                "True " if cmd_support is True else "  ?  "
            )
            lines.append(
                "  %-3s    %s     | %s | %s | %s     %s" % (
                    info["leg"], sup_str, cmd_str, ujc_str, dz_str, status,
                )
            )
        return "\n".join(lines)

    def _verify_init_hold(self):
        """Verify all legs in position-hold mode for hold_check_s."""
        check_s = max(0.5, float(self.args.hold_check_s))
        tol_m = max(0.001, float(self.args.hold_tol_mm) / 1000.0)
        rate = rospy.Rate(20.0)
        end_t = time.time() + check_s
        rospy.loginfo(
            "test_crawl_gait_with_fan: verifying INIT hold for %.1fs "
            "(operating_z=%.4f m, tol=%.4f m, %.1f mm)...",
            check_s, self._operating_z_m, tol_m, tol_m * 1000.0,
        )
        violations = []
        latest_state = None
        latest_legs = []
        while not rospy.is_shutdown() and time.time() < end_t:
            state, legs_info = self._snapshot_init_state()
            latest_state = state
            latest_legs = legs_info

            if state != STATE_INIT:
                rospy.logwarn(
                    "test_crawl_gait_with_fan: mission_state=%s (expected INIT). "
                    "Test only starts from INIT. Please exit and verify.",
                    state,
                )
                rospy.loginfo("INIT snapshot:\n%s",
                              self._format_init_table(latest_legs, tol_m))
                return False

            violations = []
            for info in legs_info:
                cmd_support = info["cmd_support"]
                if cmd_support is True:
                    violations.append(
                        "%s: cmd_support_leg=True (expected False)" % info["leg"])
                ujc_xyz = info["ujc_xyz"]
                if ujc_xyz[2] is not None and abs(ujc_xyz[2] - self._operating_z_m) > tol_m:
                    violations.append(
                        "%s: |ujc.z - operating_z|=%.2f mm > %.2f mm" % (
                            info["leg"],
                            (ujc_xyz[2] - self._operating_z_m) * 1000.0,
                            tol_m * 1000.0,
                        )
                    )
            if violations:
                rospy.logerr(
                    "test_crawl_gait_with_fan: INIT hold check failed (%s):\n%s\n  violations: %s",
                    latest_state,
                    self._format_init_table(latest_legs, tol_m),
                    "; ".join(violations),
                )
                if bool(self.args.allow_init_mismatch):
                    rospy.logwarn(
                        "test_crawl_gait_with_fan: --allow-init-mismatch set, continuing despite mismatch.",
                    )
                    return True
                rospy.logwarn(
                    "test_crawl_gait_with_fan: hint - large lr offset likely startup_move incomplete or "
                    "motor zero misalignment; increase --hold-tol-mm to relax."
                )
                return False
            rate.sleep()

        rospy.loginfo(
            "test_crawl_gait_with_fan: INIT hold check passed.\n%s",
            self._format_init_table(latest_legs, tol_m),
        )
        return True

    # ------------------------------------------------------------------ #
    # mission state helpers
    # ------------------------------------------------------------------ #
    def _fan_diagnostic_lines(self):
        """Return compact per-leg fan diagnostics for timeout/FAULT failures."""
        with self._lock:
            est = self._latest_estimated_state
            fan_rpm = list(self._latest_fan_rpm)
            fan_curr = list(self._latest_fan_currents)
            swings = dict(self._latest_swing_targets)

        lines = [
            "  leg  cmd_support  fan_rpm  fan_cur  attach  seal_conf",
        ]
        for leg_index, leg in enumerate(LEG_NAMES):
            cmd = swings.get(leg)
            support = bool(cmd.support_leg) if cmd is not None else "?"
            rpm = fan_rpm[leg_index] if leg_index < len(fan_rpm) else 0.0
            current = fan_curr[leg_index] if leg_index < len(fan_curr) else 0.0
            attach = False
            seal = 0.0
            if est is not None and leg_index < len(est.attachment_ready_mask):
                attach = bool(est.attachment_ready_mask[leg_index])
            if est is not None and leg_index < len(est.seal_confidence):
                seal = float(est.seal_confidence[leg_index])
            lines.append(
                "  %-3s  %-11s  %7.1f  %7.4f  %-6s  %7.3f" % (
                    leg, str(support), rpm, current, str(attach), seal,
                )
            )
        return "\n".join(lines)

    def _wait_state(self, target, timeout_s):
        deadline = time.time() + timeout_s
        last_log = 0.0
        while not rospy.is_shutdown():
            with self._lock:
                state = self._latest_mission_state
            if state == target:
                return True
            if state == STATE_FAULT:
                rospy.logerr(
                    "test_crawl_gait_with_fan: entered FAULT while waiting for %s.\n%s",
                    target,
                    self._fan_diagnostic_lines(),
                )
                return False
            if time.time() >= deadline:
                rospy.logwarn(
                    "test_crawl_gait_with_fan: timeout (%ss) waiting for state %s; current=%s\n%s",
                    timeout_s, target, state, self._fan_diagnostic_lines(),
                )
                return False
            now = time.time()
            if now - last_log > 1.0:
                rospy.loginfo(
                    "test_crawl_gait_with_fan: waiting for %s, current=%s "
                    "(remain %.1fs)...",
                    target, state, max(0.0, deadline - now),
                )
                last_log = now
            rospy.sleep(0.1)
        return False

    def _publish_start(self):
        rospy.loginfo("test_crawl_gait_with_fan: publishing /control/mission_start=true.")
        for _ in range(3):
            self._mission_start_pub.publish(Bool(data=True))
            rospy.sleep(0.05)

    def _publish_pause(self):
        rospy.loginfo("test_crawl_gait_with_fan: publishing /control/mission_pause=true.")
        for _ in range(3):
            self._mission_pause_pub.publish(Bool(data=True))
            rospy.sleep(0.05)

    def _fan_sequential_off(self):
        """Turn off fans one by one with delay to avoid simultaneous current surge.
        
        Uses time.sleep() instead of rospy.sleep() because after test ends,
        ROS clock may be unreliable and rospy.sleep() can block indefinitely.
        """
        rospy.loginfo("test_crawl_gait_with_fan: sequential fan shutdown...")
        for idx, leg in enumerate(LEG_NAMES):
            msg = AdhesionCommand()
            msg.header.stamp = rospy.Time.now()
            msg.leg_index = idx
            msg.mode = 0  # RELEASE
            msg.target_rpm = 0.0
            msg.normal_force_limit = 0.0
            msg.required_adhesion_force = 0.0
            self._adhesion_pub.publish(msg)
            time.sleep(1.0)
            rospy.loginfo("  %s fan: OFF", leg)
        # Send one more round to ensure delivery
        time.sleep(0.1)
        for idx in range(4):
            msg = AdhesionCommand()
            msg.header.stamp = rospy.Time.now()
            msg.leg_index = idx
            msg.mode = 0
            msg.target_rpm = 0.0
            msg.normal_force_limit = 0.0
            msg.required_adhesion_force = 0.0
            self._adhesion_pub.publish(msg)

    def _publish_fan_state(self):
        """Control fans directly based on swing leg detection.
        
        Call this in the recording loop to actively control fans per leg.
        Swing leg gets --fan-swing-rpm, support legs get --fan-target-rpm.
        Requires mission_supervisor enable_auto_adhesion_commands=false.
        """
        with self._lock:
            swings = dict(self._latest_swing_targets)
        for idx, leg in enumerate(LEG_NAMES):
            cmd = swings.get(leg)
            is_support = cmd is None or bool(cmd.support_leg)
            rpm = self.args.fan_target_rpm if is_support else self.args.fan_swing_rpm
            msg = AdhesionCommand()
            msg.header.stamp = rospy.Time.now()
            msg.leg_index = idx
            msg.mode = 0 if rpm <= 0.0 else 1
            msg.target_rpm = float(rpm)
            msg.normal_force_limit = 150.0
            msg.required_adhesion_force = 100.0
            self._adhesion_pub.publish(msg)

    # ------------------------------------------------------------------ #
    # CSV (simplified unified format)
    # ------------------------------------------------------------------ #
    def _build_csv_columns(self):
        cols = [
            "wall_time", "elapsed_s",
            "mission_state", "mission_active",
            "swing_leg", "swing_cycle_idx",
            "body_x", "body_vx", "est_vx",
        ]
        for leg in LEG_NAMES:
            cols.extend([
                "%s_cmd_support" % leg,
                "%s_fan_rpm" % leg,
                "%s_fan_current_a" % leg,
                "%s_attachment_ready" % leg,
                "%s_ujc_x" % leg,
                "%s_ujc_z" % leg,
            ])
        return cols

    def _open_csv(self):
        out_dir = os.path.expanduser(str(self.args.output_dir))
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, "crawl_gait_with_fan_%s.csv" % timestamp)
        f = open(path, "w")
        writer = csv.writer(f)
        writer.writerow(self._csv_columns)
        self._csv_path = path
        self._csv_file = f
        self._csv_writer = writer
        rospy.loginfo("test_crawl_gait_with_fan: writing CSV -> %s", path)

    def _close_csv(self):
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except (IOError, OSError):
                pass
            self._csv_file = None
            self._csv_writer = None

    def _detect_swing_leg(self, swings):
        """Return the name of the leg currently in swing (support_leg=False), or '' if none."""
        for leg in LEG_NAMES:
            cmd = swings.get(leg)
            if cmd is not None and not bool(cmd.support_leg):
                return leg
        return ""

    def _sample_row(self):
        with self._lock:
            state = self._latest_mission_state or ""
            mission_active = self._latest_mission_active
            body = self._latest_body_reference
            est = self._latest_estimated_state
            swings = dict(self._latest_swing_targets)
            fan_rpm = list(self._latest_fan_rpm)
            fan_curr = list(self._latest_fan_currents)

        wall_time = time.time()
        elapsed = self._rel_now()

        # body forward velocity (commanded vs estimated)
        body_x = 0.0
        body_vx = 0.0
        est_vx = 0.0
        if body is not None:
            body_x = float(body.pose.position.x)
            body_vx = float(body.twist.linear.x)
        if est is not None:
            est_vx = float(est.twist.linear.x)

        # detect current swing leg
        new_swing_leg = self._detect_swing_leg(swings)

        # update swing cycle tracking
        if new_swing_leg != self._current_swing_leg:
            if new_swing_leg:
                # a new leg started swinging
                rel = self._rel_now()
                self._swing_events.append((rel, new_swing_leg, "start"))
                # reset body_vx accumulator for new swing
                self._body_vx_samples = []
            else:
                # all legs back to support: cycle complete
                if self._current_swing_leg:
                    rel = self._rel_now()
                    self._swing_events.append((rel, self._current_swing_leg, "end"))
                    self._swing_count[self._current_swing_leg] += 1
                    self._swing_cycle_idx += 1
                    # record vx stats for this cycle
                    if self._body_vx_samples:
                        self._cycle_vx_stats.append({
                            "leg": self._current_swing_leg,
                            "cycle": self._swing_cycle_idx,
                            "mean_vx": sum(self._body_vx_samples) / len(self._body_vx_samples),
                            "min_vx": min(self._body_vx_samples),
                            "max_vx": max(self._body_vx_samples),
                            "samples": len(self._body_vx_samples),
                        })
                    self._body_vx_samples = []
            self._current_swing_leg = new_swing_leg

        # accumulate body_vx for current swing cycle stats
        self._body_vx_samples.append(body_vx)

        row = [
            "%.6f" % wall_time,
            "%.4f" % elapsed,
            state,
            int(bool(mission_active)),
            self._current_swing_leg,
            self._swing_cycle_idx,
            "%.4f" % body_x,
            "%.4f" % body_vx,
            "%.4f" % est_vx,
        ]

        for leg_index, leg in enumerate(LEG_NAMES):
            cmd = swings.get(leg)
            cmd_support = 0
            if cmd is not None:
                cmd_support = int(bool(cmd.support_leg))

            ujc_x = 0.0
            ujc_z = 0.0
            if est is not None and len(est.universal_joint_center_positions) > leg_index:
                ujc_x = float(est.universal_joint_center_positions[leg_index].x)
                ujc_z = float(est.universal_joint_center_positions[leg_index].z)

            attach = 0
            if est is not None and leg_index < len(est.attachment_ready_mask):
                attach = int(bool(est.attachment_ready_mask[leg_index]))

            row.extend([
                cmd_support,
                "%.1f" % float(fan_rpm[leg_index]),
                "%.4f" % float(fan_curr[leg_index]),
                attach,
                "%.4f" % ujc_x,
                "%.4f" % ujc_z,
            ])

        return row

    # ------------------------------------------------------------------ #
    # main flow
    # ------------------------------------------------------------------ #
    def run(self):
        self._t_zero_wall = time.time()
        self._t_zero_ros = rospy.Time.now()

        if not self._wait_streams(self.args.stream_timeout_s):
            return 2

        if not self._verify_init_hold():
            return 3

        if not bool(self.args.no_confirm):
            try:
                input(
                    "\n>>> Confirm robot is on wall (or test stand), "
                    "then press Enter to start STICK -> CLIMB. (Ctrl+C to cancel)\n>>> "
                )
            except (EOFError, KeyboardInterrupt):
                rospy.loginfo("test_crawl_gait_with_fan: user cancelled before start.")
                return 0

        self._open_csv()

        self._publish_start()
        # Wait for STICK (or CLIMB when adhesion_required_count=0 and
        # the system skips STICK entirely).
        if not self._wait_state(STATE_STICK, self.args.stick_timeout_s):
            with self._lock:
                current_state = self._latest_mission_state
            if current_state == STATE_CLIMB:
                rospy.loginfo(
                    "test_crawl_gait_with_fan: adhesion disabled, STICK skipped; "
                    "continuing with CLIMB"
                )
            else:
                self._publish_pause()
                return 4
        if not self._wait_state(STATE_CLIMB, self.args.climb_timeout_s):
            self._publish_pause()
            return 5

        rospy.loginfo(
            "test_crawl_gait_with_fan: CLIMB reached. recording for %.1fs at %.1f Hz...",
            float(self.args.duration), float(self.args.log_rate_hz),
        )

        rate = rospy.Rate(max(1.0, float(self.args.log_rate_hz)))
        end_t = time.time() + max(0.0, float(self.args.duration))
        last_status_log = 0.0
        while not rospy.is_shutdown() and time.time() < end_t:
            with self._lock:
                state = self._latest_mission_state
            if state == STATE_FAULT:
                rospy.logerr("test_crawl_gait_with_fan: FAULT during CLIMB; aborting.")
                break

            # Direct fan control based on swing state
            self._publish_fan_state()

            row = self._sample_row()
            try:
                self._csv_writer.writerow(row)
            except (IOError, ValueError):
                pass
            self._sample_count += 1

            now = time.time()
            if now - last_status_log > 5.0:
                total_swings = sum(self._swing_count.values())
                rospy.loginfo(
                    "test_crawl_gait_with_fan: t=%.1fs samples=%d swings(total=%d, cur=%s)",
                    self._rel_now(), self._sample_count,
                    total_swings, self._current_swing_leg or "(all support)",
                )
                last_status_log = now
            rate.sleep()

        self._publish_pause()
        # give mission_supervisor time to transition to PAUSE
        wait_pause_until = time.time() + 3.0
        while not rospy.is_shutdown() and time.time() < wait_pause_until:
            with self._lock:
                if not self._latest_mission_active:
                    break
            rospy.sleep(0.1)

        # Note: fans remain on (suction holds robot on wall).
        # Remove this comment if sequential shutdown becomes needed again.

        self._close_csv()
        self._print_summary()
        return 0 if not self._fault_seen else 6

    def _print_summary(self):
        rospy.loginfo("=" * 72)
        rospy.loginfo("test_crawl_gait_with_fan: summary")
        rospy.loginfo("=" * 72)
        rospy.loginfo("  csv: %s", self._csv_path)
        rospy.loginfo("  samples: %d", self._sample_count)
        rospy.loginfo("  total swing cycles: %d", self._swing_cycle_idx)
        rospy.loginfo("")
        rospy.loginfo("  state transitions:")
        for rel, state in self._state_history:
            rospy.loginfo("    t=+%.2fs  -> %s", rel, state)
        rospy.loginfo("")
        rospy.loginfo("  swing events:")
        for rel, leg, etype in self._swing_events:
            rospy.loginfo("    t=+%.2fs  %s  %s", rel, leg, etype)
        rospy.loginfo("")
        rospy.loginfo("  completed swing cycles per leg:")
        for leg in LEG_NAMES:
            rospy.loginfo("    %s: %d", leg, self._swing_count[leg])
        rospy.loginfo("")
        # body_vx stats per completed cycle
        if self._cycle_vx_stats:
            rospy.loginfo("  body_vx per swing cycle:")
            rospy.loginfo("    %-5s  %-5s  %10s  %10s  %10s  %s" % (
                "cycle", "leg", "mean_vx", "min_vx", "max_vx", "samples"))
            rospy.loginfo("    " + "-" * 55)
            for s in self._cycle_vx_stats:
                rospy.loginfo("    %-5d  %-5s  %+10.4f  %+10.4f  %+10.4f  %d" % (
                    s["cycle"], s["leg"],
                    s["mean_vx"], s["min_vx"], s["max_vx"],
                    s["samples"],
                ))
        rospy.loginfo("=" * 72)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="Crawl gait with integrated fan-motion coordination test",
    )
    parser.add_argument(
        "--duration", type=float, default=60.0,
        help="CLIMB recording duration (s, default 60)",
    )
    parser.add_argument(
        "--log-rate-hz", type=float, default=50.0,
        help="CSV sample rate (Hz, default 50)",
    )
    parser.add_argument(
        "--hold-check-s", type=float, default=3.0,
        help="INIT hold check duration (s, default 3)",
    )
    parser.add_argument(
        "--hold-tol-mm", type=float, default=20.0,
        help="INIT UJC z deviation tolerance (mm, default 20)",
    )
    parser.add_argument(
        "--allow-init-mismatch", action="store_true",
        help="Proceed even if INIT hold check fails",
    )
    parser.add_argument(
        "--stick-timeout-s", type=float, default=15.0,
        help="STICK wait timeout (s, default 15)",
    )
    parser.add_argument(
        "--climb-timeout-s", type=float, default=30.0,
        help="CLIMB wait timeout (s, default 30)",
    )
    parser.add_argument(
        "--stream-timeout-s", type=float, default=10.0,
        help="Stream readiness timeout (s, default 10)",
    )
    parser.add_argument(
        "--output-dir", type=str,
        default=os.path.expanduser("~/climbing_ws/test_logs"),
        help="CSV output directory",
    )
    parser.add_argument(
        "--no-confirm", action="store_true",
        help="Skip manual confirmation (unattended batch mode)",
    )
    parser.add_argument(
        "--fan-target-rpm", type=float, default=35000.0,
        help="Fan RPM when leg is in support (default 35000)",
    )
    parser.add_argument(
        "--fan-swing-rpm", type=float, default=0.0,
        help="Fan RPM when leg is swinging (default 0, fully off)",
    )
    return parser.parse_args(argv)


def main():
    cli_args = rospy.myargv(argv=sys.argv)[1:]
    args = parse_args(cli_args)
    rospy.init_node("test_crawl_gait_with_fan", anonymous=False)
    tester = CrawlGaitWithFanTester(args)
    code = 0
    try:
        code = tester.run()
    except rospy.ROSInterruptException:
        code = 130
    except KeyboardInterrupt:
        code = 130
        try:
            tester._publish_pause()
        except Exception:  # pylint: disable=broad-except
            pass
        # Fans stay on: mission_supervisor in PAUSE uses release_rpm=5000.
        tester._close_csv()
    finally:
        tester._close_csv()
    sys.exit(code)


if __name__ == "__main__":
    main()
