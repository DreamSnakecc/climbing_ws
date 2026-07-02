#!/usr/bin/env python3
"""Calibrate per-fan adhesion current thresholds at several commanded RPMs.

Run with mission_supervisor automatic fan commands disabled. The operator is
prompted to collect detached and attached samples. Raw samples and a threshold
summary are written to CSV; no runtime parameters are changed automatically.
"""

from __future__ import print_function

import argparse
import csv
import datetime
import math
import os
import threading
import time

import rospy
from std_msgs.msg import Float32MultiArray

from climbing_msgs.msg import AdhesionCommand


LEG_NAMES = ["lf", "rf", "rr", "lr"]
CONDITIONS = ["detached", "attached"]
DEFAULT_RPMS = [30000.0, 40000.0, 50000.0, 60000.0]


def percentile(values, fraction):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return float("nan")
    position = max(0.0, min(1.0, float(fraction))) * (len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - lower
    return ordered[lower] + weight * (ordered[upper] - ordered[lower])


def best_threshold(attached_values, detached_values):
    """Return threshold/direction maximizing balanced classification accuracy."""
    attached = [float(value) for value in attached_values]
    detached = [float(value) for value in detached_values]
    if not attached or not detached:
        return None

    unique = sorted(set(attached + detached))
    span = max(unique[-1] - unique[0], 1e-6)
    candidates = [unique[0] - span * 0.01]
    candidates.extend(0.5 * (left + right) for left, right in zip(unique[:-1], unique[1:]))
    candidates.append(unique[-1] + span * 0.01)

    best = None
    for attached_when in ["current_le_threshold", "current_ge_threshold"]:
        for threshold in candidates:
            if attached_when == "current_le_threshold":
                attached_correct = sum(value <= threshold for value in attached)
                detached_correct = sum(value > threshold for value in detached)
            else:
                attached_correct = sum(value >= threshold for value in attached)
                detached_correct = sum(value < threshold for value in detached)
            attached_recall = float(attached_correct) / len(attached)
            detached_recall = float(detached_correct) / len(detached)
            score = 0.5 * (attached_recall + detached_recall)
            result = {
                "threshold_a": float(threshold),
                "attached_when": attached_when,
                "balanced_accuracy": score,
                "attached_recall": attached_recall,
                "detached_recall": detached_recall,
            }
            if best is None or score > best[0]:
                best = (score, result)
    return best[1]


class FanAdhesionCurrentCalibrator(object):
    def __init__(self, args):
        rospy.init_node("calibrate_fan_adhesion_current", anonymous=False)
        self.rpms = [float(value) for value in args.rpms]
        self.settle_s = max(0.0, float(args.settle_s))
        self.sample_s = max(0.1, float(args.sample_s))
        self.cooldown_s = max(0.0, float(args.cooldown_s))
        self.log_rate_hz = max(1.0, float(args.log_rate_hz))
        self.command_rate_hz = max(1.0, float(args.command_rate_hz))
        self.min_rpm_ratio = max(0.0, float(args.min_rpm_ratio))
        self.feedback_timeout_s = max(0.1, float(args.feedback_timeout_s))
        self.output_dir = os.path.abspath(os.path.expanduser(args.output_dir))

        self.lock = threading.Lock()
        self.fan_rpms = None
        self.fan_currents = None
        self.rpm_received_s = None
        self.current_received_s = None

        self.command_pub = rospy.Publisher(
            "/jetson/fan_serial_bridge/adhesion_command",
            AdhesionCommand,
            queue_size=20,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/leg_rpm",
            Float32MultiArray,
            self._rpm_callback,
            queue_size=20,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/fan_currents",
            Float32MultiArray,
            self._current_callback,
            queue_size=20,
        )

    def _rpm_callback(self, msg):
        with self.lock:
            self.fan_rpms = list(msg.data)
            self.rpm_received_s = time.time()

    def _current_callback(self, msg):
        with self.lock:
            self.fan_currents = list(msg.data)
            self.current_received_s = time.time()

    def _snapshot(self):
        with self.lock:
            return (
                None if self.fan_rpms is None else list(self.fan_rpms),
                None if self.fan_currents is None else list(self.fan_currents),
                self.rpm_received_s,
                self.current_received_s,
            )

    def _publish_command(self, target_rpm):
        enabled = abs(float(target_rpm)) > 1e-6
        for leg_index in range(len(LEG_NAMES)):
            msg = AdhesionCommand()
            msg.header.stamp = rospy.Time.now()
            msg.leg_index = leg_index
            msg.mode = 1 if enabled else 0
            msg.target_rpm = float(target_rpm) if enabled else 0.0
            msg.normal_force_limit = 0.0
            msg.required_adhesion_force = 0.0
            self.command_pub.publish(msg)

    def _drive_for(self, target_rpm, duration_s):
        deadline = time.time() + max(0.0, duration_s)
        rate = rospy.Rate(self.command_rate_hz)
        while not rospy.is_shutdown() and time.time() < deadline:
            self._publish_command(target_rpm)
            rate.sleep()

    def _stop_fans(self):
        self._drive_for(0.0, 0.5)

    def _wait_for_feedback(self):
        deadline = time.time() + self.feedback_timeout_s
        while not rospy.is_shutdown() and time.time() < deadline:
            rpms, currents, rpm_time, current_time = self._snapshot()
            if (rpms is not None and currents is not None
                    and len(rpms) >= len(LEG_NAMES)
                    and len(currents) >= len(LEG_NAMES)
                    and rpm_time is not None and current_time is not None):
                return True
            rospy.sleep(0.05)
        rospy.logerr("fan calibration: incomplete RPM/current feedback")
        return False

    @staticmethod
    def _confirm_condition(condition):
        if condition == "detached":
            prompt = (
                "\nDETACHED collection: keep all four suction chambers completely "
                "away from the wall. Press Enter when ready: "
            )
        else:
            prompt = (
                "\nATTACHED collection: press all four suction chambers firmly "
                "against the wall and keep them sealed. Press Enter when ready: "
            )
        try:
            input(prompt)
            return True
        except (EOFError, KeyboardInterrupt):
            return False

    def _sample_point(self, condition, target_rpm, started_s, writer, rows):
        rospy.loginfo(
            "fan calibration: %s at %.0f rpm, settling %.1f s",
            condition, target_rpm, self.settle_s,
        )
        self._drive_for(target_rpm, self.settle_s)

        rospy.loginfo("fan calibration: recording for %.1f s", self.sample_s)
        deadline = time.time() + self.sample_s
        rate = rospy.Rate(self.log_rate_hz)
        last_command_s = 0.0
        while not rospy.is_shutdown() and time.time() < deadline:
            now_s = time.time()
            if now_s - last_command_s >= 1.0 / self.command_rate_hz:
                self._publish_command(target_rpm)
                last_command_s = now_s
            rpms, currents, rpm_time, current_time = self._snapshot()
            if rpms is None or currents is None:
                rate.sleep()
                continue
            feedback_age_s = max(now_s - rpm_time, now_s - current_time)
            if feedback_age_s > self.feedback_timeout_s:
                rospy.logerr("fan calibration: feedback stale for %.2f s", feedback_age_s)
                return False
            row = {
                "wall_time": "%.6f" % now_s,
                "ros_time": "%.6f" % rospy.Time.now().to_sec(),
                "elapsed_s": "%.6f" % (now_s - started_s),
                "condition": condition,
                "target_rpm": "%.3f" % target_rpm,
                "feedback_age_s": "%.6f" % feedback_age_s,
            }
            for index, leg_name in enumerate(LEG_NAMES):
                actual_rpm = abs(float(rpms[index]))
                current_a = float(currents[index])
                row["%s_actual_rpm" % leg_name] = "%.3f" % actual_rpm
                row["%s_current_a" % leg_name] = "%.6f" % current_a
                row["%s_rpm_valid" % leg_name] = int(
                    actual_rpm >= self.min_rpm_ratio * abs(target_rpm)
                )
            writer.writerow(row)
            rows.append(row)
            rate.sleep()
        return True

    @staticmethod
    def _raw_columns():
        columns = [
            "wall_time", "ros_time", "elapsed_s", "condition",
            "target_rpm", "feedback_age_s",
        ]
        for leg_name in LEG_NAMES:
            columns.extend([
                "%s_actual_rpm" % leg_name,
                "%s_current_a" % leg_name,
                "%s_rpm_valid" % leg_name,
            ])
        return columns

    def _write_summary(self, rows, path):
        columns = [
            "leg", "target_rpm", "attached_samples", "detached_samples",
            "attached_median_a", "attached_p05_a", "attached_p95_a",
            "detached_median_a", "detached_p05_a", "detached_p95_a",
            "threshold_a", "attached_when", "balanced_accuracy",
            "attached_recall", "detached_recall", "separation_gap_a", "status",
        ]
        results = []
        for leg_name in LEG_NAMES:
            for target_rpm in self.rpms:
                grouped = {}
                for condition in CONDITIONS:
                    grouped[condition] = [
                        float(row["%s_current_a" % leg_name])
                        for row in rows
                        if row["condition"] == condition
                        and abs(float(row["target_rpm"]) - target_rpm) < 1.0
                        and int(row["%s_rpm_valid" % leg_name]) == 1
                    ]
                attached = grouped["attached"]
                detached = grouped["detached"]
                decision = best_threshold(attached, detached)
                if decision is None:
                    result = {key: "" for key in columns}
                    result.update({
                        "leg": leg_name,
                        "target_rpm": "%.0f" % target_rpm,
                        "attached_samples": len(attached),
                        "detached_samples": len(detached),
                        "status": "INSUFFICIENT_SAMPLES",
                    })
                    results.append(result)
                    continue

                attached_p05 = percentile(attached, 0.05)
                attached_p95 = percentile(attached, 0.95)
                detached_p05 = percentile(detached, 0.05)
                detached_p95 = percentile(detached, 0.95)
                if decision["attached_when"] == "current_le_threshold":
                    separation_gap = detached_p05 - attached_p95
                else:
                    separation_gap = attached_p05 - detached_p95
                status = "PASS" if separation_gap > 0.0 and decision["balanced_accuracy"] >= 0.99 else "OVERLAP"
                results.append({
                    "leg": leg_name,
                    "target_rpm": "%.0f" % target_rpm,
                    "attached_samples": len(attached),
                    "detached_samples": len(detached),
                    "attached_median_a": "%.6f" % percentile(attached, 0.50),
                    "attached_p05_a": "%.6f" % attached_p05,
                    "attached_p95_a": "%.6f" % attached_p95,
                    "detached_median_a": "%.6f" % percentile(detached, 0.50),
                    "detached_p05_a": "%.6f" % detached_p05,
                    "detached_p95_a": "%.6f" % detached_p95,
                    "threshold_a": "%.6f" % decision["threshold_a"],
                    "attached_when": decision["attached_when"],
                    "balanced_accuracy": "%.6f" % decision["balanced_accuracy"],
                    "attached_recall": "%.6f" % decision["attached_recall"],
                    "detached_recall": "%.6f" % decision["detached_recall"],
                    "separation_gap_a": "%.6f" % separation_gap,
                    "status": status,
                })

        with open(path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=columns)
            writer.writeheader()
            writer.writerows(results)

        rospy.loginfo("fan calibration threshold summary: %s", path)
        for result in results:
            rospy.loginfo(
                "  %s %.0f rpm: threshold=%s A rule=%s accuracy=%s gap=%s status=%s",
                result["leg"], float(result["target_rpm"]),
                result["threshold_a"] or "n/a", result["attached_when"] or "n/a",
                result["balanced_accuracy"] or "n/a", result["separation_gap_a"] or "n/a",
                result["status"],
            )
        return results

    def run(self):
        if bool(rospy.get_param("/mission_supervisor/enable_auto_adhesion_commands", False)):
            rospy.logerr(
                "fan calibration: automatic adhesion commands are enabled; restart "
                "PC bringup with enable_auto_adhesion_commands:=false"
            )
            return 4
        if not self._wait_for_feedback():
            return 2

        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        raw_path = os.path.join(self.output_dir, "fan_adhesion_calibration_%s.csv" % stamp)
        summary_path = os.path.join(
            self.output_dir, "fan_adhesion_calibration_%s_thresholds.csv" % stamp
        )
        rows = []
        started_s = time.time()

        try:
            with open(raw_path, "w", newline="") as stream:
                writer = csv.DictWriter(stream, fieldnames=self._raw_columns())
                writer.writeheader()
                for condition in CONDITIONS:
                    self._stop_fans()
                    if not self._confirm_condition(condition):
                        return 0
                    for target_rpm in self.rpms:
                        if not self._sample_point(condition, target_rpm, started_s, writer, rows):
                            return 3
                        self._stop_fans()
                        if self.cooldown_s > 0.0:
                            rospy.sleep(self.cooldown_s)
        finally:
            self._stop_fans()

        rospy.loginfo("fan calibration raw samples: %s", raw_path)
        results = self._write_summary(rows, summary_path)
        return 0 if results and all(result["status"] == "PASS" for result in results) else 5


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rpms", nargs="+", type=float, default=DEFAULT_RPMS)
    parser.add_argument("--settle-s", type=float, default=5.0)
    parser.add_argument("--sample-s", type=float, default=8.0)
    parser.add_argument("--cooldown-s", type=float, default=2.0)
    parser.add_argument("--log-rate-hz", type=float, default=20.0)
    parser.add_argument("--command-rate-hz", type=float, default=20.0)
    parser.add_argument("--min-rpm-ratio", type=float, default=0.90)
    parser.add_argument("--feedback-timeout-s", type=float, default=2.0)
    parser.add_argument("--output-dir", default="~/climbing_ws/test_logs")
    return parser.parse_args()


if __name__ == "__main__":
    try:
        raise SystemExit(FanAdhesionCurrentCalibrator(parse_args()).run())
    except rospy.ROSInterruptException:
        pass
