#!/usr/bin/env python3
"""Record static Dynamixel current and mapped torque at the configured work point.

Prerequisites:
  - Jetson bringup is running with leg_ik_executor and dynamixel_bridge.
  - The normal startup move has taken all legs to the operating foot endpoint.
  - No mission or gait test is publishing competing fan commands.

The recorder waits for all motor feedback and a static joint state, starts all
four fans, then waits for the operator to place the robot and press Enter.
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
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray

from climbing_msgs.msg import AdhesionCommand


LEG_NAMES = ["lf", "rf", "rr", "lr"]
DEFAULT_MOTOR_IDS = [11, 1, 2, 15, 12, 3, 4, 16, 13, 5, 6, 17, 14, 7, 8, 18]


def _finite(value):
    try:
        return math.isfinite(float(value))
    except (TypeError, ValueError):
        return False


class StaticMotorCurrentRecorder(object):
    def __init__(self, args):
        rospy.init_node("record_static_motor_current", anonymous=False)
        prefix = "/static_current_recorder/"

        def cfg(name, default):
            return rospy.get_param(prefix + name, default)

        self.fan_rpm = float(args.fan_rpm if args.fan_rpm is not None else cfg("fan_rpm", 5000.0))
        self.fan_mode = int(cfg("fan_mode", 1))
        self.duration_s = max(0.1, float(args.duration if args.duration is not None else cfg("record_duration_s", 10.0)))
        self.log_rate_hz = max(1.0, float(cfg("log_rate_hz", 50.0)))
        self.stream_timeout_s = max(0.1, float(cfg("stream_timeout_s", 10.0)))
        self.static_wait_timeout_s = max(0.1, float(cfg("static_wait_timeout_s", 30.0)))
        self.static_hold_s = max(0.0, float(cfg("static_hold_s", 1.0)))
        self.static_velocity_limit_rad_s = max(0.0, float(cfg("static_velocity_limit_rad_s", 0.03)))
        self.fan_settle_s = max(0.0, float(cfg("fan_settle_s", 2.0)))
        self.output_dir = os.path.abspath(os.path.expanduser(str(
            args.output_dir if args.output_dir else cfg("output_dir", "~/climbing_ws/test_logs")
        )))
        self.surface = str(args.surface or "").strip().lower()
        self.no_confirm = bool(args.no_confirm)

        configured_ids = []
        self.leg_motor_ids = {}
        for leg_name in LEG_NAMES:
            ids = [int(value) for value in rospy.get_param("/legs/%s/motor_ids" % leg_name, [])]
            self.leg_motor_ids[leg_name] = ids
            for motor_id in ids:
                if motor_id not in configured_ids:
                    configured_ids.append(motor_id)
        self.motor_ids = configured_ids or list(DEFAULT_MOTOR_IDS)
        self.torque_model_by_motor = rospy.get_param(
            "/dynamixel_telemetry/torque_model_by_motor", {}
        )
        self.operating_point_mm = [
            float(rospy.get_param("/gait_controller/operating_universal_joint_center_x", 267.13)),
            float(rospy.get_param("/gait_controller/operating_universal_joint_center_y", 0.0)),
            float(rospy.get_param("/gait_controller/operating_universal_joint_center_z", -287.3)),
        ]

        self.lock = threading.Lock()
        self.joint_state = None
        self.joint_state_received_s = None
        self.currents = None
        self.fan_rpm_feedback = None
        self.fan_currents = None
        self.safe_mode = False

        self.adhesion_pub = rospy.Publisher(
            "/jetson/fan_serial_bridge/adhesion_command", AdhesionCommand, queue_size=20
        )
        rospy.Subscriber(
            "/jetson/dynamixel_bridge/joint_state", JointState,
            self._joint_state_cb, queue_size=10,
        )
        rospy.Subscriber(
            "/jetson/dynamixel_bridge/joint_currents", Float32MultiArray,
            self._joint_currents_cb, queue_size=10,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/leg_rpm", Float32MultiArray,
            self._fan_rpm_cb, queue_size=10,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/fan_currents", Float32MultiArray,
            self._fan_currents_cb, queue_size=10,
        )
        rospy.Subscriber(
            "/jetson/local_safety_supervisor/safe_mode", Bool,
            self._safe_mode_cb, queue_size=5,
        )

    def _joint_state_cb(self, msg):
        with self.lock:
            self.joint_state = msg
            self.joint_state_received_s = time.time()

    def _joint_currents_cb(self, msg):
        with self.lock:
            self.currents = list(msg.data) if msg.data else []

    def _fan_rpm_cb(self, msg):
        with self.lock:
            self.fan_rpm_feedback = list(msg.data) if msg.data else []

    def _fan_currents_cb(self, msg):
        with self.lock:
            self.fan_currents = list(msg.data) if msg.data else []

    def _safe_mode_cb(self, msg):
        with self.lock:
            self.safe_mode = bool(msg.data)

    def _snapshot(self):
        with self.lock:
            return (
                self.joint_state,
                None if self.currents is None else list(self.currents),
                None if self.fan_rpm_feedback is None else list(self.fan_rpm_feedback),
                None if self.fan_currents is None else list(self.fan_currents),
                self.joint_state_received_s,
                bool(self.safe_mode),
            )

    @staticmethod
    def _joint_values_by_id(msg, values):
        result = {}
        if msg is None:
            return result
        for motor_name, value in zip(msg.name, values):
            try:
                result[int(motor_name)] = float(value)
            except (TypeError, ValueError):
                continue
        return result

    def _feedback_ready(self):
        joint_state, currents, _, _, received_s, _ = self._snapshot()
        if joint_state is None or currents is None or received_s is None:
            return False
        names = set()
        for name in joint_state.name:
            try:
                names.add(int(name))
            except (TypeError, ValueError):
                pass
        return set(self.motor_ids).issubset(names) and len(currents) >= len(joint_state.name)

    def _wait_for_feedback(self):
        deadline = time.time() + self.stream_timeout_s
        while not rospy.is_shutdown() and time.time() < deadline:
            if self._feedback_ready():
                return True
            rospy.sleep(0.05)
        rospy.logerr("record_static_motor_current: incomplete motor current/joint feedback")
        return False

    def _wait_until_static(self):
        deadline = time.time() + self.static_wait_timeout_s
        stable_since = None
        while not rospy.is_shutdown() and time.time() < deadline:
            joint_state, _, _, _, _, safe_mode = self._snapshot()
            if safe_mode:
                rospy.logerr("record_static_motor_current: safe mode active while waiting for work point")
                return False
            velocities = self._joint_values_by_id(
                joint_state, list(joint_state.velocity) if joint_state is not None else []
            )
            all_static = len(velocities) >= len(self.motor_ids) and all(
                abs(velocities.get(motor_id, float("inf"))) <= self.static_velocity_limit_rad_s
                for motor_id in self.motor_ids
            )
            if all_static:
                stable_since = stable_since or time.time()
                if time.time() - stable_since >= self.static_hold_s:
                    return True
            else:
                stable_since = None
            rospy.sleep(0.05)
        rospy.logerr(
            "record_static_motor_current: motors did not remain below %.4f rad/s for %.2f s",
            self.static_velocity_limit_rad_s, self.static_hold_s,
        )
        return False

    def _publish_fans(self, enabled):
        for index in range(len(LEG_NAMES)):
            msg = AdhesionCommand()
            msg.header.stamp = rospy.Time.now()
            msg.leg_index = index
            msg.mode = self.fan_mode if enabled else 0
            msg.target_rpm = self.fan_rpm if enabled else 0.0
            msg.normal_force_limit = 0.0
            msg.required_adhesion_force = 0.0
            self.adhesion_pub.publish(msg)

    def _confirm_placement(self):
        if self.no_confirm:
            self.surface = self.surface or "unspecified"
            return True
        try:
            if not self.surface:
                self.surface = input(
                    "\nFans are running. Place the robot on the test surface, "
                    "then enter surface label [flat/wall] and press Enter: "
                ).strip().lower()
            else:
                input(
                    "\nFans are running. Place the robot on %s, then press Enter to record: "
                    % self.surface
                )
        except (EOFError, KeyboardInterrupt):
            return False
        self.surface = self.surface or "unspecified"
        return True

    def _columns(self):
        columns = [
            "wall_time", "ros_time", "elapsed_s", "surface",
            "operating_x_mm", "operating_y_mm", "operating_z_mm",
            "fan_command_rpm", "joint_feedback_age_s", "safe_mode",
        ]
        for leg_name in LEG_NAMES:
            columns.extend([
                "%s_fan_rpm" % leg_name,
                "%s_fan_current_a" % leg_name,
            ])
        for motor_id in self.motor_ids:
            columns.extend([
                "motor_%d_torque_model" % motor_id,
                "motor_%d_current_a" % motor_id,
                "motor_%d_torque_nm" % motor_id,
                "motor_%d_position_rad" % motor_id,
                "motor_%d_velocity_rad_s" % motor_id,
            ])
        return columns

    def _row(self, started_s):
        joint_state, currents, fan_rpm, fan_currents, received_s, safe_mode = self._snapshot()
        names = list(joint_state.name) if joint_state is not None else []
        positions = self._joint_values_by_id(
            joint_state, list(joint_state.position) if joint_state is not None else []
        )
        velocities = self._joint_values_by_id(
            joint_state, list(joint_state.velocity) if joint_state is not None else []
        )
        torques = self._joint_values_by_id(
            joint_state, list(joint_state.effort) if joint_state is not None else []
        )
        current_by_id = self._joint_values_by_id(joint_state, currents or [])
        now_s = time.time()
        row = {
            "wall_time": "%.6f" % now_s,
            "ros_time": "%.6f" % rospy.get_time(),
            "elapsed_s": "%.6f" % (now_s - started_s),
            "surface": self.surface,
            "operating_x_mm": "%.6f" % self.operating_point_mm[0],
            "operating_y_mm": "%.6f" % self.operating_point_mm[1],
            "operating_z_mm": "%.6f" % self.operating_point_mm[2],
            "fan_command_rpm": "%.3f" % self.fan_rpm,
            "joint_feedback_age_s": "" if received_s is None else "%.6f" % (now_s - received_s),
            "safe_mode": int(safe_mode),
        }
        for index, leg_name in enumerate(LEG_NAMES):
            row["%s_fan_rpm" % leg_name] = (
                "%.3f" % fan_rpm[index] if fan_rpm is not None and index < len(fan_rpm) else ""
            )
            row["%s_fan_current_a" % leg_name] = (
                "%.6f" % fan_currents[index]
                if fan_currents is not None and index < len(fan_currents) else ""
            )
        for motor_id in self.motor_ids:
            row["motor_%d_torque_model" % motor_id] = str(
                self.torque_model_by_motor.get(str(motor_id), "")
            )
            for suffix, mapping in [
                    ("current_a", current_by_id),
                    ("torque_nm", torques),
                    ("position_rad", positions),
                    ("velocity_rad_s", velocities)]:
                value = mapping.get(motor_id)
                row["motor_%d_%s" % (motor_id, suffix)] = (
                    "%.9f" % value if _finite(value) else ""
                )
        return row

    def _print_summary(self, samples):
        rospy.loginfo("record_static_motor_current: summary (%d samples)", len(samples))
        for motor_id in self.motor_ids:
            currents = []
            torques = []
            for row in samples:
                try:
                    currents.append(float(row["motor_%d_current_a" % motor_id]))
                    torques.append(float(row["motor_%d_torque_nm" % motor_id]))
                except (TypeError, ValueError):
                    pass
            if not currents:
                rospy.logwarn("  ID %d: no valid samples", motor_id)
                continue
            abs_currents = sorted(abs(value) for value in currents)
            abs_torques = sorted(abs(value) for value in torques)
            p95_index = int(round(0.95 * (len(abs_currents) - 1)))
            rospy.loginfo(
                "  ID %2d: current mean=%+.3fA absP95=%.3fA max=%.3fA "
                "torque mean=%+.3fNm absP95=%.3fNm max=%.3fNm",
                motor_id, sum(currents) / len(currents), abs_currents[p95_index],
                max(abs_currents), sum(torques) / len(torques),
                abs_torques[p95_index], max(abs_torques),
            )

    def run(self):
        if bool(rospy.get_param("/mission_supervisor/enable_auto_adhesion_commands", False)):
            rospy.logerr(
                "record_static_motor_current: mission_supervisor automatic fan commands are enabled; "
                "restart PC bringup with enable_auto_adhesion_commands:=false"
            )
            return 4
        if not self._wait_for_feedback() or not self._wait_until_static():
            return 2
        rospy.loginfo(
            "record_static_motor_current: work point static at [%.2f, %.2f, %.2f] mm",
            self.operating_point_mm[0], self.operating_point_mm[1], self.operating_point_mm[2],
        )
        self._publish_fans(True)
        rospy.loginfo("record_static_motor_current: all fans commanded to %.0f rpm", self.fan_rpm)
        rospy.sleep(self.fan_settle_s)
        if not self._confirm_placement():
            return 0

        os.makedirs(self.output_dir, exist_ok=True)
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(self.output_dir, "static_motor_current_%s_%s.csv" % (
            self.surface, timestamp,
        ))
        samples = []
        started_s = time.time()
        rate = rospy.Rate(self.log_rate_hz)
        with open(path, "w", newline="") as stream:
            writer = csv.DictWriter(stream, fieldnames=self._columns())
            writer.writeheader()
            while not rospy.is_shutdown() and time.time() - started_s < self.duration_s:
                self._publish_fans(True)
                row = self._row(started_s)
                if int(row["safe_mode"]):
                    rospy.logerr("record_static_motor_current: safe mode active; stopping record")
                    break
                writer.writerow(row)
                samples.append(row)
                rate.sleep()
        rospy.loginfo("record_static_motor_current: CSV saved to %s", path)
        self._print_summary(samples)
        return 0 if samples else 3


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--fan-rpm", type=float, default=None,
                        help="override /static_current_recorder/fan_rpm")
    parser.add_argument("--duration", type=float, default=None,
                        help="override configured recording duration")
    parser.add_argument("--surface", choices=["flat", "wall", "custom"], default=None)
    parser.add_argument("--output-dir", default=None)
    parser.add_argument("--no-confirm", action="store_true")
    return parser.parse_args(rospy.myargv()[1:])


def main():
    recorder = None
    try:
        recorder = StaticMotorCurrentRecorder(parse_args())
        return recorder.run()
    finally:
        if recorder is not None:
            for _ in range(3):
                recorder._publish_fans(False)
                time.sleep(0.05)
            rospy.loginfo("record_static_motor_current: all fans commanded OFF")


if __name__ == "__main__":
    raise SystemExit(main())
