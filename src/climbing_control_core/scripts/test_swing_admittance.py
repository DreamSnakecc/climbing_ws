#!/usr/bin/env python3
"""Staged swing-leg admittance test.

The test drives a single leg through the `swing_leg_controller` staged
sequence:

  1) PHASE_TEST_LIFT_CLEARANCE : lift +lift_m along the wall normal.
  2) PHASE_TEST_PRESS_CONTACT  : after lift, press downward by `press_m`.
     So the press target in normal-from-nominal is `(lift_m - press_m)`.
  3) PHASE_COMPLIANT_SETTLE    : normal-axis admittance control takes over.
  4) PHASE_ATTACHED_HOLD       : latched once attachment/adhesion asserts.

The test triggers are the existing `/swing_leg_controller/test_trigger_*`
parameters exposed by `swing_leg_controller.py`; this script simply sets
them, publishes a BodyReference marking the target leg as swing, and
drives the fan directly (bypassing mission_supervisor).

All relevant monitoring topics are subscribed; per-sample state goes to a
CSV under the workspace `test_logs/` directory (default) and a phase-level
summary is printed to stdout.

Preconditions
-------------
  * `jetson_bringup.launch` (dynamixel_bridge, leg_ik_executor,
    fan_serial_bridge, imu_serial_bridge, local_safety_supervisor) running.
  * `pc_static_bringup.launch` (state_estimator, swing_leg_controller,
    stance_force_optimizer, mission_supervisor held in INIT,
    body_planner held with mission_active=False so it publishes
    all-support). The test script then overrides support_mask for the
    single target leg.
  * No running instance of this test script.
"""

import argparse
import csv
import datetime
import math
import os
import sys
import threading
import time

import rospy
from climbing_msgs.msg import AdhesionCommand, BodyReference, EstimatedState, LegCenterCommand
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray


def _default_test_logs_dir():
    """`<workspace>/test_logs` (workspace = ancestor containing `src/climbing_control_core`)."""
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


LEG_NAMES = ["lf", "rf", "rr", "lr"]

PHASE_NAMES = {
    0: "SUPPORT",
    1: "TEST_LIFT_CLEARANCE",
    2: "TEST_PRESS_CONTACT",
    3: "DETACH_SLIDE",
    4: "TANGENTIAL_ALIGN",
    5: "PRELOAD_COMPRESS",
    6: "COMPLIANT_SETTLE",
    7: "ATTACHED_HOLD",
}

DIAG_FIELDS = [
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

FAN_MODE_RELEASE = 0
FAN_MODE_ATTACH = 1
FAN_MODE_BOOST = 2


def _safe_index(values, idx, default=0.0):
    try:
        if idx < len(values):
            return values[idx]
    except TypeError:
        return default
    return default


class SwingAdmittanceTest(object):
    def __init__(self, args):
        rospy.init_node("test_swing_admittance", anonymous=False)

        if args.leg not in LEG_NAMES:
            raise ValueError("leg %r not in %r" % (args.leg, LEG_NAMES))
        self.leg_name = args.leg
        self.leg_index = LEG_NAMES.index(self.leg_name)

        self.lift_m = float(abs(args.lift))
        self.press_m = float(abs(args.press))
        self.press_target_from_nominal_m = float(self.lift_m - self.press_m)
        self.fan_rpm = float(args.fan_rpm)
        self.fan_mode = int(args.fan_mode)
        self.hold_adhesion_s = float(args.hold_adhesion_s)
        self.test_timeout_s = float(args.test_timeout_s)
        self.control_rate_hz = max(1.0, float(args.control_rate_hz))
        self.log_rate_hz = max(1.0, float(args.log_rate_hz))
        self.adhesion_rate_hz = max(1.0, float(args.adhesion_rate_hz))
        self.fan_on_phase_id = int(args.fan_on_phase_id)
        self.pause_after_lift = bool(args.pause_after_lift)
        self.pre_boost_duration_s = max(0.0, float(args.pre_boost_duration_s))
        self.verbose_status = bool(args.verbose_status)

        nominal_z_mm = rospy.get_param(
            "/gait_controller/nominal_universal_joint_center_z",
            -191.5,
        )
        self.nominal_z_m = float(nominal_z_mm) / 1000.0
        rospy.loginfo(
            "[test_swing_admittance] leg=%s nominal_z=%.4f m lift=%.3f press=%.3f press_target_from_nominal=%.3f fan_rpm=%.0f",
            self.leg_name,
            self.nominal_z_m,
            self.lift_m,
            self.press_m,
            self.press_target_from_nominal_m,
            self.fan_rpm,
        )

        self._state_lock = threading.Lock()
        self.latest_estimated = None
        self.latest_joint_state = None
        self.latest_fan_currents = None
        self.latest_leg_rpm = None
        self.latest_leg_command = None
        self.latest_diag_vector = None
        self.latest_diag_stamp = None
        self.latest_safe_mode = False

        self.test_phase = "IDLE"
        self._lift_pause_done = False
        self.test_started_at = None
        self.fan_commanded = False
        self.highlights = {
            "phase_enter_time": {},
            "phase_exit_time": {},
            "first_wall_touch_time": None,
            "first_measured_contact_time": None,
            "first_attachment_ready_time": None,
            "first_adhesion_time": None,
            "first_fan_command_time": None,
            "max_skirt_compression": 0.0,
            "max_compliant_force_n": 0.0,
            "max_compliant_offset_m": 0.0,
            "max_estimated_leg_normal_force_n": 0.0,
            "max_fan_current_a": 0.0,
            "max_cmd_normal_from_nominal_m": 0.0,
            "min_cmd_normal_from_nominal_m": 0.0,
            "max_normal_force_reading_n": 0.0,
        }
        self.samples = []
        self.last_phase_id_seen = None

        self._setup_pubs_subs()
        self._apply_controller_trigger_params(enable=True)

    # ---------- ROS plumbing ----------

    def _setup_pubs_subs(self):
        self.body_ref_pub = rospy.Publisher(
            "/control/body_reference", BodyReference, queue_size=10
        )
        self.adhesion_pub = rospy.Publisher(
            "/jetson/fan_serial_bridge/adhesion_command", AdhesionCommand, queue_size=20
        )

        rospy.Subscriber(
            "/state/estimated", EstimatedState, self._estimated_cb, queue_size=5
        )
        rospy.Subscriber(
            "/jetson/dynamixel_bridge/joint_state", JointState, self._joint_state_cb, queue_size=5
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/fan_currents",
            Float32MultiArray,
            self._fan_current_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/leg_rpm",
            Float32MultiArray,
            self._leg_rpm_cb,
            queue_size=5,
        )
        rospy.Subscriber(
            "/control/swing_leg_target",
            LegCenterCommand,
            self._leg_target_cb,
            queue_size=50,
        )
        rospy.Subscriber(
            "/control/swing_leg_diag/" + self.leg_name,
            Float32MultiArray,
            self._diag_cb,
            queue_size=50,
        )
        rospy.Subscriber(
            "/jetson/local_safety_supervisor/safe_mode",
            Bool,
            self._safe_mode_cb,
            queue_size=5,
        )

    def _estimated_cb(self, msg):
        with self._state_lock:
            self.latest_estimated = msg

    def _joint_state_cb(self, msg):
        with self._state_lock:
            self.latest_joint_state = msg

    def _fan_current_cb(self, msg):
        with self._state_lock:
            self.latest_fan_currents = list(msg.data) if msg.data else []

    def _leg_rpm_cb(self, msg):
        with self._state_lock:
            self.latest_leg_rpm = list(msg.data) if msg.data else []

    def _leg_target_cb(self, msg):
        if str(msg.leg_name) != self.leg_name:
            return
        with self._state_lock:
            self.latest_leg_command = msg

    def _diag_cb(self, msg):
        data = list(msg.data) if msg.data else []
        with self._state_lock:
            self.latest_diag_vector = data
            self.latest_diag_stamp = rospy.Time.now()

    def _safe_mode_cb(self, msg):
        with self._state_lock:
            self.latest_safe_mode = bool(msg.data)

    # ---------- Controller trigger ----------

    def _apply_controller_trigger_params(self, enable):
        if enable:
            rospy.set_param("/swing_leg_controller/test_trigger_leg_name", self.leg_name)
            rospy.set_param(
                "/swing_leg_controller/test_trigger_normal_travel_m", float(self.lift_m)
            )
            rospy.set_param(
                "/swing_leg_controller/test_trigger_press_normal_travel_m",
                float(self.press_target_from_nominal_m),
            )
        else:
            rospy.set_param("/swing_leg_controller/test_trigger_leg_name", "")
            rospy.set_param("/swing_leg_controller/test_trigger_normal_travel_m", 0.0)
            rospy.set_param(
                "/swing_leg_controller/test_trigger_press_normal_travel_m", -0.003
            )

    # ---------- Publishers ----------

    def _build_body_reference(self, target_in_swing):
        msg = BodyReference()
        msg.header.stamp = rospy.Time.now()
        msg.pose = Pose()
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        msg.twist = Twist()
        msg.gait_mode = 0
        support_mask = [True] * 4
        if target_in_swing:
            support_mask[self.leg_index] = False
        msg.support_mask = support_mask
        return msg

    def _publish_adhesion(self, mode, rpm):
        msg = AdhesionCommand()
        msg.header.stamp = rospy.Time.now()
        msg.leg_index = self.leg_index
        msg.mode = int(mode)
        msg.target_rpm = float(rpm)
        msg.normal_force_limit = 25.0
        msg.required_adhesion_force = 65.0
        self.adhesion_pub.publish(msg)

    def _release_all_fans(self):
        for idx in range(len(LEG_NAMES)):
            msg = AdhesionCommand()
            msg.header.stamp = rospy.Time.now()
            msg.leg_index = idx
            msg.mode = FAN_MODE_RELEASE
            msg.target_rpm = 0.0
            msg.normal_force_limit = 0.0
            msg.required_adhesion_force = 0.0
            self.adhesion_pub.publish(msg)

    # ---------- Snapshotting ----------

    def _get_diag_field(self, name):
        with self._state_lock:
            diag = list(self.latest_diag_vector) if self.latest_diag_vector else None
        if diag is None:
            return None
        try:
            idx = DIAG_FIELDS.index(name)
        except ValueError:
            return None
        if idx >= len(diag):
            return None
        return float(diag[idx])

    def _snapshot(self, wall_time):
        with self._state_lock:
            estimated = self.latest_estimated
            joint_state = self.latest_joint_state
            fan_currents = list(self.latest_fan_currents) if self.latest_fan_currents else []
            leg_rpm = list(self.latest_leg_rpm) if self.latest_leg_rpm else []
            leg_command = self.latest_leg_command
            diag_vector = list(self.latest_diag_vector) if self.latest_diag_vector else []
            safe_mode = self.latest_safe_mode

        leg_idx = self.leg_index
        row = {
            "wall_time": wall_time,
            "ros_time": rospy.get_time(),
            "test_phase": self.test_phase,
        }

        for field_index, field_name in enumerate(DIAG_FIELDS):
            row[field_name] = (
                float(diag_vector[field_index]) if field_index < len(diag_vector) else None
            )
        phase_id = row.get("phase_id")
        row["phase_name"] = (
            PHASE_NAMES.get(int(phase_id), "UNKNOWN") if phase_id is not None else None
        )

        if leg_command is not None:
            row["cmd_center_x"] = float(leg_command.center.x)
            row["cmd_center_y"] = float(leg_command.center.y)
            row["cmd_center_z"] = float(leg_command.center.z)
            row["cmd_center_velocity_z"] = float(leg_command.center_velocity.z)
            row["cmd_skirt_compression_target"] = float(leg_command.skirt_compression_target)
            row["cmd_normal_force_limit"] = float(leg_command.desired_normal_force_limit)
            row["cmd_support_leg"] = bool(leg_command.support_leg)
        else:
            row["cmd_center_x"] = None
            row["cmd_center_y"] = None
            row["cmd_center_z"] = None
            row["cmd_center_velocity_z"] = None
            row["cmd_skirt_compression_target"] = None
            row["cmd_normal_force_limit"] = None
            row["cmd_support_leg"] = None

        row["skirt_compression_est"] = None
        row["leg_fan_current_est"] = None
        row["slip_risk"] = None
        row["contact_confidence"] = None
        row["seal_confidence"] = None
        row["leg_torque_sum"] = None
        row["wall_touch_mask"] = None
        row["measured_contact_mask"] = None
        row["early_contact_mask"] = None
        row["contact_mask"] = None
        row["attachment_ready_mask"] = None
        row["adhesion_mask"] = None
        row["ujc_x"] = None
        row["ujc_y"] = None
        row["ujc_z"] = None
        if estimated is not None:
            row["skirt_compression_est"] = _safe_index(estimated.skirt_compression_estimate, leg_idx)
            row["leg_fan_current_est"] = _safe_index(estimated.fan_current, leg_idx)
            row["slip_risk"] = _safe_index(estimated.slip_risk, leg_idx)
            row["contact_confidence"] = _safe_index(estimated.contact_confidence, leg_idx)
            row["seal_confidence"] = _safe_index(estimated.seal_confidence, leg_idx)
            row["leg_torque_sum"] = _safe_index(estimated.leg_torque_sum, leg_idx)
            row["wall_touch_mask"] = bool(_safe_index(estimated.wall_touch_mask, leg_idx, False))
            row["measured_contact_mask"] = bool(
                _safe_index(estimated.measured_contact_mask, leg_idx, False)
            )
            row["early_contact_mask"] = bool(_safe_index(estimated.early_contact_mask, leg_idx, False))
            row["contact_mask"] = bool(_safe_index(estimated.contact_mask, leg_idx, False))
            row["attachment_ready_mask"] = bool(
                _safe_index(estimated.attachment_ready_mask, leg_idx, False)
            )
            row["adhesion_mask"] = bool(_safe_index(estimated.adhesion_mask, leg_idx, False))
            if leg_idx < len(estimated.universal_joint_center_positions):
                ujc = estimated.universal_joint_center_positions[leg_idx]
                row["ujc_x"] = float(ujc.x)
                row["ujc_y"] = float(ujc.y)
                row["ujc_z"] = float(ujc.z)

        row["fan_current_a"] = (
            float(fan_currents[leg_idx]) if leg_idx < len(fan_currents) else None
        )
        row["fan_rpm_fb"] = float(leg_rpm[leg_idx]) if leg_idx < len(leg_rpm) else None

        row["joint_positions"] = []
        row["joint_velocities"] = []
        row["joint_currents"] = []
        if joint_state is not None:
            motor_ids = rospy.get_param("/legs/%s/motor_ids" % self.leg_name, [])
            joint_name_map = {
                str(int(name)): index for index, name in enumerate(joint_state.name)
            }
            for motor_id in motor_ids:
                idx = joint_name_map.get(str(int(motor_id)))
                if idx is None:
                    continue
                if idx < len(joint_state.position):
                    row["joint_positions"].append(float(joint_state.position[idx]))
                if idx < len(joint_state.velocity):
                    row["joint_velocities"].append(float(joint_state.velocity[idx]))
                if idx < len(joint_state.effort):
                    row["joint_currents"].append(float(joint_state.effort[idx]))
        row["safe_mode"] = bool(safe_mode)
        return row

    def _update_highlights(self, row):
        phase_id = row.get("phase_id")
        now_sec = row["wall_time"]
        if phase_id is not None:
            phase_id = int(phase_id)
            if phase_id != self.last_phase_id_seen:
                self.highlights["phase_enter_time"].setdefault(phase_id, now_sec)
                if self.last_phase_id_seen is not None:
                    self.highlights["phase_exit_time"][self.last_phase_id_seen] = now_sec
                self.last_phase_id_seen = phase_id
        if row.get("wall_touch_mask") and self.highlights["first_wall_touch_time"] is None:
            self.highlights["first_wall_touch_time"] = now_sec
        if (
            row.get("measured_contact_mask")
            and self.highlights["first_measured_contact_time"] is None
        ):
            self.highlights["first_measured_contact_time"] = now_sec
        if (
            row.get("attachment_ready_mask")
            and self.highlights["first_attachment_ready_time"] is None
        ):
            self.highlights["first_attachment_ready_time"] = now_sec
        if row.get("adhesion_mask") and self.highlights["first_adhesion_time"] is None:
            self.highlights["first_adhesion_time"] = now_sec
        skirt = row.get("skirt_compression_est")
        if skirt is not None:
            self.highlights["max_skirt_compression"] = max(
                self.highlights["max_skirt_compression"], float(skirt)
            )
        force = row.get("compliant_force_estimate_n")
        if force is not None:
            self.highlights["max_compliant_force_n"] = max(
                self.highlights["max_compliant_force_n"], float(force)
            )
        offset = row.get("compliant_normal_offset_m")
        if offset is not None:
            self.highlights["max_compliant_offset_m"] = max(
                self.highlights["max_compliant_offset_m"], float(offset)
            )
        est_force = row.get("estimated_leg_normal_force_n")
        if est_force is not None:
            self.highlights["max_estimated_leg_normal_force_n"] = max(
                self.highlights["max_estimated_leg_normal_force_n"], float(est_force)
            )
        fan_current = row.get("fan_current_a")
        if fan_current is not None:
            self.highlights["max_fan_current_a"] = max(
                self.highlights["max_fan_current_a"], float(fan_current)
            )
        cmd_normal = row.get("cmd_normal_from_nominal_m")
        if cmd_normal is not None:
            self.highlights["max_cmd_normal_from_nominal_m"] = max(
                self.highlights["max_cmd_normal_from_nominal_m"], float(cmd_normal)
            )
            self.highlights["min_cmd_normal_from_nominal_m"] = min(
                self.highlights["min_cmd_normal_from_nominal_m"], float(cmd_normal)
            )

    # ---------- Test sequencing ----------

    def _wait_for_streams(self, timeout_s=5.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline and not rospy.is_shutdown():
            with self._state_lock:
                have_est = self.latest_estimated is not None
                have_js = self.latest_joint_state is not None
            if have_est and have_js:
                return True
            time.sleep(0.05)
        rospy.logwarn(
            "[test_swing_admittance] streams not ready after %.1fs (estimated=%s joint_state=%s)",
            timeout_s,
            have_est,
            have_js,
        )
        return False

    def _hold_support_mask(self, duration_s):
        end_time = time.time() + duration_s
        rate = rospy.Rate(self.control_rate_hz)
        while time.time() < end_time and not rospy.is_shutdown():
            self.body_ref_pub.publish(self._build_body_reference(target_in_swing=False))
            rate.sleep()

    def _drive_adhesion_loop(self, duration_s, mode, rpm):
        end_time = time.time() + duration_s
        rate = rospy.Rate(self.adhesion_rate_hz)
        if self.highlights["first_fan_command_time"] is None and rpm > 0:
            self.highlights["first_fan_command_time"] = time.time() - self.test_started_at
        while time.time() < end_time and not rospy.is_shutdown():
            self._publish_adhesion(mode, rpm)
            rate.sleep()

    def _print_live_status(self, row):
        if not self.verbose_status:
            return
        phase_name = row.get("phase_name") or "-"
        cmd_z = row.get("cmd_center_z")
        force = row.get("compliant_force_estimate_n")
        offset = row.get("compliant_normal_offset_m")
        est_force = row.get("estimated_leg_normal_force_n")
        skirt = row.get("skirt_compression_est")
        fan_current = row.get("fan_current_a")
        masks = "wt=%d mc=%d att=%d adh=%d" % (
            int(bool(row.get("wall_touch_mask"))),
            int(bool(row.get("measured_contact_mask"))),
            int(bool(row.get("attachment_ready_mask"))),
            int(bool(row.get("adhesion_mask"))),
        )
        rospy.loginfo_throttle(
            0.5,
            "[%s] phase=%s cmd_z=%s f_filt=%s est_f=%s offset=%s skirt=%s fan_i=%s | %s",
            self.test_phase,
            phase_name,
            _fmt(cmd_z, "%.4f"),
            _fmt(force, "%.2f"),
            _fmt(est_force, "%.2f"),
            _fmt(offset, "%.4f"),
            _fmt(skirt, "%.2f"),
            _fmt(fan_current, "%.2f"),
            masks,
        )

    def run(self):
        try:
            self._run_inner()
        finally:
            self._shutdown_cleanup()

    def _run_inner(self):
        rospy.sleep(0.5)
        self._release_all_fans()
        rospy.loginfo("[test_swing_admittance] waiting for sensor streams...")
        self._wait_for_streams(timeout_s=5.0)

        rospy.loginfo(
            "[test_swing_admittance] pinning all legs as support for %.2fs to settle...",
            1.0,
        )
        self.test_phase = "SETTLE"
        self._hold_support_mask(1.0)

        rospy.loginfo(
            "[test_swing_admittance] starting staged test: lift=%.3f m  press=%.3f m  press_target_from_nominal=%.3f m",
            self.lift_m,
            self.press_m,
            self.press_target_from_nominal_m,
        )
        self.test_phase = "TRIGGER_SWING"
        self.test_started_at = time.time()
        logger_thread = threading.Thread(target=self._log_loop, name="log_loop")
        logger_thread.daemon = True
        logger_thread.start()
        status_thread = threading.Thread(target=self._status_loop, name="status_loop")
        status_thread.daemon = True
        status_thread.start()

        control_rate = rospy.Rate(self.control_rate_hz)
        adhesion_rate_dt = 1.0 / self.adhesion_rate_hz
        last_adhesion_pub = 0.0

        start_wall = time.time()
        deadline = start_wall + self.test_timeout_s
        last_phase_id = None
        hold_after_adhesion_until = None

        while not rospy.is_shutdown() and time.time() < deadline:
            self.body_ref_pub.publish(self._build_body_reference(target_in_swing=True))

            phase_id = self._get_diag_field("phase_id")
            now = time.time()
            phase_id_int = int(phase_id) if phase_id is not None else None

            if phase_id_int is not None and phase_id_int != last_phase_id:
                rospy.loginfo(
                    "[test_swing_admittance] phase transition %s -> %s (elapsed=%.3fs)",
                    PHASE_NAMES.get(last_phase_id, str(last_phase_id)),
                    PHASE_NAMES.get(phase_id_int, str(phase_id_int)),
                    now - start_wall,
                )
                last_phase_id = phase_id_int
                if phase_id_int == 1:
                    self.test_phase = "LIFT"
                elif phase_id_int == 2:
                    if self.pause_after_lift and not self._lift_pause_done:
                        self.test_phase = "PAUSE_AFTER_LIFT"
                        rospy.loginfo(
                            "[test_swing_admittance] lift complete; press Enter to continue PRESS/admittance flow..."
                        )
                        self._wait_for_enter()
                        self._lift_pause_done = True
                    self.test_phase = "PRESS"
                elif phase_id_int == 6:
                    self.test_phase = "COMPLIANT_SETTLE"
                elif phase_id_int == 7:
                    self.test_phase = "ATTACHED_HOLD"

            should_boost_fan = (
                phase_id_int is not None and phase_id_int >= self.fan_on_phase_id
            )
            if should_boost_fan:
                if not self.fan_commanded:
                    rospy.loginfo(
                        "[test_swing_admittance] entering fan boost mode=%d rpm=%.0f at phase=%s",
                        self.fan_mode,
                        self.fan_rpm,
                        PHASE_NAMES.get(phase_id_int, str(phase_id_int)),
                    )
                if self.highlights["first_fan_command_time"] is None:
                    self.highlights["first_fan_command_time"] = now - start_wall
                self.fan_commanded = True
                if now - last_adhesion_pub >= adhesion_rate_dt:
                    self._publish_adhesion(self.fan_mode, self.fan_rpm)
                    last_adhesion_pub = now
            else:
                if now - last_adhesion_pub >= adhesion_rate_dt:
                    self._publish_adhesion(FAN_MODE_RELEASE, 0.0)
                    last_adhesion_pub = now

            adhesion_mask = False
            attachment_mask = False
            with self._state_lock:
                est = self.latest_estimated
            if est is not None:
                adhesion_mask = bool(_safe_index(est.adhesion_mask, self.leg_index, False))
                attachment_mask = bool(
                    _safe_index(est.attachment_ready_mask, self.leg_index, False)
                )
            if (adhesion_mask or attachment_mask) and hold_after_adhesion_until is None:
                hold_after_adhesion_until = now + self.hold_adhesion_s
                rospy.loginfo(
                    "[test_swing_admittance] adhesion/attachment latched; holding for %.2fs",
                    self.hold_adhesion_s,
                )
            if hold_after_adhesion_until is not None and now >= hold_after_adhesion_until:
                rospy.loginfo(
                    "[test_swing_admittance] hold complete; exiting test loop"
                )
                break

            control_rate.sleep()

        rospy.loginfo("[test_swing_admittance] test window ended")
        self.test_phase = "TERMINATE"

    @staticmethod
    def _wait_for_enter():
        try:
            input("")
        except EOFError:
            rospy.logwarn("[test_swing_admittance] no stdin available; continue without Enter pause")

    def _log_loop(self):
        rate = rospy.Rate(self.log_rate_hz)
        while not rospy.is_shutdown():
            if self.test_started_at is None:
                rate.sleep()
                continue
            wall_time = time.time() - self.test_started_at
            row = self._snapshot(wall_time)
            self.samples.append(row)
            self._update_highlights(row)
            if self.test_phase == "TERMINATE":
                break
            rate.sleep()

    def _status_loop(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            if self.test_started_at is None:
                rate.sleep()
                continue
            wall_time = time.time() - self.test_started_at
            row = self._snapshot(wall_time)
            self._print_live_status(row)
            if self.test_phase == "TERMINATE":
                break
            rate.sleep()

    def _shutdown_cleanup(self):
        rospy.loginfo("[test_swing_admittance] cleanup: releasing swing and fans...")
        try:
            self._apply_controller_trigger_params(enable=False)
        except Exception as exc:
            rospy.logwarn("failed to clear test_trigger params: %s", exc)
        for _ in range(20):
            if rospy.is_shutdown():
                break
            self.body_ref_pub.publish(self._build_body_reference(target_in_swing=False))
            self._release_all_fans()
            rospy.sleep(0.05)

    # ---------- Output ----------

    def save_log(self, output_dir):
        if not self.samples:
            rospy.logwarn("[test_swing_admittance] no samples captured; skip log")
            return None
        os.makedirs(output_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(
            output_dir,
            "swing_admittance_%s_%s.csv" % (self.leg_name, stamp),
        )
        fieldnames = [
            "wall_time",
            "ros_time",
            "test_phase",
            "phase_name",
        ] + DIAG_FIELDS + [
            "cmd_center_x",
            "cmd_center_y",
            "cmd_center_z",
            "cmd_center_velocity_z",
            "cmd_skirt_compression_target",
            "cmd_normal_force_limit",
            "cmd_support_leg",
            "skirt_compression_est",
            "leg_fan_current_est",
            "slip_risk",
            "contact_confidence",
            "seal_confidence",
            "leg_torque_sum",
            "wall_touch_mask",
            "measured_contact_mask",
            "early_contact_mask",
            "contact_mask",
            "attachment_ready_mask",
            "adhesion_mask",
            "ujc_x",
            "ujc_y",
            "ujc_z",
            "fan_current_a",
            "fan_rpm_fb",
            "joint_positions",
            "joint_velocities",
            "joint_currents",
            "safe_mode",
        ]
        with open(filename, "w") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames)
            writer.writeheader()
            for row in self.samples:
                serialized = {}
                for key in fieldnames:
                    value = row.get(key)
                    if isinstance(value, list):
                        serialized[key] = ";".join("%.6f" % float(v) for v in value)
                    elif value is None:
                        serialized[key] = ""
                    else:
                        serialized[key] = value
                writer.writerow(serialized)
        rospy.loginfo("[test_swing_admittance] log saved to %s (%d rows)", filename, len(self.samples))
        return filename

    def print_summary(self):
        print()
        print("=" * 76)
        print("Swing-leg admittance test summary - leg=%s" % self.leg_name)
        print("=" * 76)
        print("  lift target (normal from nominal) : %+.4f m" % self.lift_m)
        print("  press target (normal from nominal): %+.4f m" % (self.press_target_from_nominal_m))
        print(
            "  cmd normal reached  max=%+.4f  min=%+.4f m"
            % (
                self.highlights["max_cmd_normal_from_nominal_m"],
                self.highlights["min_cmd_normal_from_nominal_m"],
            )
        )
        print()

        def _fmt_time(value):
            return "%.3fs" % value if value is not None else "not reached"

        def _duration(phase_id):
            enter = self.highlights["phase_enter_time"].get(phase_id)
            exit_t = self.highlights["phase_exit_time"].get(phase_id)
            if enter is None:
                return "not entered"
            if exit_t is None:
                return "entered@%.3fs (not exited)" % enter
            return "entered@%.3fs duration=%.3fs" % (enter, exit_t - enter)

        for phase_id in sorted(PHASE_NAMES):
            name = PHASE_NAMES[phase_id]
            if phase_id in self.highlights["phase_enter_time"]:
                print("  Phase %d %-22s: %s" % (phase_id, name, _duration(phase_id)))

        print()
        print("  first wall_touch        : %s" % _fmt_time(self.highlights["first_wall_touch_time"]))
        print(
            "  first measured_contact  : %s"
            % _fmt_time(self.highlights["first_measured_contact_time"])
        )
        print(
            "  first attachment_ready  : %s"
            % _fmt_time(self.highlights["first_attachment_ready_time"])
        )
        print("  first adhesion_mask     : %s" % _fmt_time(self.highlights["first_adhesion_time"]))
        print("  first fan command       : %s" % _fmt_time(self.highlights["first_fan_command_time"]))

        print()
        print("  max skirt compression est         : %.3f" % self.highlights["max_skirt_compression"])
        print("  max compliant filtered force (N)  : %.2f" % self.highlights["max_compliant_force_n"])
        print("  max compliant normal offset (m)   : %.4f" % self.highlights["max_compliant_offset_m"])
        print(
            "  max estimated leg normal force (N): %.2f"
            % self.highlights["max_estimated_leg_normal_force_n"]
        )
        print("  max fan current (A)               : %.2f" % self.highlights["max_fan_current_a"])
        print("  max/min cmd normal from nominal (m): %+.4f / %+.4f"
            % (
                self.highlights["max_cmd_normal_from_nominal_m"],
                self.highlights["min_cmd_normal_from_nominal_m"],
            )
        )
        print()

        issues = self._diagnose_issues()
        if issues:
            print("  ATTENTION (probable issues / tuning hints):")
            for issue in issues:
                print("    - " + issue)
        else:
            print("  No obvious anomalies detected.")
        print("=" * 76)

    def _diagnose_issues(self):
        issues = []
        reached_lift = self.highlights["max_cmd_normal_from_nominal_m"] + 1e-4 >= self.lift_m
        reached_press = self.highlights["min_cmd_normal_from_nominal_m"] - 1e-4 <= self.press_target_from_nominal_m
        entered_press = 2 in self.highlights["phase_enter_time"]
        entered_settle = 6 in self.highlights["phase_enter_time"]
        entered_attached = 7 in self.highlights["phase_enter_time"]

        if not reached_lift:
            issues.append(
                "Lift target not fully reached; check test_lift_velocity_limit_mps, "
                "test_lift_dwell_s, max_position_offset_m[2], or swing trigger didn't start."
            )
        if not entered_press:
            issues.append(
                "TEST_PRESS_CONTACT not entered; leg may have dropped out of swing "
                "(check BodyReference support_mask publishing rate / body_planner interference)."
            )
        elif not reached_press:
            issues.append(
                "Press target not fully reached; check test_press_velocity_limit_mps, "
                "normal_alignment_tolerance_m, or clamp limits in max_position_offset_m[2]."
            )
        if entered_press and not entered_settle:
            issues.append(
                "COMPLIANT_SETTLE not entered; probably press target not aligned within "
                "normal_alignment_tolerance_m before test_press_dwell_s elapsed."
            )
        if entered_settle and self.highlights["first_measured_contact_time"] is None:
            issues.append(
                "COMPLIANT_SETTLE reached but no measured_contact seen; check "
                "support_force_threshold_n / torque_contact_threshold_nm / joint torque estimation."
            )
        if (
            entered_settle
            and self.highlights["first_measured_contact_time"] is not None
            and self.highlights["max_compliant_force_n"] < 1.0
        ):
            issues.append(
                "Filtered compliant force stays near zero; verify compliant_force_sign, "
                "compliant_force_deadband_n, and joint torque bias capture during press."
            )
        if self.highlights["max_compliant_force_n"] > 0.0 and self.highlights["max_compliant_offset_m"] < 1e-4:
            issues.append(
                "Admittance offset never grew despite measured force; reduce "
                "compliant_admittance_stiffness or increase compliant_normal_velocity_limit_mps."
            )
        if self.highlights["max_compliant_offset_m"] > 0.0 and not entered_attached:
            issues.append(
                "Admittance pushed in but ATTACHED_HOLD never triggered; check fan RPM, "
                "fan_adhesion_reference_rpm ranges, and fan_attached_current_*_a thresholds."
            )
        if self.highlights["max_fan_current_a"] < 0.1 and self.highlights["first_fan_command_time"] is not None:
            issues.append(
                "Fan RPM commanded but no current feedback; check fan_serial_bridge port, "
                "protocol setting, and telemetry frame format."
            )
        if entered_attached and self.highlights["max_skirt_compression"] < 0.3:
            issues.append(
                "Attached but skirt compression stayed low; check adhesion_skirt_compression_mm / "
                "adhesion_force_reference_n scaling in state_estimator."
            )
        if self.highlights["first_wall_touch_time"] is None and entered_settle:
            issues.append(
                "wall_touch_mask never asserted; verify early_contact_phase_threshold, "
                "skirt estimator, and whether the wall is actually being pressed."
            )
        return issues


def _fmt(value, spec):
    if value is None:
        return "None"
    try:
        return spec % float(value)
    except (TypeError, ValueError):
        return str(value)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--leg", default="lf", choices=LEG_NAMES, help="leg to test (default: lf)")
    parser.add_argument(
        "--lift",
        type=float,
        default=0.04,
        help="lift magnitude along wall-normal, positive value (default: 0.04m)",
    )
    parser.add_argument(
        "--press",
        type=float,
        default=0.045,
        help="downward press distance after lift; press target is (lift - press) in normal-from-nominal (default: 0.045m)",
    )
    parser.add_argument(
        "--fan-rpm",
        type=float,
        default=30000.0,
        help="fan RPM during press/admittance phases (default: 30000)",
    )
    parser.add_argument(
        "--fan-mode",
        type=int,
        default=FAN_MODE_BOOST,
        help="AdhesionCommand mode while fan is on: 0=release 1=attach 2=boost (default: 2)",
    )
    parser.add_argument(
        "--fan-on-phase-id",
        type=int,
        default=2,
        help="phase id at or after which the fan is commanded on (default: 2 = TEST_PRESS_CONTACT)",
    )
    parser.add_argument(
        "--pre-boost-duration-s",
        type=float,
        default=0.0,
        help="extra hold at target rpm before enabling admittance (unused placeholder)",
    )
    parser.add_argument(
        "--hold-adhesion-s",
        type=float,
        default=2.0,
        help="hold time after attachment/adhesion latch (default: 2.0s)",
    )
    parser.add_argument(
        "--no-pause-after-lift",
        action="store_false",
        dest="pause_after_lift",
        help="disable Enter pause after lift and continue immediately to press",
    )
    parser.set_defaults(pause_after_lift=True)
    parser.add_argument(
        "--test-timeout-s",
        type=float,
        default=20.0,
        help="overall safety timeout (default: 30s)",
    )
    parser.add_argument(
        "--control-rate-hz",
        type=float,
        default=100.0,
        help="rate at which BodyReference is published (default: 100Hz)",
    )
    parser.add_argument(
        "--log-rate-hz",
        type=float,
        default=50.0,
        help="rate at which samples are captured to the log (default: 50Hz)",
    )
    parser.add_argument(
        "--adhesion-rate-hz",
        type=float,
        default=20.0,
        help="rate at which AdhesionCommand is published while boosting (default: 20Hz)",
    )
    parser.add_argument(
        "--output-dir",
        default=_default_test_logs_dir(),
        help="where to write CSV logs (default: <workspace>/test_logs)",
    )
    parser.add_argument(
        "--verbose-status",
        action="store_true",
        help="print a throttled live status line while the test runs",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    tester = SwingAdmittanceTest(args)
    try:
        tester.run()
    except rospy.ROSInterruptException:
        pass
    tester.save_log(args.output_dir)
    tester.print_summary()


if __name__ == "__main__":
    main(sys.argv[1:] if len(sys.argv) > 1 else None)
