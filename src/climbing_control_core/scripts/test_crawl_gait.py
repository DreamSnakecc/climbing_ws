#!/usr/bin/env python3
"""Whole-body crawl-gait test.

This script drives the robot through a whole-body crawl gait and records
key system data. It is designed to verify that:

  * The crawl gait schedule emitted by ``body_planner`` swings each leg in
    order while the other three remain in support.
  * ``swing_leg_controller`` completes the full staged swing
    (DETACH_SLIDE -> TANGENTIAL_ALIGN -> PRELOAD_COMPRESS -> COMPLIANT_SETTLE
    -> ATTACHED_HOLD) for every leg.
  * ``mission_supervisor`` synchronizes each fan with the swing of the
    corresponding leg (release during detach/align, boost immediately once
    ``PRELOAD_COMPRESS`` completes / ``COMPLIANT_SETTLE`` begins, then attach
    once ``attachment_ready`` / adhesion asserts).

Key knobs (exposed as CLI options):

  * ``--ujc-z-mm`` (default **-80.0 mm**): overrides
    ``/gait_controller/nominal_universal_joint_center_z``. Note that this
    parameter is consumed at node startup; the script sets it but warns
    the user if the control stack must be relaunched to pick it up.
  * ``--test-duration-s``: total recording window after mission start.
  * ``--linear-velocity-mps`` / ``--angular-velocity-rps``: optional body
    velocity overrides pushed to ``/body_planner`` so the gait can walk
    (defaults keep the robot stepping in place).

Preconditions
-------------
  * ``jetson_bringup.launch`` already running (dynamixel_bridge,
    leg_ik_executor, fan_serial_bridge, imu_serial_bridge,
    local_safety_supervisor).
  * ``pc_bringup.launch`` running with
    ``mission_auto_start:=false`` and
    ``enable_auto_adhesion_commands:=true``. The script explicitly
    publishes ``/control/mission_start`` so the mission state machine
    transitions INIT -> STICK -> CLIMB.
  * All four feet are adhered to the wall (required for the STICK ->
    CLIMB transition inside ``mission_supervisor``).
  * ``swing_leg_controller`` has the "test trigger" overrides cleared so
    the swing phase runs the full staged sequence (not the test
    LIFT/PRESS override).
"""

import argparse
import csv
import datetime
import math
import os
import signal
import sys
import threading
import time

import rospy
from climbing_msgs.msg import (
    AdhesionCommand,
    BodyReference,
    EstimatedState,
    LegCenterCommand,
    StanceWrenchCommand,
)
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float32MultiArray, String


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

FAN_MODE_RELEASE = 0
FAN_MODE_ATTACH = 1
FAN_MODE_BOOST = 2
FAN_MODE_NAMES = {
    FAN_MODE_RELEASE: "RELEASE",
    FAN_MODE_ATTACH: "ATTACH",
    FAN_MODE_BOOST: "BOOST",
}

# Mirrors swing_leg_controller.diagnostic_field_labels ordering so the
# per-leg Float32MultiArray can be unpacked by name.
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
    "test_hold_at_lift",
    "test_lift_aligned",
    "leg_torque_sum_nm",
    "leg_current_sum_a",
    "press_torque_baseline_nm",
    "press_current_baseline_a",
    "press_torque_delta_nm",
    "press_current_delta_a",
    "press_contact_confirmed",
]


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


def _safe_index(values, idx, default=0.0):
    try:
        if idx < len(values):
            return values[idx]
    except TypeError:
        return default
    return default


def _aggregate_leg_scalar(motor_ids, values, joint_name_map):
    """Sum of absolute values over leg motors; mirrors state_estimator._aggregate_leg_scalar."""
    total = 0.0
    count = 0
    for motor_id in motor_ids:
        joint_index = joint_name_map.get(str(int(motor_id)))
        if joint_index is None or joint_index >= len(values):
            continue
        total += abs(float(values[joint_index]))
        count += 1
    return total, count


def _min_max_mean(values):
    if not values:
        return None
    return min(values), max(values), sum(values) / float(len(values))


class CrawlGaitTest(object):
    def __init__(self, args):
        rospy.init_node("test_crawl_gait", anonymous=False)

        self.ujc_z_mm = float(args.ujc_z_mm)
        self.test_duration_s = max(float(args.test_duration_s), 1.0)
        self.control_rate_hz = max(1.0, float(args.control_rate_hz))
        self.log_rate_hz = max(1.0, float(args.log_rate_hz))
        self.warmup_s = max(float(args.warmup_s), 0.0)
        self.cooldown_s = max(float(args.cooldown_s), 0.0)
        self.auto_release_fans_on_exit = bool(args.release_fans_on_exit)
        self.verbose_status = bool(args.verbose_status)
        self.output_dir = str(args.output_dir)
        self.linear_velocity_mps = [float(value) for value in args.linear_velocity_mps]
        self.angular_velocity_rps = [float(value) for value in args.angular_velocity_rps]
        if len(self.linear_velocity_mps) != 3:
            raise ValueError("--linear-velocity-mps expects 3 values (x y z)")
        if len(self.angular_velocity_rps) != 3:
            raise ValueError("--angular-velocity-rps expects 3 values (x y z)")

        self._apply_ujc_z_param(self.ujc_z_mm)
        self._apply_body_planner_velocity_params()
        self._apply_crawl_sync_params()
        self._clear_swing_test_triggers()

        # Snapshot what swing_leg_controller actually thinks nominal_z is (may differ from
        # the param if the node was launched before we updated the param).
        self.configured_ujc_z_m = float(
            rospy.get_param("/gait_controller/nominal_universal_joint_center_z", -195.5)
        ) / 1000.0

        self._state_lock = threading.Lock()
        self.latest_body_reference = None
        self.latest_estimated = None
        self.latest_joint_state = None
        self.latest_joint_currents = None
        self.latest_fan_currents = None
        self.latest_leg_rpm = None
        self.latest_leg_command = {name: None for name in LEG_NAMES}
        self.latest_leg_diag = {name: None for name in LEG_NAMES}
        self.latest_leg_diag_stamp = {name: None for name in LEG_NAMES}
        self.latest_leg_adhesion_cmd = {name: None for name in LEG_NAMES}
        self.latest_stance_wrench = {name: None for name in LEG_NAMES}
        self.latest_mission_state = None
        self.latest_mission_active = False
        self.latest_safe_mode = False

        self._leg_motor_ids = {
            name: [int(mid) for mid in rospy.get_param("/legs/%s/motor_ids" % name, [])]
            for name in LEG_NAMES
        }

        self.samples = []
        self.test_phase = "IDLE"
        self.test_started_at = None
        self._stop_requested = False
        self._mission_state_history = []  # list of (t_rel, state)
        self._fan_cmd_history = {name: [] for name in LEG_NAMES}  # list of (t_rel, mode, rpm)
        self._swing_events = {name: [] for name in LEG_NAMES}  # per-leg swing-cycle summaries

        self._last_support_leg = {name: None for name in LEG_NAMES}
        self._current_swing_event = {name: None for name in LEG_NAMES}

        self._setup_pubs_subs()
        rospy.on_shutdown(self._handle_ros_shutdown)
        signal.signal(signal.SIGINT, self._handle_sigint)

    # ---------- Param setup ----------

    def _apply_ujc_z_param(self, ujc_z_mm):
        current_mm = float(
            rospy.get_param(
                "/gait_controller/nominal_universal_joint_center_z", -195.5
            )
        )
        rospy.set_param(
            "/gait_controller/nominal_universal_joint_center_z",
            float(ujc_z_mm),
        )
        rospy.loginfo(
            "[test_crawl_gait] set /gait_controller/nominal_universal_joint_center_z: "
            "%.3f mm -> %.3f mm",
            current_mm,
            float(ujc_z_mm),
        )
        if abs(current_mm - float(ujc_z_mm)) > 1e-3:
            rospy.logwarn(
                "[test_crawl_gait] UJC z param changed at runtime. Already-running nodes "
                "(swing_leg_controller / state_estimator / body_planner) read this value "
                "only at init. Relaunch the control stack so the new UJC z is in effect."
            )

    def _apply_body_planner_velocity_params(self):
        """Push requested crawl progression into body_planner params.

        body_planner reads /body_planner/linear_velocity_world_mps and
        /body_planner/angular_velocity_world_rps at init. Setting them here only
        helps if the planner is (re)started after we set the params. For a stack
        that is already running, these are effectively documented-in-the-log
        hints; the planner will keep whatever values it captured at startup.
        """
        rospy.set_param(
            "/body_planner/linear_velocity_world_mps",
            [float(v) for v in self.linear_velocity_mps],
        )
        rospy.set_param(
            "/body_planner/angular_velocity_world_rps",
            [float(v) for v in self.angular_velocity_rps],
        )
        rospy.loginfo(
            "[test_crawl_gait] set body_planner velocity overrides (requires planner restart "
            "to take effect): linear=%s  angular=%s",
            self.linear_velocity_mps,
            self.angular_velocity_rps,
        )

    def _apply_crawl_sync_params(self):
        """Align crawl-gait fan timing with the single-leg admittance test.

        The relevant nodes read these params at init, so runtime updates only
        affect a newly launched mission_supervisor / swing_leg_controller.
        """
        current_boost_after_preload = bool(
            rospy.get_param(
                "/mission_supervisor/boost_after_preload_without_contact",
                False,
            )
        )
        current_compliant_requires_contact = bool(
            rospy.get_param(
                "/swing_leg_controller/compliant_start_requires_contact",
                False,
            )
        )

        rospy.set_param(
            "/mission_supervisor/boost_after_preload_without_contact",
            True,
        )
        rospy.set_param(
            "/swing_leg_controller/compliant_start_requires_contact",
            False,
        )
        rospy.loginfo(
            "[test_crawl_gait] set crawl sync params (requires node restart to take effect): "
            "boost_after_preload_without_contact=True  "
            "compliant_start_requires_contact=False"
        )
        if (not current_boost_after_preload) or current_compliant_requires_contact:
            rospy.logwarn(
                "[test_crawl_gait] crawl fan/admittance sync params changed at runtime. "
                "Already-running mission_supervisor / swing_leg_controller will keep their "
                "startup values until the control stack is relaunched."
            )

    def _clear_swing_test_triggers(self):
        """Ensure the swing-leg staged-test override is disabled.

        The crawl gait requires the normal DETACH->...->ATTACHED swing sequence,
        not the LIFT/PRESS test override used by test_swing_admittance.py.
        """
        rospy.set_param("/swing_leg_controller/test_force_support_reset_enable", False)
        rospy.set_param("/swing_leg_controller/test_trigger_leg_name", "")
        rospy.set_param("/swing_leg_controller/test_trigger_normal_travel_m", 0.0)
        rospy.set_param("/swing_leg_controller/test_trigger_press_normal_travel_m", -0.003)
        rospy.set_param("/swing_leg_controller/test_trigger_hold_at_lift", False)

    # ---------- ROS plumbing ----------

    def _setup_pubs_subs(self):
        self.mission_start_pub = rospy.Publisher(
            "/control/mission_start", Bool, queue_size=5, latch=True
        )
        self.mission_pause_pub = rospy.Publisher(
            "/control/mission_pause", Bool, queue_size=5, latch=True
        )
        self.adhesion_pub = rospy.Publisher(
            "/jetson/fan_serial_bridge/adhesion_command", AdhesionCommand, queue_size=20
        )

        rospy.Subscriber(
            "/control/body_reference", BodyReference, self._body_reference_cb, queue_size=20
        )
        rospy.Subscriber(
            "/state/estimated", EstimatedState, self._estimated_cb, queue_size=20
        )
        rospy.Subscriber(
            "/jetson/dynamixel_bridge/joint_state", JointState, self._joint_state_cb, queue_size=20
        )
        rospy.Subscriber(
            "/jetson/dynamixel_bridge/joint_currents",
            Float32MultiArray,
            self._joint_currents_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/fan_currents",
            Float32MultiArray,
            self._fan_current_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/leg_rpm",
            Float32MultiArray,
            self._leg_rpm_cb,
            queue_size=20,
        )
        rospy.Subscriber(
            "/jetson/fan_serial_bridge/adhesion_command",
            AdhesionCommand,
            self._adhesion_cmd_cb,
            queue_size=40,
        )
        rospy.Subscriber(
            "/control/swing_leg_target",
            LegCenterCommand,
            self._leg_target_cb,
            queue_size=200,
        )
        for leg_name in LEG_NAMES:
            rospy.Subscriber(
                "/control/swing_leg_diag/" + leg_name,
                Float32MultiArray,
                self._diag_cb_factory(leg_name),
                queue_size=50,
            )
            rospy.Subscriber(
                "/control/stance_wrench/" + leg_name,
                StanceWrenchCommand,
                self._stance_wrench_cb_factory(leg_name),
                queue_size=20,
            )
        rospy.Subscriber(
            "/control/mission_state", String, self._mission_state_cb, queue_size=10
        )
        rospy.Subscriber(
            "/control/mission_active", Bool, self._mission_active_cb, queue_size=10
        )
        rospy.Subscriber(
            "/jetson/local_safety_supervisor/safe_mode",
            Bool,
            self._safe_mode_cb,
            queue_size=10,
        )

    def _body_reference_cb(self, msg):
        with self._state_lock:
            self.latest_body_reference = msg

    def _estimated_cb(self, msg):
        with self._state_lock:
            self.latest_estimated = msg

    def _joint_state_cb(self, msg):
        with self._state_lock:
            self.latest_joint_state = msg

    def _joint_currents_cb(self, msg):
        with self._state_lock:
            self.latest_joint_currents = list(msg.data) if msg.data else []

    def _fan_current_cb(self, msg):
        with self._state_lock:
            self.latest_fan_currents = list(msg.data) if msg.data else []

    def _leg_rpm_cb(self, msg):
        with self._state_lock:
            self.latest_leg_rpm = list(msg.data) if msg.data else []

    def _adhesion_cmd_cb(self, msg):
        try:
            leg_index = int(msg.leg_index)
        except Exception:
            return
        if leg_index < 0 or leg_index >= len(LEG_NAMES):
            return
        leg_name = LEG_NAMES[leg_index]
        with self._state_lock:
            self.latest_leg_adhesion_cmd[leg_name] = msg
            if self.test_started_at is not None:
                self._fan_cmd_history[leg_name].append(
                    (
                        time.time() - self.test_started_at,
                        int(msg.mode),
                        float(msg.target_rpm),
                    )
                )

    def _leg_target_cb(self, msg):
        name = str(msg.leg_name)
        if name not in self.latest_leg_command:
            return
        with self._state_lock:
            self.latest_leg_command[name] = msg

    def _diag_cb_factory(self, leg_name):
        def _cb(msg):
            data = list(msg.data) if msg.data else []
            with self._state_lock:
                self.latest_leg_diag[leg_name] = data
                self.latest_leg_diag_stamp[leg_name] = rospy.Time.now()
        return _cb

    def _stance_wrench_cb_factory(self, leg_name):
        def _cb(msg):
            with self._state_lock:
                self.latest_stance_wrench[leg_name] = msg
        return _cb

    def _mission_state_cb(self, msg):
        value = str(msg.data)
        with self._state_lock:
            previous = self.latest_mission_state
            self.latest_mission_state = value
            if previous != value:
                if self.test_started_at is not None:
                    self._mission_state_history.append(
                        (time.time() - self.test_started_at, value)
                    )

    def _mission_active_cb(self, msg):
        with self._state_lock:
            self.latest_mission_active = bool(msg.data)

    def _safe_mode_cb(self, msg):
        with self._state_lock:
            self.latest_safe_mode = bool(msg.data)

    # ---------- Signal & shutdown ----------

    def _handle_sigint(self, signum, frame):
        del signum, frame
        rospy.loginfo(
            "[test_crawl_gait] SIGINT received, flagging stop_requested and letting the "
            "main loop wind down cleanly (will pause mission and save CSV)."
        )
        self._stop_requested = True

    def _handle_ros_shutdown(self):
        self._stop_requested = True

    # ---------- Helpers ----------

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

    def _wait_for_streams(self, timeout_s=8.0):
        deadline = time.time() + timeout_s
        while time.time() < deadline and not rospy.is_shutdown():
            with self._state_lock:
                have_body_ref = self.latest_body_reference is not None
                have_est = self.latest_estimated is not None
                have_js = self.latest_joint_state is not None
                have_diag = any(v is not None for v in self.latest_leg_diag.values())
            if have_body_ref and have_est and have_js and have_diag:
                return True
            time.sleep(0.05)
        with self._state_lock:
            body_ref = self.latest_body_reference is not None
            est = self.latest_estimated is not None
            js = self.latest_joint_state is not None
            diag = {k: (v is not None) for k, v in self.latest_leg_diag.items()}
        rospy.logwarn(
            "[test_crawl_gait] required streams not all ready after %.1fs "
            "(body_reference=%s estimated=%s joint_state=%s diag=%s). "
            "Proceeding anyway; CSV columns may be blank where streams are missing.",
            timeout_s,
            body_ref,
            est,
            js,
            diag,
        )
        return False

    def _publish_mission_start(self):
        msg = Bool(data=True)
        for _ in range(5):
            self.mission_start_pub.publish(msg)
            time.sleep(0.05)
        # Also make sure mission_pause is cleared.
        self.mission_pause_pub.publish(Bool(data=False))

    def _publish_mission_pause(self):
        msg = Bool(data=True)
        for _ in range(5):
            self.mission_pause_pub.publish(msg)
            time.sleep(0.05)

    # ---------- Snapshot ----------

    def _snapshot(self, wall_time):
        with self._state_lock:
            body_ref = self.latest_body_reference
            estimated = self.latest_estimated
            joint_state = self.latest_joint_state
            joint_currents_vec = (
                list(self.latest_joint_currents)
                if self.latest_joint_currents is not None
                else None
            )
            fan_currents = list(self.latest_fan_currents) if self.latest_fan_currents else []
            leg_rpm = list(self.latest_leg_rpm) if self.latest_leg_rpm else []
            per_leg_cmd = dict(self.latest_leg_command)
            per_leg_diag = {k: list(v) if v else [] for k, v in self.latest_leg_diag.items()}
            per_leg_fan_cmd = dict(self.latest_leg_adhesion_cmd)
            per_leg_stance = dict(self.latest_stance_wrench)
            mission_state = self.latest_mission_state
            mission_active = self.latest_mission_active
            safe_mode = self.latest_safe_mode

        row = {
            "wall_time": wall_time,
            "ros_time": rospy.get_time(),
            "test_phase": self.test_phase,
            "mission_state": mission_state if mission_state is not None else "",
            "mission_active": bool(mission_active),
            "safe_mode": bool(safe_mode),
        }

        # ---------- body reference ----------
        if body_ref is not None:
            row["body_ref_pose_x"] = float(body_ref.pose.position.x)
            row["body_ref_pose_y"] = float(body_ref.pose.position.y)
            row["body_ref_pose_z"] = float(body_ref.pose.position.z)
            row["body_ref_quat_x"] = float(body_ref.pose.orientation.x)
            row["body_ref_quat_y"] = float(body_ref.pose.orientation.y)
            row["body_ref_quat_z"] = float(body_ref.pose.orientation.z)
            row["body_ref_quat_w"] = float(body_ref.pose.orientation.w)
            row["body_ref_linear_x"] = float(body_ref.twist.linear.x)
            row["body_ref_linear_y"] = float(body_ref.twist.linear.y)
            row["body_ref_linear_z"] = float(body_ref.twist.linear.z)
            row["body_ref_angular_x"] = float(body_ref.twist.angular.x)
            row["body_ref_angular_y"] = float(body_ref.twist.angular.y)
            row["body_ref_angular_z"] = float(body_ref.twist.angular.z)
            row["body_ref_gait_mode"] = int(body_ref.gait_mode)
            support_mask = list(body_ref.support_mask)
            for idx, leg_name in enumerate(LEG_NAMES):
                row["body_ref_support_" + leg_name] = bool(
                    support_mask[idx] if idx < len(support_mask) else True
                )
        else:
            for key in (
                "body_ref_pose_x body_ref_pose_y body_ref_pose_z "
                "body_ref_quat_x body_ref_quat_y body_ref_quat_z body_ref_quat_w "
                "body_ref_linear_x body_ref_linear_y body_ref_linear_z "
                "body_ref_angular_x body_ref_angular_y body_ref_angular_z "
                "body_ref_gait_mode"
            ).split():
                row[key] = None
            for leg_name in LEG_NAMES:
                row["body_ref_support_" + leg_name] = None

        # ---------- estimated global body ----------
        if estimated is not None:
            row["body_est_pose_x"] = float(estimated.pose.position.x)
            row["body_est_pose_y"] = float(estimated.pose.position.y)
            row["body_est_pose_z"] = float(estimated.pose.position.z)
            row["body_est_quat_x"] = float(estimated.pose.orientation.x)
            row["body_est_quat_y"] = float(estimated.pose.orientation.y)
            row["body_est_quat_z"] = float(estimated.pose.orientation.z)
            row["body_est_quat_w"] = float(estimated.pose.orientation.w)
            row["body_est_linear_x"] = float(estimated.twist.linear.x)
            row["body_est_linear_y"] = float(estimated.twist.linear.y)
            row["body_est_linear_z"] = float(estimated.twist.linear.z)
            row["body_est_angular_x"] = float(estimated.twist.angular.x)
            row["body_est_angular_y"] = float(estimated.twist.angular.y)
            row["body_est_angular_z"] = float(estimated.twist.angular.z)
        else:
            for key in (
                "body_est_pose_x body_est_pose_y body_est_pose_z "
                "body_est_quat_x body_est_quat_y body_est_quat_z body_est_quat_w "
                "body_est_linear_x body_est_linear_y body_est_linear_z "
                "body_est_angular_x body_est_angular_y body_est_angular_z"
            ).split():
                row[key] = None

        # Joint-name map is needed for per-leg aggregates.
        joint_name_map = {}
        if joint_state is not None:
            joint_name_map = {
                str(int(name)): index for index, name in enumerate(joint_state.name)
            }

        # ---------- per-leg rows ----------
        for idx, leg_name in enumerate(LEG_NAMES):
            cmd = per_leg_cmd.get(leg_name)
            diag = per_leg_diag.get(leg_name) or []
            fan_cmd = per_leg_fan_cmd.get(leg_name)
            stance = per_leg_stance.get(leg_name)

            if cmd is not None:
                row["%s_cmd_center_x" % leg_name] = float(cmd.center.x)
                row["%s_cmd_center_y" % leg_name] = float(cmd.center.y)
                row["%s_cmd_center_z" % leg_name] = float(cmd.center.z)
                row["%s_cmd_vel_x" % leg_name] = float(cmd.center_velocity.x)
                row["%s_cmd_vel_y" % leg_name] = float(cmd.center_velocity.y)
                row["%s_cmd_vel_z" % leg_name] = float(cmd.center_velocity.z)
                row["%s_cmd_skirt_target" % leg_name] = float(cmd.skirt_compression_target)
                row["%s_cmd_support_leg" % leg_name] = bool(cmd.support_leg)
                row["%s_cmd_normal_force_limit" % leg_name] = float(
                    cmd.desired_normal_force_limit
                )
            else:
                for key_suffix in (
                    "cmd_center_x",
                    "cmd_center_y",
                    "cmd_center_z",
                    "cmd_vel_x",
                    "cmd_vel_y",
                    "cmd_vel_z",
                    "cmd_skirt_target",
                    "cmd_support_leg",
                    "cmd_normal_force_limit",
                ):
                    row["%s_%s" % (leg_name, key_suffix)] = None

            # Unpack diag by field name (matches swing_leg_controller layout).
            for field_index, field_name in enumerate(DIAG_FIELDS):
                column = "%s_diag_%s" % (leg_name, field_name)
                row[column] = (
                    float(diag[field_index]) if field_index < len(diag) else None
                )
            phase_id = row.get("%s_diag_phase_id" % leg_name)
            row["%s_phase_name" % leg_name] = (
                PHASE_NAMES.get(int(phase_id), "UNKNOWN") if phase_id is not None else None
            )

            # Fan command (what mission_supervisor / the test emits to the fan bridge).
            if fan_cmd is not None:
                row["%s_fan_cmd_mode" % leg_name] = int(fan_cmd.mode)
                row["%s_fan_cmd_mode_name" % leg_name] = FAN_MODE_NAMES.get(
                    int(fan_cmd.mode), "UNK"
                )
                row["%s_fan_cmd_rpm" % leg_name] = float(fan_cmd.target_rpm)
                row["%s_fan_cmd_normal_force_limit" % leg_name] = float(
                    fan_cmd.normal_force_limit
                )
                row["%s_fan_cmd_required_adhesion_force" % leg_name] = float(
                    fan_cmd.required_adhesion_force
                )
            else:
                for key_suffix in (
                    "fan_cmd_mode",
                    "fan_cmd_mode_name",
                    "fan_cmd_rpm",
                    "fan_cmd_normal_force_limit",
                    "fan_cmd_required_adhesion_force",
                ):
                    row["%s_%s" % (leg_name, key_suffix)] = None

            # Fan feedback.
            row["%s_fan_current_a" % leg_name] = (
                float(fan_currents[idx]) if idx < len(fan_currents) else None
            )
            row["%s_fan_rpm_fb" % leg_name] = (
                float(leg_rpm[idx]) if idx < len(leg_rpm) else None
            )

            # Stance-wrench planner output (normal force, friction use, etc.).
            if stance is not None:
                row["%s_stance_fx" % leg_name] = float(stance.wrench.force.x)
                row["%s_stance_fy" % leg_name] = float(stance.wrench.force.y)
                row["%s_stance_fz" % leg_name] = float(stance.wrench.force.z)
                row["%s_stance_tx" % leg_name] = float(stance.wrench.torque.x)
                row["%s_stance_ty" % leg_name] = float(stance.wrench.torque.y)
                row["%s_stance_tz" % leg_name] = float(stance.wrench.torque.z)
                row["%s_stance_normal_force_limit" % leg_name] = float(
                    stance.normal_force_limit
                )
                row["%s_stance_tangential_magnitude" % leg_name] = float(
                    stance.tangential_force_magnitude
                )
                row["%s_stance_required_adhesion_force" % leg_name] = float(
                    stance.required_adhesion_force
                )
                row["%s_stance_planned_support" % leg_name] = bool(stance.planned_support)
                row["%s_stance_actual_contact" % leg_name] = bool(stance.actual_contact)
                row["%s_stance_active" % leg_name] = bool(stance.active)
            else:
                for key_suffix in (
                    "stance_fx",
                    "stance_fy",
                    "stance_fz",
                    "stance_tx",
                    "stance_ty",
                    "stance_tz",
                    "stance_normal_force_limit",
                    "stance_tangential_magnitude",
                    "stance_required_adhesion_force",
                    "stance_planned_support",
                    "stance_actual_contact",
                    "stance_active",
                ):
                    row["%s_%s" % (leg_name, key_suffix)] = None

            # Estimated per-leg state.
            if estimated is not None:
                row["%s_skirt_compression_est" % leg_name] = _safe_index(
                    estimated.skirt_compression_estimate, idx
                )
                row["%s_slip_risk" % leg_name] = _safe_index(estimated.slip_risk, idx)
                row["%s_contact_confidence" % leg_name] = _safe_index(
                    estimated.contact_confidence, idx
                )
                row["%s_seal_confidence" % leg_name] = _safe_index(
                    estimated.seal_confidence, idx
                )
                row["%s_leg_torque_sum" % leg_name] = _safe_index(
                    estimated.leg_torque_sum, idx
                )
                row["%s_est_fan_current" % leg_name] = _safe_index(
                    estimated.fan_current, idx
                )
                row["%s_wall_touch" % leg_name] = bool(
                    _safe_index(estimated.wall_touch_mask, idx, False)
                )
                row["%s_measured_contact" % leg_name] = bool(
                    _safe_index(estimated.measured_contact_mask, idx, False)
                )
                row["%s_early_contact" % leg_name] = bool(
                    _safe_index(estimated.early_contact_mask, idx, False)
                )
                row["%s_contact_mask" % leg_name] = bool(
                    _safe_index(estimated.contact_mask, idx, False)
                )
                row["%s_support_mask" % leg_name] = bool(
                    _safe_index(estimated.support_mask, idx, False)
                )
                row["%s_adhesion_mask" % leg_name] = bool(
                    _safe_index(estimated.adhesion_mask, idx, False)
                )
                row["%s_attachment_ready" % leg_name] = bool(
                    _safe_index(estimated.attachment_ready_mask, idx, False)
                )
                row["%s_plan_support" % leg_name] = bool(
                    _safe_index(estimated.plan_support_mask, idx, False)
                )
                if idx < len(estimated.universal_joint_center_positions):
                    ujc = estimated.universal_joint_center_positions[idx]
                    row["%s_ujc_x" % leg_name] = float(ujc.x)
                    row["%s_ujc_y" % leg_name] = float(ujc.y)
                    row["%s_ujc_z" % leg_name] = float(ujc.z)
                else:
                    row["%s_ujc_x" % leg_name] = None
                    row["%s_ujc_y" % leg_name] = None
                    row["%s_ujc_z" % leg_name] = None
            else:
                for key_suffix in (
                    "skirt_compression_est",
                    "slip_risk",
                    "contact_confidence",
                    "seal_confidence",
                    "leg_torque_sum",
                    "est_fan_current",
                    "wall_touch",
                    "measured_contact",
                    "early_contact",
                    "contact_mask",
                    "support_mask",
                    "adhesion_mask",
                    "attachment_ready",
                    "plan_support",
                    "ujc_x",
                    "ujc_y",
                    "ujc_z",
                ):
                    row["%s_%s" % (leg_name, key_suffix)] = None

            # Per-leg aggregated joint torque/current (mirrors state_estimator).
            row["%s_leg_current_sum_a" % leg_name] = None
            row["%s_leg_current_count" % leg_name] = None
            row["%s_leg_torque_sum_nm" % leg_name] = None
            row["%s_leg_torque_count" % leg_name] = None
            if joint_state is not None and self._leg_motor_ids.get(leg_name):
                efforts = list(joint_state.effort) if joint_state.effort else []
                torque_sum, torque_count = _aggregate_leg_scalar(
                    self._leg_motor_ids[leg_name], efforts, joint_name_map
                )
                if torque_count > 0:
                    row["%s_leg_torque_sum_nm" % leg_name] = float(torque_sum)
                    row["%s_leg_torque_count" % leg_name] = int(torque_count)
                if joint_currents_vec is not None:
                    cur_sum, cur_count = _aggregate_leg_scalar(
                        self._leg_motor_ids[leg_name],
                        joint_currents_vec,
                        joint_name_map,
                    )
                    if cur_count > 0:
                        row["%s_leg_current_sum_a" % leg_name] = float(cur_sum)
                        row["%s_leg_current_count" % leg_name] = int(cur_count)
        return row

    # ---------- Swing event tracking ----------

    def _update_swing_events(self, row, now_rel):
        """Track per-leg swing cycles and record fan-synchronization metrics.

        A "swing" is bracketed by the `_cmd_support_leg` flag on swing_leg_target
        transitioning True -> False (swing start) and False -> True (swing end).
        Fan-synchronization metrics captured per swing event:
          * swing_start_time / swing_end_time (rel-to-test)
          * phase_id trajectory: first time each phase id was observed
          * first_fan_release_time  (first fan_cmd_mode==0 after swing start)
          * first_fan_boost_time    (first fan_cmd_mode==2 after swing start)
          * first_fan_attach_time   (first fan_cmd_mode==1 after swing start)
          * first_wall_touch_time / first_measured_contact_time /
            first_attachment_ready_time / first_adhesion_time (rel to swing start)
        """
        for leg_name in LEG_NAMES:
            support_flag = row.get("%s_cmd_support_leg" % leg_name)
            last_support = self._last_support_leg.get(leg_name)
            if support_flag is None:
                continue
            support_flag = bool(support_flag)

            # Swing start: support -> !support
            if last_support is not False and support_flag is False:
                event = {
                    "leg_name": leg_name,
                    "swing_start_time": now_rel,
                    "swing_end_time": None,
                    "phase_first_time": {},
                    "first_fan_release_time": None,
                    "first_fan_boost_time": None,
                    "first_fan_attach_time": None,
                    "first_wall_touch_time": None,
                    "first_measured_contact_time": None,
                    "first_attachment_ready_time": None,
                    "first_adhesion_time": None,
                    "max_est_normal_force_n": 0.0,
                    "max_compliant_offset_m": 0.0,
                    "max_skirt_compression": 0.0,
                    "fan_rpm_at_attach_ready": None,
                    "fan_current_at_attach_ready": None,
                    "phase_seq": [],
                }
                self._current_swing_event[leg_name] = event
                self._swing_events[leg_name].append(event)

            event = self._current_swing_event.get(leg_name)
            if event is not None:
                # Track phase first-seen times while the leg is in swing.
                phase_id = row.get("%s_diag_phase_id" % leg_name)
                if phase_id is not None:
                    pid = int(phase_id)
                    if pid not in event["phase_first_time"]:
                        event["phase_first_time"][pid] = now_rel
                        event["phase_seq"].append(pid)

                # Fan command bookkeeping relative to swing start.
                fan_mode = row.get("%s_fan_cmd_mode" % leg_name)
                rel = now_rel - event["swing_start_time"]
                if fan_mode is not None:
                    fm = int(fan_mode)
                    if fm == FAN_MODE_RELEASE and event["first_fan_release_time"] is None:
                        event["first_fan_release_time"] = rel
                    if fm == FAN_MODE_BOOST and event["first_fan_boost_time"] is None:
                        event["first_fan_boost_time"] = rel
                    if fm == FAN_MODE_ATTACH and event["first_fan_attach_time"] is None:
                        event["first_fan_attach_time"] = rel

                # Contact/adhesion progression relative to swing start.
                if bool(row.get("%s_wall_touch" % leg_name)) and event["first_wall_touch_time"] is None:
                    event["first_wall_touch_time"] = rel
                if bool(row.get("%s_measured_contact" % leg_name)) and event["first_measured_contact_time"] is None:
                    event["first_measured_contact_time"] = rel
                if bool(row.get("%s_attachment_ready" % leg_name)) and event["first_attachment_ready_time"] is None:
                    event["first_attachment_ready_time"] = rel
                    event["fan_rpm_at_attach_ready"] = row.get("%s_fan_rpm_fb" % leg_name)
                    event["fan_current_at_attach_ready"] = row.get("%s_fan_current_a" % leg_name)
                if bool(row.get("%s_adhesion_mask" % leg_name)) and event["first_adhesion_time"] is None:
                    event["first_adhesion_time"] = rel

                est_force = row.get("%s_diag_estimated_leg_normal_force_n" % leg_name)
                if est_force is not None:
                    event["max_est_normal_force_n"] = max(
                        event["max_est_normal_force_n"], abs(float(est_force))
                    )
                offset = row.get("%s_diag_compliant_normal_offset_m" % leg_name)
                if offset is not None:
                    event["max_compliant_offset_m"] = max(
                        event["max_compliant_offset_m"], abs(float(offset))
                    )
                skirt = row.get("%s_skirt_compression_est" % leg_name)
                if skirt is not None:
                    event["max_skirt_compression"] = max(
                        event["max_skirt_compression"], float(skirt)
                    )

            # Swing end: !support -> support
            if last_support is False and support_flag is True and event is not None:
                event["swing_end_time"] = now_rel
                self._current_swing_event[leg_name] = None

            self._last_support_leg[leg_name] = support_flag

    # ---------- Main loop ----------

    def run(self):
        try:
            self._run_inner()
        finally:
            self._shutdown_cleanup()

    def _run_inner(self):
        rospy.loginfo(
            "[test_crawl_gait] waiting for body_reference / state / swing diag streams..."
        )
        self._wait_for_streams(timeout_s=8.0)

        # Log a startup snapshot so the operator can see what the robot looked
        # like before the mission was started.
        rospy.loginfo(
            "[test_crawl_gait] configured nominal UJC z = %.4f m (requested %.4f m). "
            "Starting mission in %.2fs.",
            self.configured_ujc_z_m,
            self.ujc_z_mm / 1000.0,
            self.warmup_s,
        )

        self.test_phase = "WARMUP"
        self.test_started_at = time.time()

        logger_thread = threading.Thread(target=self._log_loop, name="crawl_log_loop")
        logger_thread.daemon = True
        logger_thread.start()
        status_thread = threading.Thread(target=self._status_loop, name="crawl_status_loop")
        status_thread.daemon = True
        status_thread.start()

        warmup_end = self.test_started_at + self.warmup_s
        while time.time() < warmup_end and not self._stop_requested and not rospy.is_shutdown():
            time.sleep(0.05)
        if self._stop_requested or rospy.is_shutdown():
            return

        rospy.loginfo(
            "[test_crawl_gait] publishing /control/mission_start=True; expect "
            "INIT -> STICK -> CLIMB transitions."
        )
        self.test_phase = "CLIMB_REQUESTED"
        self._publish_mission_start()

        deadline = time.time() + self.test_duration_s
        rate = rospy.Rate(self.control_rate_hz)
        while not self._stop_requested and not rospy.is_shutdown() and time.time() < deadline:
            # The test script itself does not publish BodyReference / AdhesionCommand
            # during a crawl test: body_planner drives the gait and mission_supervisor
            # owns the fans. We just monitor and keep the mission start flag latched.
            if self.latest_mission_state == "FAULT":
                rospy.logerr(
                    "[test_crawl_gait] mission_supervisor entered FAULT state; aborting test loop."
                )
                self.test_phase = "FAULTED"
                break
            rate.sleep()

        rospy.loginfo("[test_crawl_gait] test window ended (stop_requested=%s)", self._stop_requested)
        self.test_phase = "COOLDOWN"
        self._publish_mission_pause()
        cooldown_end = time.time() + self.cooldown_s
        while time.time() < cooldown_end and not rospy.is_shutdown():
            time.sleep(0.05)

        self.test_phase = "TERMINATE"

    def _log_loop(self):
        rate = rospy.Rate(self.log_rate_hz)
        while not rospy.is_shutdown():
            if self.test_started_at is None:
                rate.sleep()
                continue
            wall_time = time.time() - self.test_started_at
            row = self._snapshot(wall_time)
            self.samples.append(row)
            self._update_swing_events(row, wall_time)
            if self.test_phase == "TERMINATE":
                break
            rate.sleep()

    def _status_loop(self):
        rate = rospy.Rate(2.0)
        while not rospy.is_shutdown():
            if self.test_started_at is None:
                rate.sleep()
                continue
            if not self.verbose_status:
                rate.sleep()
                if self.test_phase == "TERMINATE":
                    break
                continue
            wall_time = time.time() - self.test_started_at
            self._print_live_status(wall_time)
            if self.test_phase == "TERMINATE":
                break
            rate.sleep()

    def _print_live_status(self, wall_time):
        with self._state_lock:
            mission_state = self.latest_mission_state
            mission_active = self.latest_mission_active
            body_ref = self.latest_body_reference
            estimated = self.latest_estimated
            fan_currents = list(self.latest_fan_currents) if self.latest_fan_currents else []
            fan_rpm = list(self.latest_leg_rpm) if self.latest_leg_rpm else []
            per_leg_diag = {k: list(v) if v else [] for k, v in self.latest_leg_diag.items()}
            per_leg_fan_cmd = dict(self.latest_leg_adhesion_cmd)

        parts = []
        for idx, leg_name in enumerate(LEG_NAMES):
            diag = per_leg_diag.get(leg_name) or []
            phase_id = None
            if DIAG_FIELDS.index("phase_id") < len(diag):
                phase_id = int(diag[DIAG_FIELDS.index("phase_id")])
            phase_name = PHASE_NAMES.get(phase_id, "?") if phase_id is not None else "?"
            support_cmd = None
            cmd_fan = per_leg_fan_cmd.get(leg_name)
            mode = int(cmd_fan.mode) if cmd_fan is not None else -1
            mode_name = FAN_MODE_NAMES.get(mode, "--")
            fan_cur = fan_currents[idx] if idx < len(fan_currents) else float("nan")
            fan_rpm_fb = fan_rpm[idx] if idx < len(fan_rpm) else float("nan")
            support = None
            adh = None
            if estimated is not None:
                support = bool(_safe_index(estimated.support_mask, idx, False))
                adh = bool(_safe_index(estimated.adhesion_mask, idx, False))
            parts.append(
                "%s[ph=%s sup=%s adh=%s fan=%s rpm=%.0f i=%.2f]" % (
                    leg_name,
                    phase_name,
                    ("1" if support else "0") if support is not None else "?",
                    ("1" if adh else "0") if adh is not None else "?",
                    mode_name,
                    fan_rpm_fb,
                    fan_cur,
                )
            )
        rospy.loginfo_throttle(
            0.5,
            "[test_crawl_gait t=%6.2fs] mission=%s active=%s %s",
            wall_time,
            mission_state,
            mission_active,
            " ".join(parts),
        )

    def _shutdown_cleanup(self):
        rospy.loginfo("[test_crawl_gait] cleanup: pausing mission")
        try:
            self._publish_mission_pause()
        except Exception as exc:
            rospy.logwarn("[test_crawl_gait] failed to publish mission_pause: %s", exc)

        if self.auto_release_fans_on_exit:
            rospy.loginfo(
                "[test_crawl_gait] releasing all fans on exit (--release-fans-on-exit=True)."
            )
            for _ in range(10):
                if rospy.is_shutdown():
                    break
                self._release_all_fans()
                rospy.sleep(0.05)
        else:
            rospy.loginfo(
                "[test_crawl_gait] leaving fans under mission_supervisor control "
                "(use --release-fans-on-exit to zero them at the end)."
            )

    # ---------- CSV output ----------

    def _build_csv_fieldnames(self):
        fieldnames = [
            "wall_time",
            "ros_time",
            "test_phase",
            "mission_state",
            "mission_active",
            "safe_mode",
            "body_ref_pose_x",
            "body_ref_pose_y",
            "body_ref_pose_z",
            "body_ref_quat_x",
            "body_ref_quat_y",
            "body_ref_quat_z",
            "body_ref_quat_w",
            "body_ref_linear_x",
            "body_ref_linear_y",
            "body_ref_linear_z",
            "body_ref_angular_x",
            "body_ref_angular_y",
            "body_ref_angular_z",
            "body_ref_gait_mode",
        ]
        for leg_name in LEG_NAMES:
            fieldnames.append("body_ref_support_" + leg_name)
        fieldnames += [
            "body_est_pose_x",
            "body_est_pose_y",
            "body_est_pose_z",
            "body_est_quat_x",
            "body_est_quat_y",
            "body_est_quat_z",
            "body_est_quat_w",
            "body_est_linear_x",
            "body_est_linear_y",
            "body_est_linear_z",
            "body_est_angular_x",
            "body_est_angular_y",
            "body_est_angular_z",
        ]
        for leg_name in LEG_NAMES:
            fieldnames.append("%s_phase_name" % leg_name)
            fieldnames += [
                "%s_cmd_center_x" % leg_name,
                "%s_cmd_center_y" % leg_name,
                "%s_cmd_center_z" % leg_name,
                "%s_cmd_vel_x" % leg_name,
                "%s_cmd_vel_y" % leg_name,
                "%s_cmd_vel_z" % leg_name,
                "%s_cmd_skirt_target" % leg_name,
                "%s_cmd_support_leg" % leg_name,
                "%s_cmd_normal_force_limit" % leg_name,
            ]
            for field_name in DIAG_FIELDS:
                fieldnames.append("%s_diag_%s" % (leg_name, field_name))
            fieldnames += [
                "%s_fan_cmd_mode" % leg_name,
                "%s_fan_cmd_mode_name" % leg_name,
                "%s_fan_cmd_rpm" % leg_name,
                "%s_fan_cmd_normal_force_limit" % leg_name,
                "%s_fan_cmd_required_adhesion_force" % leg_name,
                "%s_fan_current_a" % leg_name,
                "%s_fan_rpm_fb" % leg_name,
                "%s_stance_fx" % leg_name,
                "%s_stance_fy" % leg_name,
                "%s_stance_fz" % leg_name,
                "%s_stance_tx" % leg_name,
                "%s_stance_ty" % leg_name,
                "%s_stance_tz" % leg_name,
                "%s_stance_normal_force_limit" % leg_name,
                "%s_stance_tangential_magnitude" % leg_name,
                "%s_stance_required_adhesion_force" % leg_name,
                "%s_stance_planned_support" % leg_name,
                "%s_stance_actual_contact" % leg_name,
                "%s_stance_active" % leg_name,
                "%s_skirt_compression_est" % leg_name,
                "%s_slip_risk" % leg_name,
                "%s_contact_confidence" % leg_name,
                "%s_seal_confidence" % leg_name,
                "%s_leg_torque_sum" % leg_name,
                "%s_est_fan_current" % leg_name,
                "%s_wall_touch" % leg_name,
                "%s_measured_contact" % leg_name,
                "%s_early_contact" % leg_name,
                "%s_contact_mask" % leg_name,
                "%s_support_mask" % leg_name,
                "%s_adhesion_mask" % leg_name,
                "%s_attachment_ready" % leg_name,
                "%s_plan_support" % leg_name,
                "%s_ujc_x" % leg_name,
                "%s_ujc_y" % leg_name,
                "%s_ujc_z" % leg_name,
                "%s_leg_current_sum_a" % leg_name,
                "%s_leg_current_count" % leg_name,
                "%s_leg_torque_sum_nm" % leg_name,
                "%s_leg_torque_count" % leg_name,
            ]
        return fieldnames

    def save_log(self):
        if not self.samples:
            rospy.logwarn("[test_crawl_gait] no samples captured; skip log")
            return None
        os.makedirs(self.output_dir, exist_ok=True)
        stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = os.path.join(
            self.output_dir,
            "crawl_gait_%s.csv" % stamp,
        )
        fieldnames = self._build_csv_fieldnames()
        with open(filename, "w") as fh:
            writer = csv.DictWriter(fh, fieldnames=fieldnames, extrasaction="ignore")
            writer.writeheader()
            for row in self.samples:
                serialized = {}
                for key in fieldnames:
                    value = row.get(key)
                    if value is None:
                        serialized[key] = ""
                    elif isinstance(value, bool):
                        serialized[key] = "1" if value else "0"
                    elif isinstance(value, float):
                        if math.isnan(value) or math.isinf(value):
                            serialized[key] = ""
                        else:
                            serialized[key] = "%.6f" % value
                    else:
                        serialized[key] = value
                writer.writerow(serialized)
        rospy.loginfo(
            "[test_crawl_gait] log saved to %s (%d rows, %d columns)",
            filename,
            len(self.samples),
            len(fieldnames),
        )
        return filename

    # ---------- Summary ----------

    def print_summary(self):
        print()
        print("=" * 78)
        print(" Whole-body crawl gait test summary")
        print("=" * 78)
        print("  UJC z configured (param/m): %.4f" % self.configured_ujc_z_m)
        print("  UJC z requested  (cli /mm ): %.3f" % self.ujc_z_mm)
        print("  test_duration_s           : %.2f" % self.test_duration_s)
        print("  samples captured          : %d" % len(self.samples))

        if self._mission_state_history:
            print()
            print("  Mission state transitions:")
            for t_rel, state in self._mission_state_history:
                print("    t=%6.3fs  %s" % (t_rel, state))

        if not self.samples:
            print("=" * 78)
            return

        duration = self.samples[-1]["wall_time"] - self.samples[0]["wall_time"]
        print()
        print("  Recorded window: %.2fs  (samples: %d, avg rate: %.1f Hz)" % (
            duration,
            len(self.samples),
            len(self.samples) / max(duration, 1e-3),
        ))

        # Per-leg swing-event analysis
        print()
        print("  --- Per-leg swing / fan coordination ---")
        for leg_name in LEG_NAMES:
            events = [e for e in self._swing_events[leg_name] if e["swing_end_time"] is not None]
            incomplete = [e for e in self._swing_events[leg_name] if e["swing_end_time"] is None]
            print(
                "  %s: complete_swings=%d  open_swings_at_cutoff=%d"
                % (leg_name, len(events), len(incomplete))
            )
            if not events:
                if incomplete:
                    ev = incomplete[-1]
                    print(
                        "    last open swing started at t=%.3fs; phases seen so far: %s"
                        % (
                            ev["swing_start_time"],
                            ", ".join(
                                PHASE_NAMES.get(p, str(p)) for p in ev["phase_seq"]
                            ) or "(none)",
                        )
                    )
                continue

            swing_durations = [
                e["swing_end_time"] - e["swing_start_time"] for e in events
            ]
            mm = _min_max_mean(swing_durations)
            if mm is not None:
                lo, hi, mid = mm
                print(
                    "    swing duration  (s) min=%.3f  max=%.3f  mean=%.3f"
                    % (lo, hi, mid)
                )

            def _collect(field):
                values = [e[field] for e in events if e[field] is not None]
                return _min_max_mean(values)

            for field in (
                "first_fan_release_time",
                "first_fan_boost_time",
                "first_fan_attach_time",
                "first_wall_touch_time",
                "first_measured_contact_time",
                "first_attachment_ready_time",
                "first_adhesion_time",
            ):
                mm = _collect(field)
                label = field.replace("first_", "first ").replace("_time", "").replace("_", " ")
                if mm is None:
                    print("    %s: (never asserted)" % label)
                else:
                    lo, hi, mid = mm
                    print(
                        "    %s (s since swing start): min=%.3f max=%.3f mean=%.3f "
                        "(n=%d / %d)"
                        % (
                            label,
                            lo,
                            hi,
                            mid,
                            sum(1 for e in events if e[field] is not None),
                            len(events),
                        )
                    )

            max_force = _collect("max_est_normal_force_n")
            if max_force is not None:
                lo, hi, mid = max_force
                print(
                    "    peak |normal force| per swing (N): min=%.2f max=%.2f mean=%.2f"
                    % (lo, hi, mid)
                )

            max_offset = _collect("max_compliant_offset_m")
            if max_offset is not None:
                lo, hi, mid = max_offset
                print(
                    "    peak admittance travel per swing (m): min=%.4f max=%.4f mean=%.4f"
                    % (lo, hi, mid)
                )

            max_skirt = _collect("max_skirt_compression")
            if max_skirt is not None:
                lo, hi, mid = max_skirt
                print(
                    "    peak skirt compression (unitless): min=%.3f max=%.3f mean=%.3f"
                    % (lo, hi, mid)
                )

            # Fan coordination window: time from first_fan_boost to first_adhesion (per swing).
            boost_to_adhesion = []
            for ev in events:
                if ev["first_fan_boost_time"] is not None and ev["first_adhesion_time"] is not None:
                    boost_to_adhesion.append(
                        ev["first_adhesion_time"] - ev["first_fan_boost_time"]
                    )
            mm = _min_max_mean(boost_to_adhesion)
            if mm is not None:
                lo, hi, mid = mm
                print(
                    "    fan_boost -> adhesion latency (s): min=%.3f max=%.3f mean=%.3f"
                    % (lo, hi, mid)
                )

            # Coordination check: the fan boost should normally happen after the leg is
            # past PRELOAD (phase 5) / around COMPLIANT_SETTLE (phase 6). Count how many
            # swings had boost land on or after phase 5.
            in_phase = {"boost_before_phase5": 0, "boost_on_or_after_phase5": 0}
            for ev in events:
                t_boost = ev["first_fan_boost_time"]
                t_p5 = ev["phase_first_time"].get(5)
                if t_boost is None:
                    continue
                if t_p5 is None or t_boost < t_p5:
                    in_phase["boost_before_phase5"] += 1
                else:
                    in_phase["boost_on_or_after_phase5"] += 1
            print(
                "    fan-boost vs swing phase: on_or_after_PRELOAD=%d  before_PRELOAD=%d "
                "(lower 'before' is better)"
                % (
                    in_phase["boost_on_or_after_phase5"],
                    in_phase["boost_before_phase5"],
                )
            )

        # Gait-level: average cycle time across legs using body_ref support-mask edges.
        print()
        print("  --- Gait cycle timing (from body_ref_support_* edges) ---")
        for leg_name in LEG_NAMES:
            edges = []
            last = None
            for row in self.samples:
                s = row.get("body_ref_support_" + leg_name)
                if s is None:
                    continue
                if last is None:
                    last = s
                    continue
                if bool(s) != bool(last):
                    edges.append((row["wall_time"], bool(s)))
                    last = bool(s)
            starts = [t for t, s in edges if not s]  # True -> False (swing start)
            if len(starts) >= 2:
                periods = [b - a for a, b in zip(starts, starts[1:])]
                mm = _min_max_mean(periods)
                lo, hi, mid = mm
                print(
                    "  %s: cycle_period(s)  n=%d  min=%.3f  max=%.3f  mean=%.3f"
                    % (leg_name, len(periods), lo, hi, mid)
                )
            else:
                print("  %s: (no full cycle observed in recording)" % leg_name)

        print("=" * 78)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    parser.add_argument(
        "--ujc-z-mm",
        type=float,
        default=-80.0,
        help=(
            "Override /gait_controller/nominal_universal_joint_center_z (millimetres). "
            "Default: -80.0 mm. NOTE: consumed at node startup; if the control stack "
            "is already running it must be relaunched to pick up a new value."
        ),
    )
    parser.add_argument(
        "--test-duration-s",
        type=float,
        default=30.0,
        help="Recording window after mission_start is published (default: 30s).",
    )
    parser.add_argument(
        "--warmup-s",
        type=float,
        default=1.5,
        help="Delay after the test script starts logging before /control/mission_start "
             "is published (default: 1.5s). Gives a clean pre-mission baseline in the CSV.",
    )
    parser.add_argument(
        "--cooldown-s",
        type=float,
        default=1.0,
        help="Delay after /control/mission_pause is published before CSV is saved "
             "(default: 1.0s). Captures the PAUSE transient in the log.",
    )
    parser.add_argument(
        "--control-rate-hz",
        type=float,
        default=50.0,
        help="Outer monitoring loop rate (default: 50Hz).",
    )
    parser.add_argument(
        "--log-rate-hz",
        type=float,
        default=50.0,
        help="Rate at which samples are captured to the log (default: 50Hz).",
    )
    parser.add_argument(
        "--linear-velocity-mps",
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=[0.0, 0.0, 0.0],
        help="Desired body linear velocity (m/s) pushed to body_planner params. "
             "Requires body_planner restart to take effect (default: in-place crawl).",
    )
    parser.add_argument(
        "--angular-velocity-rps",
        nargs=3,
        metavar=("X", "Y", "Z"),
        default=[0.0, 0.0, 0.0],
        help="Desired body angular velocity (rad/s) pushed to body_planner params. "
             "Requires body_planner restart (default: 0 0 0).",
    )
    parser.add_argument(
        "--output-dir",
        default=_default_test_logs_dir(),
        help="Where to write the CSV log (default: <workspace>/test_logs).",
    )
    parser.add_argument(
        "--verbose-status",
        action="store_true",
        help="Print a throttled live per-leg status line while the test runs.",
    )
    parser.add_argument(
        "--release-fans-on-exit",
        action="store_true",
        help="Explicitly command all fans to mode=RELEASE rpm=0 during cleanup. "
             "Default: leave fans under mission_supervisor PAUSE-state control.",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    tester = CrawlGaitTest(args)
    try:
        tester.run()
    except rospy.ROSInterruptException:
        pass
    tester.save_log()
    tester.print_summary()


if __name__ == "__main__":
    main(sys.argv[1:] if len(sys.argv) > 1 else None)
