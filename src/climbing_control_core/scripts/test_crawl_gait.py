#!/usr/bin/env python3
"""
整机 crawl 步态集成测试 (半交互流程)

预设运行环境 (调用本脚本前必须就绪):
  - Jetson 侧已起 jetson_bringup.launch, leg_ik_executor 已把四条腿
    平滑送至 operating UJC (270.21, 0, -100) [mm].
  - PC 侧已起 test_crawl_gait.launch (mission_auto_start=false). 此时
    mission_state == "INIT", swing_leg_controller 在 INIT 状态强制
    LegCenterCommand.support_leg=false, dynamixel_bridge 不切电流模式,
    四腿在位置模式下静止保持 operating UJC.

本脚本流程:
  1. 等关键话题就绪 (mission_state, body_reference, estimated_state,
     swing_leg_target, fan_rpm, fan_currents).
  2. 在 INIT 阶段做一段稳定性观察 (--hold-check-s, 默认 3s):
       - 所有腿 cmd_support_leg 必须为 false
       - 所有腿 UJC z 与 operating_z 偏差 < --hold-tol-mm (默认 5mm)
     未通过则中止 (避免后续盲飞).
  3. 提示用户回车确认 "机器人已贴墙, 风机控制就位" (除非 --no-confirm).
  4. 发布 /control/mission_start=true:
       - 等到 mission_state == STICK (--stick-timeout-s, 默认 15s)
       - 等到 mission_state == CLIMB (--climb-timeout-s, 默认 30s)
  5. CLIMB 状态下持续 --duration 秒 (默认 60), 实时按 --log-rate-hz
     采样 CSV. 中途如出现 FAULT 立即退出.
  6. 发布 /control/mission_pause=true 收尾, 写入 CSV, 打印总结.

记录字段精简版 (约 60 列):
  通用 (6):    wall_time, ros_time, elapsed_s, mission_state, mission_active, phase_label
  body  (6):   body_x, body_y, body_z, body_vx, body_vy, body_vz
  腿 ×4(12):   {leg}_phase, {leg}_phase_id,
               {leg}_cmd_x, {leg}_cmd_y, {leg}_cmd_z, {leg}_cmd_support,
               {leg}_ujc_z, {leg}_attachment_ready, {leg}_adhesion,
               {leg}_measured_contact, {leg}_fan_rpm, {leg}_fan_current_a
"""

from __future__ import print_function

import argparse
import csv
import datetime as _dt
import math
import os
import sys
import threading
import time

import rospy
from std_msgs.msg import Bool, Float32MultiArray, String

from climbing_msgs.msg import (
    BodyReference,
    EstimatedState,
    LegCenterCommand,
)


LEG_NAMES = ["lf", "rf", "rr", "lr"]

PHASE_ID_MAP = {
    "SUPPORT": 0,
    "TEST_LIFT_CLEARANCE": 1,
    "TEST_PRESS_CONTACT": 2,
    "DETACH_SLIDE": 3,
    "TANGENTIAL_ALIGN": 4,
    "PRELOAD_COMPRESS": 5,
    "COMPLIANT_SETTLE": 6,
    "ATTACHED_HOLD": 7,
}

STATE_INIT = "INIT"
STATE_STICK = "STICK"
STATE_CLIMB = "CLIMB"
STATE_PAUSE = "PAUSE"
STATE_FAULT = "FAULT"


class CrawlGaitTester(object):
    def __init__(self, args):
        self.args = args

        # 最近一次收到的各话题数据, 全部用 lock 保护读写
        self._lock = threading.Lock()
        self._latest_mission_state = None
        self._latest_mission_active = False
        self._latest_body_reference = None
        self._latest_estimated_state = None
        self._latest_swing_targets = {leg: None for leg in LEG_NAMES}
        self._latest_fan_rpm = [0.0, 0.0, 0.0, 0.0]
        self._latest_fan_currents = [0.0, 0.0, 0.0, 0.0]

        # 状态机切换时间线, 用于最后总结
        self._state_history = []  # list of (rel_time, state)
        self._fault_seen = False

        # operating UJC 用于 INIT 稳定性判定
        operating_z_mm = float(rospy.get_param(
            "/gait_controller/operating_universal_joint_center_z",
            -100.0,
        ))
        self._operating_z_m = operating_z_mm / 1000.0

        # 发布器, latch=False (不需要保持最后状态)
        self._mission_start_pub = rospy.Publisher(
            "/control/mission_start", Bool, queue_size=2,
        )
        self._mission_pause_pub = rospy.Publisher(
            "/control/mission_pause", Bool, queue_size=2,
        )

        # 订阅器
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

        # CSV 字段表与文件句柄, 在 _open_csv 中初始化
        self._csv_path = None
        self._csv_file = None
        self._csv_writer = None
        self._csv_columns = self._build_csv_columns()
        self._sample_count = 0
        self._t_zero_wall = None
        self._t_zero_ros = None

        # 摆动周期计数 (用于总结)
        self._prev_support = {leg: None for leg in LEG_NAMES}
        self._swing_count = {leg: 0 for leg in LEG_NAMES}

    # ------------------------------------------------------------------ #
    # ROS 回调
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
        values = [float(value) for value in msg.data[:4]]
        while len(values) < 4:
            values.append(0.0)
        with self._lock:
            self._latest_fan_rpm = values

    def _cb_fan_currents(self, msg):
        values = [float(value) for value in msg.data[:4]]
        while len(values) < 4:
            values.append(0.0)
        with self._lock:
            self._latest_fan_currents = values

    # ------------------------------------------------------------------ #
    # 时序 / 工具
    # ------------------------------------------------------------------ #
    def _rel_now(self):
        if self._t_zero_ros is None:
            return 0.0
        return (rospy.Time.now() - self._t_zero_ros).to_sec()

    def _wait_streams(self, timeout_s):
        """阻塞等待所有关键订阅都至少收到一次. 超时返回 False."""
        deadline = time.time() + timeout_s
        rospy.loginfo("test_crawl_gait: waiting for upstream streams (timeout=%.1fs)...",
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
                rospy.loginfo("test_crawl_gait: all streams ready.")
                return True
            if time.time() >= deadline:
                rospy.logerr(
                    "test_crawl_gait: stream readiness timeout. "
                    "state=%s body=%s estimated=%s swing=%s",
                    ok_state, ok_body, ok_est, ok_swing,
                )
                return False
            rospy.sleep(0.1)
        return False

    # ------------------------------------------------------------------ #
    # INIT 稳定性检查
    # ------------------------------------------------------------------ #
    def _snapshot_init_state(self):
        """单帧抓取 mission_state, swing_target, estimated UJC; 用于诊断打印."""
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
        """生成可读性好的 INIT 状态表 (按腿一行)."""
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
        """在 INIT 阶段连续 hold_check_s 内验证四腿处于位置保持模式.

        失败时打印 4 条腿的完整 INIT 状态表, 用户能直接看到哪条偏多少, 是机械
        误差还是真异常. 可以通过 --allow-init-mismatch 强制继续.
        """
        check_s = max(0.5, float(self.args.hold_check_s))
        tol_m = max(0.001, float(self.args.hold_tol_mm) / 1000.0)
        rate = rospy.Rate(20.0)
        end_t = time.time() + check_s
        rospy.loginfo(
            "test_crawl_gait: verifying INIT hold for %.1fs "
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
                    "test_crawl_gait: mission_state=%s (expected INIT). "
                    "脚本只在 INIT 启动. 请退出后确认.",
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
                    "test_crawl_gait: INIT hold check failed (%s):\n%s\n  violations: %s",
                    latest_state,
                    self._format_init_table(latest_legs, tol_m),
                    "; ".join(violations),
                )
                if bool(self.args.allow_init_mismatch):
                    rospy.logwarn(
                        "test_crawl_gait: --allow-init-mismatch set, continuing despite mismatch.",
                    )
                    return True
                rospy.logwarn(
                    "test_crawl_gait: hint - lr 大幅偏差通常是 startup_move 未完成 / 电机零点错位; "
                    "12 mm 量级常见于机械校准误差, 可加大 --hold-tol-mm 放宽."
                )
                return False
            rate.sleep()

        rospy.loginfo(
            "test_crawl_gait: INIT hold check passed.\n%s",
            self._format_init_table(latest_legs, tol_m),
        )
        return True

    # ------------------------------------------------------------------ #
    # mission state 推进
    # ------------------------------------------------------------------ #
    def _mission_diagnostic_lines(self):
        """Return compact per-leg adhesion/fan diagnostics for STICK failures."""
        with self._lock:
            est = self._latest_estimated_state
            fan_rpm = list(self._latest_fan_rpm)
            fan_curr = list(self._latest_fan_currents)

        lines = [
            "  leg  adhesion  attach_ready  seal_conf  fan_rpm  fan_current_a  measured_contact",
        ]
        for leg_index, leg in enumerate(LEG_NAMES):
            def mask(attr_name):
                if est is None:
                    return False
                values = getattr(est, attr_name, [])
                return leg_index < len(values) and bool(values[leg_index])

            seal = 0.0
            if est is not None and leg_index < len(est.seal_confidence):
                seal = float(est.seal_confidence[leg_index])
            rpm = fan_rpm[leg_index] if leg_index < len(fan_rpm) else 0.0
            current = fan_curr[leg_index] if leg_index < len(fan_curr) else 0.0
            lines.append(
                "  %-3s     %-5s       %-5s       %6.3f   %7.1f      %7.3f          %-5s" % (
                    leg,
                    str(mask("adhesion_mask")),
                    str(mask("attachment_ready_mask")),
                    seal,
                    float(rpm),
                    float(current),
                    str(mask("measured_contact_mask")),
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
                    "test_crawl_gait: entered FAULT while waiting for %s.\n%s",
                    target,
                    self._mission_diagnostic_lines(),
                )
                return False
            if time.time() >= deadline:
                rospy.logerr(
                    "test_crawl_gait: timeout (%ss) waiting for state %s; current=%s\n%s",
                    timeout_s, target, state, self._mission_diagnostic_lines(),
                )
                return False
            now = time.time()
            if now - last_log > 1.0:
                rospy.loginfo(
                    "test_crawl_gait: waiting for %s, current=%s "
                    "(remain %.1fs)...",
                    target, state, max(0.0, deadline - now),
                )
                last_log = now
            rospy.sleep(0.1)
        return False

    def _publish_start(self):
        rospy.loginfo("test_crawl_gait: publishing /control/mission_start=true.")
        for _ in range(3):
            self._mission_start_pub.publish(Bool(data=True))
            rospy.sleep(0.05)

    def _publish_pause(self):
        rospy.loginfo("test_crawl_gait: publishing /control/mission_pause=true.")
        for _ in range(3):
            self._mission_pause_pub.publish(Bool(data=True))
            rospy.sleep(0.05)

    # ------------------------------------------------------------------ #
    # CSV
    # ------------------------------------------------------------------ #
    def _build_csv_columns(self):
        cols = [
            "wall_time", "ros_time", "elapsed_s",
            "mission_state", "mission_active", "phase_label",
            "body_x", "body_y", "body_z",
            "body_vx", "body_vy", "body_vz",
        ]
        for leg in LEG_NAMES:
            cols.extend([
                "%s_phase" % leg,
                "%s_phase_id" % leg,
                "%s_cmd_x" % leg,
                "%s_cmd_y" % leg,
                "%s_cmd_z" % leg,
                "%s_cmd_support" % leg,
                "%s_ujc_z" % leg,
                "%s_attachment_ready" % leg,
                "%s_adhesion" % leg,
                "%s_measured_contact" % leg,
                "%s_fan_rpm" % leg,
                "%s_fan_current_a" % leg,
            ])
        return cols

    def _open_csv(self):
        out_dir = os.path.expanduser(str(self.args.output_dir))
        if not os.path.isdir(out_dir):
            os.makedirs(out_dir)
        timestamp = _dt.datetime.now().strftime("%Y%m%d_%H%M%S")
        path = os.path.join(out_dir, "crawl_gait_%s.csv" % timestamp)
        f = open(path, "w")
        writer = csv.writer(f)
        writer.writerow(self._csv_columns)
        self._csv_path = path
        self._csv_file = f
        self._csv_writer = writer
        rospy.loginfo("test_crawl_gait: writing CSV -> %s", path)

    def _close_csv(self):
        if self._csv_file is not None:
            try:
                self._csv_file.flush()
                self._csv_file.close()
            except (IOError, OSError):
                pass
            self._csv_file = None
            self._csv_writer = None

    def _phase_label_from_swing(self, leg_index, swing_msg):
        # LegCenterCommand 没有 phase 字段; 用 support_leg 与 normal_force_limit
        # 推断粗粒度 phase, 仅作 CSV 标注. 详细 phase 在 swing_leg_diag, 这里精简版
        # 不订阅, 避免列爆炸.
        if swing_msg is None:
            return "UNKNOWN", -1
        if bool(swing_msg.support_leg):
            return "SUPPORT", PHASE_ID_MAP["SUPPORT"]
        # 非 support: 进入 swing 序列
        normal_limit = float(getattr(swing_msg, "desired_normal_force_limit", 0.0))
        if normal_limit > 0.5:
            # PRELOAD/COMPLIANT 阶段都给了 normal_force_limit
            return "PRELOAD_OR_COMPLIANT", PHASE_ID_MAP["PRELOAD_COMPRESS"]
        skirt = float(getattr(swing_msg, "skirt_compression_target", 0.0))
        if skirt < 0.05:
            return "DETACH_OR_LIFT", PHASE_ID_MAP["DETACH_SLIDE"]
        return "TANGENTIAL_ALIGN", PHASE_ID_MAP["TANGENTIAL_ALIGN"]

    def _sample_row(self, phase_label):
        with self._lock:
            state = self._latest_mission_state or ""
            mission_active = self._latest_mission_active
            body = self._latest_body_reference
            est = self._latest_estimated_state
            swings = dict(self._latest_swing_targets)
            fan_rpm = list(self._latest_fan_rpm)
            fan_curr = list(self._latest_fan_currents)

        wall_time = time.time()
        ros_time = rospy.Time.now().to_sec()
        elapsed = self._rel_now()

        body_x = body_y = body_z = 0.0
        body_vx = body_vy = body_vz = 0.0
        if body is not None:
            body_x = float(body.pose.position.x)
            body_y = float(body.pose.position.y)
            body_z = float(body.pose.position.z)
            body_vx = float(body.twist.linear.x)
            body_vy = float(body.twist.linear.y)
            body_vz = float(body.twist.linear.z)

        row = [
            "%.6f" % wall_time, "%.6f" % ros_time, "%.4f" % elapsed,
            state, int(bool(mission_active)), phase_label,
            "%.4f" % body_x, "%.4f" % body_y, "%.4f" % body_z,
            "%.4f" % body_vx, "%.4f" % body_vy, "%.4f" % body_vz,
        ]

        for leg_index, leg in enumerate(LEG_NAMES):
            cmd = swings.get(leg)
            phase_name, phase_id = self._phase_label_from_swing(leg_index, cmd)
            cmd_x = cmd_y = cmd_z = 0.0
            cmd_support = 0
            if cmd is not None:
                cmd_x = float(cmd.center.x)
                cmd_y = float(cmd.center.y)
                cmd_z = float(cmd.center.z)
                cmd_support = int(bool(cmd.support_leg))

            ujc_z = 0.0
            if est is not None and len(est.universal_joint_center_positions) > leg_index:
                ujc_z = float(est.universal_joint_center_positions[leg_index].z)

            def mask(name):
                if est is None:
                    return 0
                values = getattr(est, name, [])
                if leg_index >= len(values):
                    return 0
                return int(bool(values[leg_index]))

            row.extend([
                phase_name, phase_id,
                "%.4f" % cmd_x, "%.4f" % cmd_y, "%.4f" % cmd_z,
                cmd_support,
                "%.4f" % ujc_z,
                mask("attachment_ready_mask"),
                mask("adhesion_mask"),
                mask("measured_contact_mask"),
                "%.2f" % float(fan_rpm[leg_index]),
                "%.4f" % float(fan_curr[leg_index]),
            ])

            # swing 周期统计 (cmd_support: True->False->True 计一次)
            prev = self._prev_support.get(leg)
            if prev is not None and prev is True and cmd_support == 0:
                # 进入 swing
                pass
            elif prev is not None and prev is False and cmd_support == 1:
                # 回到 support, 计为完成一次 swing
                self._swing_count[leg] += 1
            self._prev_support[leg] = bool(cmd_support)

        return row

    # ------------------------------------------------------------------ #
    # 主流程
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
                # 阻塞等用户回车
                input(
                    "\n>>> 请确认机器人已贴墙 (或在测试台上稳定就位), "
                    "随后按回车开始 STICK -> CLIMB. (Ctrl+C 取消)\n>>> "
                )
            except (EOFError, KeyboardInterrupt):
                rospy.loginfo("test_crawl_gait: user cancelled before start.")
                return 0

        self._open_csv()

        self._publish_start()
        if not self._wait_state(STATE_STICK, self.args.stick_timeout_s):
            self._publish_pause()
            return 4
        if not self._wait_state(STATE_CLIMB, self.args.climb_timeout_s):
            self._publish_pause()
            return 5

        rospy.loginfo(
            "test_crawl_gait: CLIMB reached. recording for %.1fs at %.1f Hz...",
            float(self.args.duration), float(self.args.log_rate_hz),
        )

        rate = rospy.Rate(max(1.0, float(self.args.log_rate_hz)))
        end_t = time.time() + max(0.0, float(self.args.duration))
        last_status_log = 0.0
        while not rospy.is_shutdown() and time.time() < end_t:
            with self._lock:
                state = self._latest_mission_state
            if state == STATE_FAULT:
                rospy.logerr("test_crawl_gait: FAULT during CLIMB; aborting.")
                break
            row = self._sample_row(state or "")
            try:
                self._csv_writer.writerow(row)
            except (IOError, ValueError):
                pass
            self._sample_count += 1

            now = time.time()
            if now - last_status_log > 5.0:
                with self._lock:
                    swing_summary = ", ".join(
                        "%s=%d" % (leg, self._swing_count[leg])
                        for leg in LEG_NAMES
                    )
                rospy.loginfo(
                    "test_crawl_gait: t=%.1fs samples=%d swings(%s)",
                    self._rel_now(), self._sample_count, swing_summary,
                )
                last_status_log = now
            rate.sleep()

        self._publish_pause()
        # 给 mission_supervisor 一点时间转 PAUSE
        wait_pause_until = time.time() + 3.0
        while not rospy.is_shutdown() and time.time() < wait_pause_until:
            with self._lock:
                if not self._latest_mission_active:
                    break
            rospy.sleep(0.1)

        self._close_csv()
        self._print_summary()
        return 0 if not self._fault_seen else 6

    def _print_summary(self):
        rospy.loginfo("=" * 70)
        rospy.loginfo("test_crawl_gait: summary")
        rospy.loginfo("  csv: %s", self._csv_path)
        rospy.loginfo("  samples: %d", self._sample_count)
        rospy.loginfo("  state transitions:")
        for rel, state in self._state_history:
            rospy.loginfo("    t=+%.2fs  -> %s", rel, state)
        rospy.loginfo("  completed swing cycles per leg:")
        for leg in LEG_NAMES:
            rospy.loginfo("    %s: %d", leg, self._swing_count[leg])
        rospy.loginfo("=" * 70)


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description="整机 crawl 步态半交互测试",
    )
    parser.add_argument(
        "--duration", type=float, default=60.0,
        help="CLIMB 阶段记录时长 (s, 默认 60)",
    )
    parser.add_argument(
        "--log-rate-hz", type=float, default=50.0,
        help="CSV 采样率 (Hz, 默认 50)",
    )
    parser.add_argument(
        "--hold-check-s", type=float, default=3.0,
        help="INIT 稳定性观察时长 (s, 默认 3)",
    )
    parser.add_argument(
        "--hold-tol-mm", type=float, default=20.0,
        help="INIT 阶段 UJC z 偏离 operating 的允许误差 (mm, 默认 20). "
             "经验值: <10 mm 通常需要良好的机械校准; 实机 ~12 mm 误差常见.",
    )
    parser.add_argument(
        "--allow-init-mismatch", action="store_true",
        help="即使 INIT hold 检查失败 (UJC 偏差 / support_leg 异常) 也继续, "
             "仅打印警告. 用于已知机械误差场景的强制测试.",
    )
    parser.add_argument(
        "--stick-timeout-s", type=float, default=15.0,
        help="STICK 等待超时 (s, 默认 15)",
    )
    parser.add_argument(
        "--climb-timeout-s", type=float, default=30.0,
        help="CLIMB 等待超时 (s, 默认 30)",
    )
    parser.add_argument(
        "--stream-timeout-s", type=float, default=10.0,
        help="上游话题就绪超时 (s, 默认 10)",
    )
    parser.add_argument(
        "--output-dir", type=str,
        default=os.path.expanduser("~/climbing_ws/test_logs"),
        help="CSV 输出目录",
    )
    parser.add_argument(
        "--no-confirm", action="store_true",
        help="跳过手动回车确认 (用于无人值守批量测试)",
    )
    return parser.parse_args(argv)


def main():
    # 先解析参数 (使用 rospy.myargv 过滤 ROS remap), 这样 --help 不会触发
    # rospy.init_node 去连 ROS master.
    cli_args = rospy.myargv(argv=sys.argv)[1:]
    args = parse_args(cli_args)
    rospy.init_node("test_crawl_gait", anonymous=False)
    tester = CrawlGaitTester(args)
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
        tester._close_csv()
    finally:
        # 确保 CSV 关掉
        tester._close_csv()
    sys.exit(code)


if __name__ == "__main__":
    main()
