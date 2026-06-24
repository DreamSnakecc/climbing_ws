#!/usr/bin/env python3
"""Resumable, operator-gated Dynamixel position tuning workflow.

Stages are deliberately separate because direct step tests and crawl tests use
different bringup modes.  Each stage records its inputs and results in a JSON
session file so the robot may be restarted safely between stages.
"""

from __future__ import print_function

import argparse
import csv
import glob
import json
import math
import os
import shutil
import subprocess
import sys
import time

import rospy
import yaml

from dynamixel_control.srv import GetPositionTuning, SetPositionTuning


DEFAULT_MOTOR_IDS = [11, 1, 2, 12, 3, 4, 13, 5, 6, 14, 7, 8]
MOVING_PHASES = set(["LIFT", "TRANSFER", "PRELOAD", "ADMIT", "LIFT_SWING"])
SERVO_ENDPOINT_PHASES = [(5, "LIFT"), (6, "TRANSFER"), (2, "PRELOAD")]


def percentile(values, fraction):
    values = sorted([float(value) for value in values])
    if not values:
        return None
    index = int(round((len(values) - 1) * float(fraction)))
    return values[index]


def mean(values):
    values = [float(value) for value in values if value is not None]
    return sum(values) / float(len(values)) if values else None


def ensure_dir(path):
    if not os.path.isdir(path):
        os.makedirs(path)


def load_yaml(path):
    with open(path, "r") as stream:
        return yaml.safe_load(stream) or {}


def write_yaml(path, data):
    with open(path, "w") as stream:
        yaml.safe_dump(data, stream, default_flow_style=False, sort_keys=False)


def load_session(path):
    with open(path, "r") as stream:
        return json.load(stream)


def save_session(path, session):
    session["updated_at"] = time.strftime("%Y-%m-%dT%H:%M:%S")
    temporary = path + ".tmp"
    with open(temporary, "w") as stream:
        json.dump(session, stream, indent=2, sort_keys=True)
        stream.write("\n")
    os.rename(temporary, path)


def confirm(args, prompt):
    if args.no_confirm:
        return True
    try:
        answer = input("\n%s [y/N]: " % prompt).strip().lower()
    except (EOFError, KeyboardInterrupt):
        return False
    return answer in ("y", "yes")


def tuning_from_response(response):
    return {
        "operating_mode": int(response.operating_mode),
        "pwm_limit": int(response.pwm_limit),
        "current_limit": int(response.current_limit),
        "p": int(response.position_p_gain),
        "i": int(response.position_i_gain),
        "d": int(response.position_d_gain),
        "velocity": int(response.profile_velocity),
        "acceleration": int(response.profile_acceleration),
    }


def metric_score(metrics, step_ticks):
    if not metrics or not metrics.get("hard_safe"):
        return None
    step = max(1.0, abs(float(step_ticks)))
    response = float(metrics.get("response_error_tick", step)) / step
    settling = float(metrics.get("settling_time_s", 0.8)) / 0.8
    overshoot = float(metrics.get("overshoot_tick", 0.0)) / step
    current = float(metrics.get("current_ratio", 1.0))
    return 0.55 * response + 0.30 * settling + 0.10 * overshoot + 0.05 * current


def parse_step_csv(path, pass_error_ticks, feedback_age_limit_s):
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    rows = [row for row in rows if row.get("phase") in ("ramp_out", "settle")]
    if not rows:
        return {"safe": False, "reason": "no_ramp_samples"}

    start = min(float(row["elapsed_s"]) for row in rows)
    samples = []
    for row in rows:
        rel = float(row["elapsed_s"]) - start
        samples.append({
            "t": rel,
            "target": float(row["target_tick"]),
            "actual": float(row["actual_tick"]),
            "error": float(row["error_tick"]),
            "current_a": abs(float(row["current_a"])),
            "feedback_age_s": float(row.get("feedback_age_s", "0") or 0.0),
        })
    samples.sort(key=lambda sample: sample["t"])
    target = samples[0]["target"]
    initial = samples[0]["actual"]
    direction = 1.0 if target >= initial else -1.0
    response_sample = min(samples, key=lambda sample: abs(sample["t"] - 0.20))
    settling_time = None
    for index, sample in enumerate(samples):
        if abs(sample["error"]) <= pass_error_ticks and all(
                abs(later["error"]) <= pass_error_ticks for later in samples[index:]):
            settling_time = sample["t"]
            break
    overshoot = max([max(0.0, direction * (sample["actual"] - target)) for sample in samples])
    max_current = max(sample["current_a"] for sample in samples)
    max_age = max(sample["feedback_age_s"] for sample in samples)
    endpoint_pass = abs(samples[-1]["error"]) <= pass_error_ticks
    feedback_safe = max_age <= feedback_age_limit_s
    return {
        "safe": endpoint_pass and feedback_safe,
        "endpoint_pass": endpoint_pass,
        "feedback_safe": feedback_safe,
        "final_error_tick": abs(samples[-1]["error"]),
        "response_error_tick": abs(response_sample["error"]),
        "settling_time_s": settling_time,
        "overshoot_tick": overshoot,
        "max_current_a": max_current,
        "max_feedback_age_s": max_age,
        "sample_count": len(samples),
        "csv": path,
    }


def parse_single_leg_csv(path, pass_error_ticks):
    """Summarize the independent servo endpoint gate from one leg swing CSV."""
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
    required = [
        "servo_gate_enabled", "servo_timed_out", "servo_last_pass_phase_id",
        "servo_last_pass_sequence", "servo_wait_s",
    ]
    if not rows or any(field not in rows[0] for field in required):
        return {"passed": False, "reason": "missing_servo_endpoint_diagnostics", "csv": path}

    def value(row, field, default=0.0):
        try:
            return float(row.get(field, default) or default)
        except (TypeError, ValueError):
            return float(default)

    endpoint_errors = {}
    last_sequence = 0
    for row in rows:
        if value(row, "servo_gate_enabled") < 0.5:
            continue
        sequence = int(round(value(row, "servo_last_pass_sequence")))
        phase_id = int(round(value(row, "servo_last_pass_phase_id", -1.0)))
        if sequence <= last_sequence or phase_id not in [item[0] for item in SERVO_ENDPOINT_PHASES]:
            continue
        errors = [
            value(row, "servo_last_pass_error_joint1_tick"),
            value(row, "servo_last_pass_error_joint2_tick"),
            value(row, "servo_last_pass_error_joint3_tick"),
        ]
        if all(abs(error) <= pass_error_ticks for error in errors):
            endpoint_errors[phase_id] = errors
            last_sequence = sequence

    passed_phase_ids = []
    for phase_id, _ in SERVO_ENDPOINT_PHASES:
        if phase_id not in endpoint_errors:
            break
        passed_phase_ids.append(phase_id)
    waiting_rows = [row for row in rows if value(row, "servo_wait_s") > 0.0]
    terminal_row = waiting_rows[-1] if waiting_rows else rows[-1]
    terminal_errors = [
        value(terminal_row, "servo_error_joint1_tick"),
        value(terminal_row, "servo_error_joint2_tick"),
        value(terminal_row, "servo_error_joint3_tick"),
    ]
    timed_out = any(value(row, "servo_timed_out") >= 0.5 for row in rows)
    endpoint_count = len(passed_phase_ids)
    return {
        "csv": path,
        "passed": endpoint_count == len(SERVO_ENDPOINT_PHASES) and not timed_out,
        "endpoint_count": endpoint_count,
        "passed_phases": [phase_name for phase_id, phase_name in SERVO_ENDPOINT_PHASES if phase_id in passed_phase_ids],
        "endpoint_errors_tick": {
            phase_name: endpoint_errors[phase_id]
            for phase_id, phase_name in SERVO_ENDPOINT_PHASES if phase_id in endpoint_errors
        },
        "timed_out": timed_out,
        "terminal_phase_id": int(round(value(terminal_row, "servo_phase_id", -1.0))),
        "terminal_errors_tick": terminal_errors,
        "terminal_max_abs_error_tick": max(abs(error) for error in terminal_errors),
    }


def leg_endpoint_improves(candidate, baseline):
    if candidate.get("passed"):
        return True
    if candidate.get("timed_out") and not baseline.get("timed_out"):
        return False
    if candidate.get("endpoint_count", 0) != baseline.get("endpoint_count", 0):
        return candidate.get("endpoint_count", 0) > baseline.get("endpoint_count", 0)
    return candidate.get("terminal_max_abs_error_tick", float("inf")) < baseline.get("terminal_max_abs_error_tick", float("inf"))


def combine_step_metrics(direction_metrics, current_limit_a, step_ticks, pass_error_ticks):
    if len(direction_metrics) != 2:
        return {"safe": False, "reason": "missing_direction"}
    max_current = max(metric["max_current_a"] for metric in direction_metrics)
    max_final = max(metric["final_error_tick"] for metric in direction_metrics)
    max_age = max(metric["max_feedback_age_s"] for metric in direction_metrics)
    result = {
        "directions": direction_metrics,
        "final_error_tick": max_final,
        "response_error_tick": mean([metric["response_error_tick"] for metric in direction_metrics]),
        "settling_time_s": mean([
            metric["settling_time_s"] if metric["settling_time_s"] is not None else 0.8
            for metric in direction_metrics
        ]),
        "overshoot_tick": max(metric["overshoot_tick"] for metric in direction_metrics),
        "max_current_a": max_current,
        "max_feedback_age_s": max_age,
        "current_ratio": max_current / max(current_limit_a, 1e-6),
    }
    result["feedback_safe"] = all(metric.get("feedback_safe", metric["safe"]) for metric in direction_metrics)
    result["current_safe"] = result["current_ratio"] < 0.80
    result["endpoint_pass"] = all(metric.get("endpoint_pass", metric["safe"]) for metric in direction_metrics)
    result["hard_safe"] = result["feedback_safe"] and result["current_safe"]
    result["safe"] = result["hard_safe"] and result["endpoint_pass"]
    result["score"] = metric_score(result, step_ticks)
    return result


def board_by_motor(config):
    result = {}
    for board_name in ("left_board", "right_board"):
        for motor_id in config.get(board_name, {}).get("motor_ids", []):
            result[int(motor_id)] = board_name
    return result


def leg_by_motor(config):
    result = {}
    for leg_name, leg_config in config.get("legs", {}).items():
        for motor_id in leg_config.get("motor_ids", []):
            result[int(motor_id)] = str(leg_name)
    return result


def motor_ids_for_leg(config, leg_name):
    leg_config = config.get("legs", {}).get(str(leg_name), {})
    return [int(value) for value in leg_config.get("motor_ids", [])]


def failed_leg_motor_id(result, motor_ids, pass_error_ticks):
    errors = list(result.get("terminal_errors_tick", []))
    if len(motor_ids) != 3 or len(errors) != 3:
        return None
    failing = [
        (abs(float(error)), int(motor_id))
        for motor_id, error in zip(motor_ids, errors)
        if abs(float(error)) > float(pass_error_ticks)
    ]
    return max(failing)[1] if failing else None


def leg_endpoint_candidates(original, active):
    """Bounded P/Profile candidates relative to the original motor settings."""
    candidates = []
    profile = dict(active)
    profile["velocity"] = int(round(float(original["velocity"]) * 1.25))
    profile["acceleration"] = int(round(float(original["acceleration"]) * 1.25))
    if profile != active:
        candidates.append(("profile_125", profile))
    for percentage in (110, 120, 130):
        candidate = dict(active)
        candidate["p"] = int(round(float(original["p"]) * percentage / 100.0))
        if candidate != active:
            candidates.append(("p_%d" % percentage, candidate))
    return candidates


def transformed_ujc_error(row, leg_name, config):
    leg_config = config.get("legs", {}).get(leg_name, {})
    gait = config.get("gait_controller", {})
    yaw = math.radians(float(leg_config.get("hip_yaw_deg", 0.0)))
    base_radius = float(gait.get("base_radius", 203.06)) / 1000.0
    nominal_x = float(gait.get("nominal_x", 118.75)) / 1000.0
    nominal_y = float(gait.get("nominal_y", 0.0)) / 1000.0
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    hip_x = base_radius * cos_yaw
    hip_y = base_radius * sin_yaw
    ujc_x = float(row["%s_ujc_x" % leg_name])
    ujc_y = float(row["%s_ujc_y" % leg_name])
    ujc_z = float(row["%s_ujc_z" % leg_name])
    dx = ujc_x - hip_x
    dy = ujc_y - hip_y
    dx_leg = cos_yaw * dx + sin_yaw * dy
    dy_leg = -sin_yaw * dx + cos_yaw * dy
    actual_x = cos_yaw * (dx_leg - nominal_x) - sin_yaw * (dy_leg - nominal_y)
    actual_y = sin_yaw * (dx_leg - nominal_x) + cos_yaw * (dy_leg - nominal_y)
    error_x = actual_x - float(row["%s_cmd_x" % leg_name])
    error_y = actual_y - float(row["%s_cmd_y" % leg_name])
    error_z = ujc_z - float(row["%s_cmd_z" % leg_name])
    return math.hypot(error_x, error_y), abs(error_z), math.sqrt(error_x * error_x + error_y * error_y + error_z * error_z)


def parse_crawl_metrics(path, config):
    motor_legs = leg_by_motor(config)
    motor_errors = {motor_id: [] for motor_id in motor_legs}
    motor_currents = {motor_id: [] for motor_id in motor_legs}
    leg_errors = {leg_name: [] for leg_name in config.get("legs", {})}
    fault_seen = False
    with open(path, newline="") as stream:
        for row in csv.DictReader(stream):
            fault_seen = fault_seen or row.get("mission_state") == "FAULT"
            for motor_id, leg_name in motor_legs.items():
                if row.get("%s_phase" % leg_name) not in MOVING_PHASES:
                    continue
                error = row.get("motor_%d_error_tick" % motor_id, "")
                current = row.get("motor_%d_actual_current_a" % motor_id, "")
                if error != "":
                    motor_errors[motor_id].append(abs(float(error)))
                if current != "":
                    motor_currents[motor_id].append(abs(float(current)))
            for leg_name in leg_errors:
                if row.get("%s_phase" % leg_name) in MOVING_PHASES:
                    try:
                        leg_errors[leg_name].append(transformed_ujc_error(row, leg_name, config))
                    except (KeyError, TypeError, ValueError):
                        pass

    return {
        "csv": path,
        "fault_seen": fault_seen,
        "motors": {
            str(motor_id): {
                "p95_abs_error_tick": percentile(errors, 0.95),
                "p95_current_a": percentile(motor_currents[motor_id], 0.95),
            }
            for motor_id, errors in motor_errors.items()
        },
        "legs": {
            leg_name: {
                "p95_tangent_error_mm": None if not errors else percentile([item[0] * 1000.0 for item in errors], 0.95),
                "p95_normal_error_mm": None if not errors else percentile([item[1] * 1000.0 for item in errors], 0.95),
                "p95_total_error_mm": None if not errors else percentile([item[2] * 1000.0 for item in errors], 0.95),
            }
            for leg_name, errors in leg_errors.items()
        },
    }


def compare_crawl_metrics(baseline, candidate):
    reasons = []
    if candidate.get("fault_seen"):
        reasons.append("candidate crawl entered FAULT")
    for motor_id, baseline_values in baseline.get("motors", {}).items():
        baseline_p95 = baseline_values.get("p95_abs_error_tick")
        candidate_p95 = candidate.get("motors", {}).get(motor_id, {}).get("p95_abs_error_tick")
        if baseline_p95 is None or candidate_p95 is None:
            reasons.append("motor %s has insufficient moving samples" % motor_id)
        elif candidate_p95 > baseline_p95 * 1.05 + 5.0:
            reasons.append("motor %s p95 error worsened %.1f -> %.1f tick" % (motor_id, baseline_p95, candidate_p95))
    for leg_name, baseline_values in baseline.get("legs", {}).items():
        baseline_p95 = baseline_values.get("p95_total_error_mm")
        candidate_p95 = candidate.get("legs", {}).get(leg_name, {}).get("p95_total_error_mm")
        if baseline_p95 is not None and candidate_p95 is not None and candidate_p95 > baseline_p95 * 1.10 + 1.0:
            reasons.append("%s UJC p95 error worsened %.2f -> %.2f mm" % (leg_name, baseline_p95, candidate_p95))
    return {"passed": not reasons, "reasons": reasons}


class DxlAutoTuner(object):
    def __init__(self, args):
        self.args = args
        self.config = load_yaml(args.robot_config)
        self.board_by_motor = board_by_motor(self.config)
        self.motor_ids = [int(value) for value in args.motor_ids.split(",") if value.strip()]
        self.current_lsb_ma = float(rospy.get_param("/dynamixel_telemetry/current_lsb_ma", 2.69))
        self.get_clients = {}
        self.set_clients = {}
        for board_name in ("left_board", "right_board"):
            namespace = "/jetson/%s" % board_name
            self.get_clients[board_name] = rospy.ServiceProxy(namespace + "/get_position_tuning", GetPositionTuning)
            self.set_clients[board_name] = rospy.ServiceProxy(namespace + "/set_position_tuning", SetPositionTuning)

    def _confirm(self, prompt):
        return confirm(self.args, prompt)

    def _wait_services(self):
        for board_name in ("left_board", "right_board"):
            namespace = "/jetson/%s" % board_name
            try:
                rospy.wait_for_service(namespace + "/get_position_tuning", timeout=self.args.stream_timeout_s)
                rospy.wait_for_service(namespace + "/set_position_tuning", timeout=self.args.stream_timeout_s)
            except rospy.ROSException as exc:
                raise RuntimeError("%s tuning services unavailable: %s" % (board_name, exc))

    def _get_tuning(self, motor_id):
        board_name = self.board_by_motor.get(int(motor_id))
        if board_name is None:
            raise RuntimeError("motor %d has no configured board" % motor_id)
        response = self.get_clients[board_name](int(motor_id))
        if not response.success:
            raise RuntimeError("motor %d tuning read failed: %s" % (motor_id, response.message))
        return tuning_from_response(response)

    def _set_tuning(self, motor_id, tuning):
        board_name = self.board_by_motor[int(motor_id)]
        response = self.set_clients[board_name](
            int(motor_id), int(tuning["p"]), int(tuning["i"]), int(tuning["d"]),
            int(tuning["velocity"]), int(tuning["acceleration"]),
        )
        if not response.success:
            raise RuntimeError("motor %d tuning write failed: %s" % (motor_id, response.message))

    def _run_step(self, motor_id, step_ticks, label, session_dir):
        output_dir = os.path.join(session_dir, "bench", "id_%d" % motor_id, label)
        ensure_dir(output_dir)
        command = [
            "rosrun", "climbing_hw_bridge", "test_dxl_position_step.py",
            "--motor-ids", str(motor_id),
            "--step-ticks", str(int(step_ticks)),
            "--ramp-steps", "1",
            "--directions", "+",
            "--hold-s", "0.6",
            "--settle-s", "0.2",
            "--pass-error-ticks", str(self.args.pass_error_ticks),
            "--sample-rate-hz", "50",
            "--no-confirm",
            "--output-dir", output_dir,
        ]
        result = subprocess.call(command)
        paths = glob.glob(os.path.join(output_dir, "dxl_position_step_*.csv"))
        if not paths:
            raise RuntimeError("position step produced no CSV for motor %d" % motor_id)
        metrics = parse_step_csv(max(paths, key=os.path.getmtime), self.args.pass_error_ticks, 0.15)
        metrics["process_returncode"] = int(result)
        return metrics

    def _run_trial(self, motor_id, tuning, step_ticks, label, session_dir):
        self._set_tuning(motor_id, tuning)
        rospy.sleep(0.10)
        positive = self._run_step(motor_id, abs(step_ticks), label + "_plus", session_dir)
        negative = self._run_step(motor_id, -abs(step_ticks), label + "_minus", session_dir)
        current_limit_a = float(tuning["current_limit"]) * self.current_lsb_ma / 1000.0
        combined = combine_step_metrics(
            [positive, negative], current_limit_a, step_ticks, self.args.pass_error_ticks,
        )
        combined["tuning"] = dict(tuning)
        combined["label"] = label
        return combined

    @staticmethod
    def _improves(candidate, baseline):
        candidate_score = candidate.get("score")
        baseline_score = baseline.get("score")
        return (
            candidate.get("safe") and candidate_score is not None and baseline_score is not None and
            candidate_score <= baseline_score * 0.90
        )

    def _tune_motor(self, motor_id, baseline_tuning, session_dir):
        try:
            baseline = self._run_trial(motor_id, baseline_tuning, self.args.step_ticks, "baseline", session_dir)
            if not baseline.get("hard_safe"):
                raise RuntimeError("motor %d baseline exceeded the hard current or feedback safety limit" % motor_id)
            best = baseline
            trials = [baseline]

            profile_candidate = dict(baseline_tuning)
            profile_candidate["velocity"] = int(round(profile_candidate["velocity"] * 1.25))
            profile_candidate["acceleration"] = int(round(profile_candidate["acceleration"] * 1.25))
            profile_trial = self._run_trial(motor_id, profile_candidate, self.args.step_ticks, "profile_125", session_dir)
            trials.append(profile_trial)
            if self._improves(profile_trial, baseline):
                best = profile_trial

            profile_source = dict(best["tuning"])
            for percentage in (110, 120, 130):
                candidate = dict(profile_source)
                candidate["p"] = int(round(baseline_tuning["p"] * percentage / 100.0))
                trial = self._run_trial(motor_id, candidate, self.args.step_ticks, "p_%d" % percentage, session_dir)
                trials.append(trial)
                if self._improves(trial, baseline) and (best.get("score") is None or trial["score"] < best["score"]):
                    best = trial

            if best.get("overshoot_tick", 0.0) > abs(float(self.args.step_ticks)) * 0.10:
                for d_gain in (20, 40):
                    candidate = dict(best["tuning"])
                    candidate["d"] = d_gain
                    trial = self._run_trial(motor_id, candidate, self.args.step_ticks, "d_%d" % d_gain, session_dir)
                    trials.append(trial)
                    if self._improves(trial, baseline) and trial["score"] < best["score"]:
                        best = trial

            self._set_tuning(motor_id, best["tuning"])
            return {"baseline": baseline, "trials": trials, "selected": best}
        except Exception:
            try:
                self._set_tuning(motor_id, baseline_tuning)
            except Exception:
                rospy.logerr("tune_dxl_position: failed to restore motor %d baseline tuning", motor_id)
            raise

    def _validate_extended(self, motor_id, result, session_dir):
        baseline = result["baseline"]
        candidate = result["selected"]
        baseline_trial = self._run_trial(
            motor_id, baseline["tuning"], self.args.extended_step_ticks, "extended_baseline", session_dir,
        )
        candidate_trial = self._run_trial(
            motor_id, candidate["tuning"], self.args.extended_step_ticks, "extended_candidate", session_dir,
        )
        result["extended"] = {"baseline": baseline_trial, "candidate": candidate_trial}
        if not candidate_trial.get("safe") or (
                baseline_trial.get("score") is not None and candidate_trial.get("score") is not None and
                candidate_trial["score"] > baseline_trial["score"]):
            self._set_tuning(motor_id, baseline["tuning"])
            result["selected"] = baseline
            result["extended_reverted"] = True
        else:
            self._set_tuning(motor_id, candidate["tuning"])


def new_session(args):
    output_dir = os.path.abspath(os.path.expanduser(args.output_dir))
    ensure_dir(output_dir)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    session_dir = os.path.join(output_dir, "dxl_autotune_%s" % stamp)
    ensure_dir(session_dir)
    session_path = os.path.join(session_dir, "session.json")
    session = {
        "created_at": time.strftime("%Y-%m-%dT%H:%M:%S"),
        "session_dir": session_dir,
        "robot_config": os.path.abspath(args.robot_config),
        "motor_ids": [int(value) for value in args.motor_ids.split(",") if value.strip()],
        "stage": "created",
    }
    save_session(session_path, session)
    return session_path, session


def run_crawl(args, session, label):
    output_dir = os.path.join(session["session_dir"], label)
    ensure_dir(output_dir)
    command = [
        "rosrun", "climbing_control_core", "test_crawl_gait_with_fan.py",
        "--duration", str(args.crawl_duration_s),
        "--log-rate-hz", "50",
        "--output-dir", output_dir,
        "--no-confirm",
    ]
    returncode = subprocess.call(command)
    paths = glob.glob(os.path.join(output_dir, "crawl_gait_with_fan_*.csv"))
    if not paths:
        raise RuntimeError("crawl validation produced no CSV")
    return max(paths, key=os.path.getmtime), int(returncode)


def run_single_leg_test(args, session, label):
    output_dir = os.path.join(session["session_dir"], "leg_endpoint", label)
    ensure_dir(output_dir)
    command = [
        "rosrun", "climbing_control_core", "test_single_leg_swing.py",
        "--leg", args.leg,
        "--cycles", str(args.leg_cycles),
        "--swing-wait", str(args.leg_swing_wait_s),
        "--hold-before", str(args.leg_hold_before_s),
        "--hold-after", str(args.leg_hold_after_s),
        "--output-dir", output_dir,
    ]
    returncode = subprocess.call(command)
    paths = glob.glob(os.path.join(output_dir, "single_swing_%s_*.csv" % args.leg))
    if not paths:
        raise RuntimeError("single-leg endpoint test produced no CSV")
    metrics = parse_single_leg_csv(max(paths, key=os.path.getmtime), args.pass_error_ticks)
    metrics["process_returncode"] = int(returncode)
    metrics["passed"] = bool(metrics.get("passed")) and returncode == 0
    if returncode not in (0, 4):
        raise RuntimeError("single-leg endpoint test returned %d" % returncode)
    return metrics


def yaml_mapping_lines(name, values, profile=False):
    lines = ["  %s:\n" % name]
    for motor_id in sorted([int(value) for value in values]):
        value = values[str(motor_id)] if str(motor_id) in values else values[motor_id]
        if profile:
            lines.append("    \"%d\": {velocity: %d, acceleration: %d}\n" % (
                motor_id, int(value["velocity"]), int(value["acceleration"])))
        else:
            lines.append("    \"%d\": {p: %d, i: %d, d: %d}\n" % (
                motor_id, int(value["p"]), int(value["i"]), int(value["d"])))
    return lines


def replace_board_mapping(lines, board_name, mapping_name, replacement):
    board_start = next(index for index, line in enumerate(lines) if line.strip() == board_name + ":" and not line.startswith(" "))
    board_end = len(lines)
    for index in range(board_start + 1, len(lines)):
        if lines[index] and not lines[index].startswith(" ") and lines[index].rstrip().endswith(":"):
            board_end = index
            break
    mapping_start = None
    for index in range(board_start + 1, board_end):
        if lines[index].strip() == mapping_name + ":":
            mapping_start = index
            break
    if mapping_start is None:
        lines[board_end:board_end] = replacement
        return lines
    mapping_end = board_end
    for index in range(mapping_start + 1, board_end):
        if lines[index].startswith("  ") and not lines[index].startswith("    ") and lines[index].strip():
            mapping_end = index
            break
    lines[mapping_start:mapping_end] = replacement
    return lines


def write_committed_tuning(robot_config, candidates, session_dir):
    backup = os.path.join(session_dir, "robot.yaml.before_autotune")
    shutil.copy2(robot_config, backup)
    with open(robot_config, "r") as stream:
        lines = stream.readlines()
    for board_name in ("left_board", "right_board"):
        board_candidates = candidates.get(board_name, {})
        gains = {str(motor_id): {"p": value["p"], "i": value["i"], "d": value["d"]}
                 for motor_id, value in board_candidates.items()}
        profiles = {str(motor_id): {"velocity": value["velocity"], "acceleration": value["acceleration"]}
                    for motor_id, value in board_candidates.items()}
        lines = replace_board_mapping(lines, board_name, "position_gains", yaml_mapping_lines("position_gains", gains))
        lines = replace_board_mapping(lines, board_name, "position_profiles", yaml_mapping_lines("position_profiles", profiles, profile=True))
    temporary = robot_config + ".autotune.tmp"
    with open(temporary, "w") as stream:
        stream.writelines(lines)
    load_yaml(temporary)
    os.rename(temporary, robot_config)
    return backup


def candidate_override(candidates):
    result = {}
    for board_name in ("left_board", "right_board"):
        gains = {}
        profiles = {}
        for motor_id, tuning in candidates.get(board_name, {}).items():
            gains[str(motor_id)] = {"p": int(tuning["p"]), "i": int(tuning["i"]), "d": int(tuning["d"])}
            profiles[str(motor_id)] = {
                "velocity": int(tuning["velocity"]),
                "acceleration": int(tuning["acceleration"]),
            }
        result[board_name] = {"position_gains": gains, "position_profiles": profiles}
    return result


def leg_endpoint_candidates_by_board(tuner, active_tuning):
    candidates = {"left_board": {}, "right_board": {}}
    for motor_id in sorted(tuner.board_by_motor):
        motor_key = str(motor_id)
        tuning = active_tuning.get(motor_key)
        if tuning is None:
            tuning = tuner._get_tuning(motor_id)
        board_name = tuner.board_by_motor[motor_id]
        candidates[board_name][motor_key] = dict(tuning)
    return candidates


def run_leg_endpoint_stage(args, session_path, session):
    rospy.init_node("tune_dxl_leg_endpoint", anonymous=False)
    tuner = DxlAutoTuner(args)
    tuner._wait_services()
    motor_ids = motor_ids_for_leg(tuner.config, args.leg)
    if len(motor_ids) != 3:
        raise RuntimeError("leg %s must define exactly three motor_ids" % args.leg)

    endpoint = session.get("leg_endpoint")
    if endpoint is None:
        if not tuner._confirm(
                "Robot is supported and normal bringup has servo_tracking_gate_enabled:=true. "
                "Run the single-leg test/tune/test loop for %s now?" % args.leg):
            return 0
        original_tuning = {}
        for motor_id in motor_ids:
            tuning = tuner._get_tuning(motor_id)
            if tuning["operating_mode"] != 3:
                raise RuntimeError("motor %d must be in Position Control Mode (3), got %d" % (
                    motor_id, tuning["operating_mode"]))
            original_tuning[str(motor_id)] = tuning
        endpoint = {
            "leg": args.leg,
            "motor_ids": motor_ids,
            "original_tuning": original_tuning,
            "active_tuning": dict(original_tuning),
            "trials": [],
            "status": "running",
        }
        session["motor_ids"] = list(motor_ids)
        session["leg_endpoint"] = endpoint
        session["stage"] = "leg_endpoint_running"
        save_session(session_path, session)
    elif endpoint.get("leg") != args.leg:
        raise RuntimeError("session is for leg %s, not %s" % (endpoint.get("leg"), args.leg))

    for motor_id, tuning in endpoint["active_tuning"].items():
        tuner._set_tuning(int(motor_id), tuning)

    current = endpoint.get("current_metrics")
    if current is None:
        current = run_single_leg_test(args, session, "baseline")
        current["label"] = "baseline"
        endpoint["trials"].append(current)
        endpoint["current_metrics"] = current
        save_session(session_path, session)

    while not current.get("passed"):
        candidate_trials = len(endpoint["trials"]) - 1
        if candidate_trials >= args.leg_max_trials:
            endpoint["status"] = "failed_max_trials"
            break

        failed_motor = failed_leg_motor_id(current, motor_ids, args.pass_error_ticks)
        if failed_motor is None:
            endpoint["status"] = "failed_without_identifiable_tick_error"
            break

        motor_key = str(failed_motor)
        original = endpoint["original_tuning"][motor_key]
        active = endpoint["active_tuning"][motor_key]
        accepted = False
        for label, candidate in leg_endpoint_candidates(original, active):
            trial_label = "trial_%02d_id_%d_%s" % (candidate_trials + 1, failed_motor, label)
            try:
                tuner._set_tuning(failed_motor, candidate)
                rospy.sleep(0.10)
                trial = run_single_leg_test(args, session, trial_label)
            except Exception:
                for motor_id, tuning in endpoint["original_tuning"].items():
                    try:
                        tuner._set_tuning(int(motor_id), tuning)
                    except (RuntimeError, rospy.ServiceException):
                        rospy.logerr("tune_dxl_position: failed to restore motor %s", motor_id)
                endpoint["active_tuning"] = dict(endpoint["original_tuning"])
                endpoint["status"] = "interrupted_original_tuning_restored"
                save_session(session_path, session)
                raise
            trial["label"] = trial_label
            trial["motor_id"] = failed_motor
            trial["tuning"] = dict(candidate)
            endpoint["trials"].append(trial)
            if leg_endpoint_improves(trial, current):
                endpoint["active_tuning"][motor_key] = dict(candidate)
                current = trial
                endpoint["current_metrics"] = current
                accepted = True
                save_session(session_path, session)
                break
            tuner._set_tuning(failed_motor, active)
            save_session(session_path, session)
            candidate_trials += 1
            if candidate_trials >= args.leg_max_trials:
                break

        if not accepted:
            endpoint["status"] = "failed_no_improving_candidate"
            break

    if current.get("passed"):
        candidates = leg_endpoint_candidates_by_board(tuner, endpoint["active_tuning"])
        override_path = os.path.join(session["session_dir"], "leg_endpoint_candidate.yaml")
        write_yaml(override_path, candidate_override(candidates))
        endpoint["candidate_override_file"] = override_path
        endpoint["status"] = "passed"
        session["candidates"] = candidates
        session["stage"] = "leg_endpoint_passed"
        save_session(session_path, session)
        print("Single-leg endpoint loop PASSED for %s" % args.leg)
        print("Candidate override: %s" % override_path)
        return 0

    for motor_id, tuning in endpoint["original_tuning"].items():
        try:
            tuner._set_tuning(int(motor_id), tuning)
        except (RuntimeError, rospy.ServiceException):
            rospy.logerr("tune_dxl_position: failed to restore motor %s", motor_id)
    endpoint["current_metrics"] = current
    session["stage"] = "leg_endpoint_failed"
    save_session(session_path, session)
    print("Single-leg endpoint loop FAILED for %s; original tuning was restored." % args.leg)
    return 1


def parse_args():
    parser = argparse.ArgumentParser(description="Operator-gated Dynamixel position tuning workflow")
    parser.add_argument("--stage", required=True, choices=["crawl-baseline", "bench", "crawl-candidate", "commit", "leg-endpoint"])
    parser.add_argument("--session", default="", help="Session JSON path; omitted for crawl-baseline or leg-endpoint")
    parser.add_argument("--output-dir", default="test_logs")
    parser.add_argument("--robot-config", default="src/climbing_description/config/robot.yaml")
    parser.add_argument("--motor-ids", default=",".join(str(value) for value in DEFAULT_MOTOR_IDS))
    parser.add_argument("--crawl-duration-s", type=float, default=40.0)
    parser.add_argument("--step-ticks", type=int, default=100)
    parser.add_argument("--extended-step-ticks", type=int, default=300)
    parser.add_argument("--pass-error-ticks", type=float, default=10.0)
    parser.add_argument("--stream-timeout-s", type=float, default=10.0)
    parser.add_argument("--leg", default="lf", choices=["lf", "rf", "rr", "lr"])
    parser.add_argument("--leg-cycles", type=int, default=1)
    parser.add_argument("--leg-swing-wait-s", type=float, default=15.0)
    parser.add_argument("--leg-hold-before-s", type=float, default=1.0)
    parser.add_argument("--leg-hold-after-s", type=float, default=1.0)
    parser.add_argument("--leg-max-trials", type=int, default=8)
    parser.add_argument("--no-confirm", action="store_true")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.stage in ("crawl-baseline", "leg-endpoint") and not args.session:
        session_path, session = new_session(args)
    elif args.session:
        session_path = os.path.abspath(args.session)
        session = load_session(session_path)
        args.robot_config = session.get("robot_config", args.robot_config)
        args.motor_ids = ",".join(str(value) for value in session.get("motor_ids", args.motor_ids.split(",")))
    else:
        raise RuntimeError("--session is required after the baseline stage")

    if args.stage == "leg-endpoint":
        return run_leg_endpoint_stage(args, session_path, session)

    if args.stage == "crawl-baseline":
        if not confirm(args, "Robot is ready for a 40 s flat adhesion crawl baseline. Start now?"):
            return 0
        csv_path, returncode = run_crawl(args, session, "crawl_baseline")
        config = load_yaml(args.robot_config)
        session["baseline_crawl"] = parse_crawl_metrics(csv_path, config)
        session["baseline_crawl"]["process_returncode"] = returncode
        session["stage"] = "crawl_baseline_done"
        save_session(session_path, session)
        print("Session: %s" % session_path)
        return 0 if returncode == 0 and not session["baseline_crawl"]["fault_seen"] else 1

    if args.stage == "bench":
        rospy.init_node("tune_dxl_position", anonymous=False)
        tuner = DxlAutoTuner(args)
        tuner._wait_services()
        if not tuner._confirm("Robot is supported; auto position/mode/current control is disabled. Run direct 100 tick tuning now?"):
            return 0
        baseline_tuning = dict(session.get("baseline_tuning", {}))
        bench_results = dict(session.get("bench", {}).get("motors", {}))
        for motor_id in tuner.motor_ids:
            existing = bench_results.get(str(motor_id))
            if existing is not None and existing.get("selected", {}).get("tuning"):
                tuner._set_tuning(motor_id, existing["selected"]["tuning"])
                rospy.loginfo("tune_dxl_position: restored completed motor %d from session", motor_id)
                continue
            tuning = tuner._get_tuning(motor_id)
            if tuning["operating_mode"] != 3:
                raise RuntimeError("motor %d must be in Position Control Mode (3), got %d" % (motor_id, tuning["operating_mode"]))
            baseline_tuning[str(motor_id)] = tuning
            bench_results[str(motor_id)] = tuner._tune_motor(motor_id, tuning, session["session_dir"])
            session["baseline_tuning"] = baseline_tuning
            session["bench"] = {"motors": bench_results}
            save_session(session_path, session)

        if tuner._confirm("All 100 tick trials passed. Run the selected candidates and baselines at 300 tick?"):
            for motor_id in tuner.motor_ids:
                result = bench_results[str(motor_id)]
                if result["selected"].get("safe"):
                    tuner._validate_extended(motor_id, result, session["session_dir"])
                else:
                    result["extended_skipped"] = "no candidate met the endpoint criterion at 100 tick"
                save_session(session_path, session)

        candidates = {"left_board": {}, "right_board": {}}
        for motor_id, result in bench_results.items():
            board_name = tuner.board_by_motor[int(motor_id)]
            candidates[board_name][str(motor_id)] = result["selected"]["tuning"]
        override_path = os.path.join(session["session_dir"], "dxl_tuning_candidate.yaml")
        write_yaml(override_path, candidate_override(candidates))
        session["candidates"] = candidates
        session["candidate_override_file"] = override_path
        session["stage"] = "bench_done"
        save_session(session_path, session)
        print("Candidate override: %s" % override_path)
        print("Restart normal bringup with: dxl_tuning_override_file:=%s" % override_path)
        print("Then run: rosrun climbing_hw_bridge tune_dxl_position.py --stage crawl-candidate --session %s" % session_path)
        return 0

    if args.stage == "crawl-candidate":
        rospy.init_node("tune_dxl_position", anonymous=False)
        tuner = DxlAutoTuner(args)
        tuner._wait_services()
        if "baseline_crawl" not in session or "candidate_override_file" not in session:
            raise RuntimeError("baseline crawl and bench candidate are required before crawl-candidate")
        for board_name, values in session["candidates"].items():
            for motor_id, expected in values.items():
                actual = tuner._get_tuning(int(motor_id))
                for key in ("p", "i", "d", "velocity", "acceleration"):
                    if actual[key] != expected[key]:
                        raise RuntimeError("motor %s does not match candidate override for %s" % (motor_id, key))
        if not tuner._confirm("Normal bringup with the candidate override is running on flat adhesion ground. Start 40 s validation crawl?"):
            return 0
        csv_path, returncode = run_crawl(args, session, "crawl_candidate")
        candidate_metrics = parse_crawl_metrics(csv_path, tuner.config)
        candidate_metrics["process_returncode"] = returncode
        comparison = compare_crawl_metrics(session["baseline_crawl"], candidate_metrics)
        if returncode != 0:
            comparison["passed"] = False
            comparison["reasons"].append("candidate crawl process returned %d" % returncode)
        session["candidate_crawl"] = candidate_metrics
        session["crawl_comparison"] = comparison
        session["stage"] = "crawl_candidate_done"
        save_session(session_path, session)
        print(json.dumps(comparison, indent=2, sort_keys=True))
        return 0 if comparison["passed"] else 1

    if args.stage == "commit":
        comparison = session.get("crawl_comparison", {})
        if not comparison.get("passed"):
            raise RuntimeError("refusing commit because candidate crawl validation did not pass")
        if not confirm(args, "Candidate validation passed. Write selected P/I/D/Profile values into robot.yaml?"):
            return 0
        backup = write_committed_tuning(args.robot_config, session["candidates"], session["session_dir"])
        session["robot_config_backup"] = backup
        session["stage"] = "committed"
        save_session(session_path, session)
        print("Committed tuning to %s (backup: %s)" % (args.robot_config, backup))
        return 0

    raise RuntimeError("unsupported stage")


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except (RuntimeError, rospy.ROSException) as exc:
        print("tune_dxl_position: %s" % exc, file=sys.stderr)
        raise SystemExit(2)
