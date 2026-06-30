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


DEFAULT_MOTOR_IDS = [11, 1, 2, 15, 12, 3, 4, 16, 13, 5, 6, 17, 14, 7, 8, 18]
MOVING_PHASES = set(["LIFT", "TRANSFER", "PRELOAD", "ADMIT", "LIFT_SWING"])
SERVO_ENDPOINT_PHASES = [(5, "LIFT"), (6, "TRANSFER"), (2, "PRELOAD")]
SERVO_JOINT_COUNT = 4
DEFAULT_AUTOTUNE = {
    "bench_step_ticks": 40,
    "validation_step_ticks": 100,
    "pass_error_ticks": 3.0,
    "max_overshoot_ticks": 2.0,
    "max_foot_overshoot_mm": 1.0,
    "max_zero_crossings": 1,
    "max_current_ratio": 0.80,
    "feedback_age_limit_s": 0.10,
    "normal_p95_limit_mm": 4.0,
    "tangent_p95_limit_mm": 6.0,
    "lag_limit_ms": 80.0,
    "minimum_valid_ratio": 0.95,
    "minimum_cycles_per_leg": 2,
    "leg_max_trials": 12,
    "extended_max_candidates": 8,
    "p_multipliers": [0.75, 1.0, 1.25, 1.5],
    "i_candidates": [0, 25, 50, 100, 200],
    "d_candidates": [0, 16, 32, 64, 128],
    "velocity_candidates": [100, 150, 200, 250, 300],
    "acceleration_candidates": [100, 200, 300, 400, 600],
}


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


def autotune_config(config):
    result = dict(DEFAULT_AUTOTUNE)
    result.update(config.get("position_autotune", {}))
    return result


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
        "drive_mode": int(getattr(response, "drive_mode", 0)),
        "operating_mode": int(response.operating_mode),
        "pwm_limit": int(response.pwm_limit),
        "current_limit": int(response.current_limit),
        "p": int(response.position_p_gain),
        "i": int(response.position_i_gain),
        "d": int(response.position_d_gain),
        "velocity": int(response.profile_velocity),
        "acceleration": int(response.profile_acceleration),
    }


def metric_score(metrics, step_ticks, pass_error_ticks=3.0, lag_limit_s=0.08):
    if not metrics or not metrics.get("hard_safe"):
        return None
    step = max(1.0, abs(float(step_ticks)))
    integrated = float(metrics.get("integrated_abs_error_tick_s", step * 0.8)) / (step * 0.8)
    settling = float(metrics.get("settling_time_s", 0.8)) / 0.8
    endpoint = float(metrics.get("final_error_tick", pass_error_ticks)) / max(float(pass_error_ticks), 1.0)
    lag = float(metrics.get("lag_time_s", lag_limit_s)) / max(float(lag_limit_s), 1e-3)
    current = float(metrics.get("current_ratio", 1.0))
    return 0.35 * integrated + 0.25 * settling + 0.20 * endpoint + 0.10 * lag + 0.10 * current


def error_zero_crossings(samples, deadband=0.5):
    signs = []
    for sample in samples:
        error = float(sample["error"])
        if abs(error) <= float(deadband):
            continue
        signs.append(1 if error > 0.0 else -1)
    return sum(1 for index in range(1, len(signs)) if signs[index] != signs[index - 1])


def parse_step_csv(
    path,
    pass_error_ticks,
    feedback_age_limit_s,
    max_overshoot_ticks=2.0,
    max_zero_crossings=1,
):
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
    step_distance = max(abs(target - initial), 1.0)
    lag_time = None
    for sample in samples:
        if direction * (sample["actual"] - initial) >= 0.10 * step_distance:
            lag_time = sample["t"]
            break
    settling_time = None
    for index, sample in enumerate(samples):
        if abs(sample["error"]) <= pass_error_ticks and all(
                abs(later["error"]) <= pass_error_ticks for later in samples[index:]):
            settling_time = sample["t"]
            break
    overshoot = max([max(0.0, direction * (sample["actual"] - target)) for sample in samples])
    max_current = max(sample["current_a"] for sample in samples)
    max_age = max(sample["feedback_age_s"] for sample in samples)
    zero_crossings = error_zero_crossings(samples)
    integrated_abs_error = 0.0
    for index in range(1, len(samples)):
        dt = max(0.0, samples[index]["t"] - samples[index - 1]["t"])
        integrated_abs_error += 0.5 * dt * (
            abs(samples[index - 1]["error"]) + abs(samples[index]["error"])
        )
    error_rmse = math.sqrt(mean([sample["error"] ** 2 for sample in samples]) or 0.0)
    endpoint_pass = abs(samples[-1]["error"]) <= pass_error_ticks
    feedback_safe = max_age <= feedback_age_limit_s
    overshoot_safe = overshoot <= float(max_overshoot_ticks)
    oscillation_safe = zero_crossings <= int(max_zero_crossings)
    return {
        "safe": endpoint_pass and feedback_safe and overshoot_safe and oscillation_safe,
        "endpoint_pass": endpoint_pass,
        "feedback_safe": feedback_safe,
        "overshoot_safe": overshoot_safe,
        "oscillation_safe": oscillation_safe,
        "final_error_tick": abs(samples[-1]["error"]),
        "response_error_tick": abs(response_sample["error"]),
        "settling_time_s": settling_time,
        "lag_time_s": lag_time,
        "overshoot_tick": overshoot,
        "zero_crossings": zero_crossings,
        "integrated_abs_error_tick_s": integrated_abs_error,
        "error_rmse_tick": error_rmse,
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
            value(row, "servo_last_pass_error_joint%d_tick" % joint_index)
            for joint_index in range(1, SERVO_JOINT_COUNT + 1)
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
        value(terminal_row, "servo_error_joint%d_tick" % joint_index)
        for joint_index in range(1, SERVO_JOINT_COUNT + 1)
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
    candidate_tracking = candidate.get("tracking", {})
    baseline_tracking = baseline.get("tracking", {})
    if candidate_tracking.get("score") is not None and baseline_tracking.get("score") is not None:
        if candidate_tracking["score"] < baseline_tracking["score"] * 0.98:
            return True
    if candidate.get("timed_out") and not baseline.get("timed_out"):
        return False
    if candidate.get("endpoint_count", 0) != baseline.get("endpoint_count", 0):
        return candidate.get("endpoint_count", 0) > baseline.get("endpoint_count", 0)
    return candidate.get("terminal_max_abs_error_tick", float("inf")) < baseline.get("terminal_max_abs_error_tick", float("inf"))


def combine_step_metrics(
    direction_metrics,
    current_limit_a,
    step_ticks,
    pass_error_ticks,
    max_current_ratio=0.80,
    lag_limit_s=0.08,
):
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
        "lag_time_s": mean([
            metric["lag_time_s"] if metric.get("lag_time_s") is not None else lag_limit_s
            for metric in direction_metrics
        ]),
        "overshoot_tick": max(metric["overshoot_tick"] for metric in direction_metrics),
        "zero_crossings": max(metric.get("zero_crossings", 0) for metric in direction_metrics),
        "integrated_abs_error_tick_s": mean([
            metric.get("integrated_abs_error_tick_s", 0.0) for metric in direction_metrics
        ]),
        "error_rmse_tick": mean([
            metric.get("error_rmse_tick", 0.0) for metric in direction_metrics
        ]),
        "max_current_a": max_current,
        "max_feedback_age_s": max_age,
        "current_ratio": max_current / max(current_limit_a, 1e-6),
    }
    result["feedback_safe"] = all(metric.get("feedback_safe", metric["safe"]) for metric in direction_metrics)
    result["current_safe"] = result["current_ratio"] <= float(max_current_ratio)
    result["endpoint_pass"] = all(metric.get("endpoint_pass", metric["safe"]) for metric in direction_metrics)
    result["overshoot_safe"] = all(metric.get("overshoot_safe", False) for metric in direction_metrics)
    result["oscillation_safe"] = all(metric.get("oscillation_safe", False) for metric in direction_metrics)
    result["hard_safe"] = (
        result["feedback_safe"] and result["current_safe"] and
        result["overshoot_safe"] and result["oscillation_safe"]
    )
    result["safe"] = result["hard_safe"] and result["endpoint_pass"]
    result["score"] = metric_score(result, step_ticks, pass_error_ticks, lag_limit_s)
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
    if len(motor_ids) != SERVO_JOINT_COUNT or len(errors) != SERVO_JOINT_COUNT:
        return None
    failing = [
        (abs(float(error)), int(motor_id))
        for motor_id, error in zip(motor_ids, errors)
        if abs(float(error)) > float(pass_error_ticks)
    ]
    if failing:
        return max(failing)[1]
    p95_errors = result.get("motor_p95_abs_error_tick", {})
    available = [
        (float(p95_errors[str(int(motor_id))]), int(motor_id))
        for motor_id in motor_ids if str(int(motor_id)) in p95_errors
    ]
    return max(available)[1] if available else None


def leg_endpoint_candidates(original, active):
    return tuning_axis_candidates(original, active, DEFAULT_AUTOTUNE)


def tuning_axis_values(original, config):
    p_values = [
        int(round(float(original["p"]) * float(multiplier)))
        for multiplier in config["p_multipliers"]
    ]
    return [
        ("p", sorted(set(max(0, min(16383, value)) for value in p_values))),
        ("d", [int(value) for value in config["d_candidates"]]),
        ("i", [int(value) for value in config["i_candidates"]]),
        ("velocity", [int(value) for value in config["velocity_candidates"]]),
        ("acceleration", [int(value) for value in config["acceleration_candidates"]]),
    ]


def tuning_axis_candidates(original, active, config):
    candidates = []
    for key, values in tuning_axis_values(original, config):
        for value in values:
            if int(active[key]) == int(value):
                continue
            candidate = dict(active)
            candidate[key] = int(value)
            candidates.append(("%s_%d" % (key, value), candidate))
    return candidates


def ranked_safe_tuning_trials(result):
    ranked = sorted(
        [trial for trial in result.get("trials", [])
         if trial.get("safe") and trial.get("score") is not None],
        key=lambda trial: float(trial["score"]),
    )
    unique = []
    signatures = set()
    for trial in ranked:
        tuning = trial.get("tuning", {})
        signature = tuple(int(tuning.get(key, 0)) for key in (
            "p", "i", "d", "velocity", "acceleration",
        ))
        if signature in signatures:
            continue
        signatures.add(signature)
        unique.append(trial)
    return unique


def bench_result_passed(result):
    return bool(
        result.get("selected", {}).get("safe") and
        result.get("extended", {}).get("candidate", {}).get("safe")
    )


def motor_current_limit_a(config, motor_id, hardware_limit_a):
    limits = [float(hardware_limit_a)] if float(hardware_limit_a) > 0.0 else []
    telemetry = config.get("dynamixel_telemetry", {})
    model_name = telemetry.get("torque_model_by_motor", {}).get(str(int(motor_id)))
    model = telemetry.get("torque_models", {}).get(model_name, {})
    if model.get("max_current_a") is not None:
        limits.append(float(model["max_current_a"]))
    return min(limits) if limits else float(hardware_limit_a)


def actual_command_position(row, leg_name, config):
    actual_fields = ["%s_actual_cmd_%s" % (leg_name, axis) for axis in "xyz"]
    if all(row.get(field, "") != "" for field in actual_fields):
        return [float(row[field]) for field in actual_fields]
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
    return [actual_x, actual_y, ujc_z]


def transformed_ujc_error(row, leg_name, config):
    actual_x, actual_y, actual_z = actual_command_position(row, leg_name, config)
    error_x = actual_x - float(row["%s_cmd_x" % leg_name])
    error_y = actual_y - float(row["%s_cmd_y" % leg_name])
    error_z = actual_z - float(row["%s_cmd_z" % leg_name])
    return math.hypot(error_x, error_y), abs(error_z), math.sqrt(error_x * error_x + error_y * error_y + error_z * error_z)


def segment_overshoot_metrics(segment, leg_name, config):
    if len(segment) < 2:
        return {"overshoot_mm": 0.0, "zero_crossings": 0}
    start = [float(segment[0]["%s_cmd_%s" % (leg_name, axis)]) for axis in "xyz"]
    target = [float(segment[-1]["%s_cmd_%s" % (leg_name, axis)]) for axis in "xyz"]
    direction = [target[index] - start[index] for index in range(3)]
    span = math.sqrt(sum(value * value for value in direction))
    if span <= 1e-9:
        return {"overshoot_mm": 0.0, "zero_crossings": 0}
    unit = [value / span for value in direction]
    projected_errors = []
    for row in segment:
        actual = actual_command_position(row, leg_name, config)
        projected_errors.append(sum((actual[index] - target[index]) * unit[index] for index in range(3)))
    overshoot_mm = 1000.0 * max([0.0] + projected_errors)
    signs = []
    for error in projected_errors:
        if abs(error) <= 0.001:
            continue
        signs.append(1 if error > 0.0 else -1)
    zero_crossings = sum(1 for index in range(1, len(signs)) if signs[index] != signs[index - 1])
    return {"overshoot_mm": overshoot_mm, "zero_crossings": zero_crossings}


def parse_crawl_metrics(path, config):
    tuning = autotune_config(config)
    motor_legs = leg_by_motor(config)
    motor_errors = {motor_id: [] for motor_id in motor_legs}
    motor_currents = {motor_id: [] for motor_id in motor_legs}
    leg_errors = {leg_name: [] for leg_name in config.get("legs", {})}
    phase_errors = {
        (leg_name, phase): []
        for leg_name in config.get("legs", {}) for phase in ("LIFT", "TRANSFER", "PRELOAD", "ADMIT")
    }
    phase_total = {key: 0 for key in phase_errors}
    phase_valid = {key: 0 for key in phase_errors}
    phase_segments = {key: [] for key in phase_errors}
    active_phase = {leg_name: None for leg_name in config.get("legs", {})}
    active_segment = {leg_name: [] for leg_name in config.get("legs", {})}
    fault_seen = False
    with open(path, newline="") as stream:
        rows = list(csv.DictReader(stream))
        for row in rows:
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
                phase = row.get("%s_phase" % leg_name)
                if phase != active_phase[leg_name]:
                    previous = active_phase[leg_name]
                    if previous in ("LIFT", "TRANSFER", "PRELOAD", "ADMIT") and active_segment[leg_name]:
                        phase_segments[(leg_name, previous)].append(active_segment[leg_name])
                    active_phase[leg_name] = phase
                    active_segment[leg_name] = []
                if phase in ("LIFT", "TRANSFER", "PRELOAD", "ADMIT"):
                    active_segment[leg_name].append(row)
                    phase_total[(leg_name, phase)] += 1
                    valid = row.get("%s_tracking_valid" % leg_name, "1")
                    if str(valid) in ("1", "1.0", "True", "true"):
                        phase_valid[(leg_name, phase)] += 1
                if phase in MOVING_PHASES:
                    try:
                        errors = transformed_ujc_error(row, leg_name, config)
                        leg_errors[leg_name].append(errors)
                        valid = row.get("%s_tracking_valid" % leg_name, "1")
                        if (leg_name, phase) in phase_errors and str(valid) in ("1", "1.0", "True", "true"):
                            phase_errors[(leg_name, phase)].append(errors)
                    except (KeyError, TypeError, ValueError):
                        pass

    for leg_name, phase in active_phase.items():
        if phase in ("LIFT", "TRANSFER", "PRELOAD", "ADMIT") and active_segment[leg_name]:
            phase_segments[(leg_name, phase)].append(active_segment[leg_name])

    lag_by_leg = {}
    summary_path = os.path.splitext(path)[0] + "_tracking_summary.csv"
    if os.path.isfile(summary_path):
        with open(summary_path, newline="") as stream:
            for row in csv.DictReader(stream):
                if row.get("leg") in config.get("legs", {}) and row.get("estimated_lag_ms", "") != "":
                    lag_by_leg[row["leg"]] = max(
                        lag_by_leg.get(row["leg"], 0.0), float(row["estimated_lag_ms"]),
                    )

    phase_results = {}
    reasons = []
    required_phases = ("LIFT", "TRANSFER", "PRELOAD", "ADMIT")
    for leg_name in config.get("legs", {}):
        for phase in required_phases:
            errors = phase_errors[(leg_name, phase)]
            segments = phase_segments[(leg_name, phase)]
            overshoot = [segment_overshoot_metrics(segment, leg_name, config) for segment in segments]
            result = {
                "sample_count": len(errors),
                "valid_ratio": (
                    float(phase_valid[(leg_name, phase)]) / float(phase_total[(leg_name, phase)])
                    if phase_total[(leg_name, phase)] else 0.0
                ),
                "cycle_count": len(segments),
                "p95_tangent_error_mm": percentile([item[0] * 1000.0 for item in errors], 0.95),
                "p95_normal_error_mm": percentile([item[1] * 1000.0 for item in errors], 0.95),
                "p95_total_error_mm": percentile([item[2] * 1000.0 for item in errors], 0.95),
                "max_overshoot_mm": max([item["overshoot_mm"] for item in overshoot] or [0.0]),
                "max_zero_crossings": max([item["zero_crossings"] for item in overshoot] or [0]),
            }
            phase_results["%s:%s" % (leg_name, phase)] = result
            if len(segments) < int(tuning["minimum_cycles_per_leg"]):
                reasons.append("%s %s has %d/%d cycles" % (
                    leg_name, phase, len(segments), int(tuning["minimum_cycles_per_leg"]),
                ))
            if result["valid_ratio"] < float(tuning["minimum_valid_ratio"]):
                reasons.append("%s %s valid ratio %.3f below %.3f" % (
                    leg_name, phase, result["valid_ratio"], float(tuning["minimum_valid_ratio"]),
                ))
            if result["p95_normal_error_mm"] is None or result["p95_normal_error_mm"] > float(tuning["normal_p95_limit_mm"]):
                reasons.append("%s %s normal P95 exceeds %.1f mm" % (leg_name, phase, float(tuning["normal_p95_limit_mm"])))
            if result["p95_tangent_error_mm"] is None or result["p95_tangent_error_mm"] > float(tuning["tangent_p95_limit_mm"]):
                reasons.append("%s %s tangent P95 exceeds %.1f mm" % (leg_name, phase, float(tuning["tangent_p95_limit_mm"])))
            if result["max_overshoot_mm"] > float(tuning["max_foot_overshoot_mm"]):
                reasons.append("%s %s overshoot %.2f mm exceeds %.2f mm" % (
                    leg_name, phase, result["max_overshoot_mm"], float(tuning["max_foot_overshoot_mm"]),
                ))
            if result["max_zero_crossings"] > int(tuning["max_zero_crossings"]):
                reasons.append("%s %s oscillation crossings %d exceed %d" % (
                    leg_name, phase, result["max_zero_crossings"], int(tuning["max_zero_crossings"]),
                ))
        if lag_by_leg.get(leg_name, float("inf")) > float(tuning["lag_limit_ms"]):
            reasons.append("%s lag exceeds %.1f ms" % (leg_name, float(tuning["lag_limit_ms"])))

    if fault_seen:
        reasons.append("candidate crawl entered FAULT")

    motor_results = {}
    telemetry = config.get("dynamixel_telemetry", {})
    default_current_limit = float(telemetry.get("max_goal_current_a", 4.8))
    for motor_id, errors in motor_errors.items():
        values = motor_currents[motor_id]
        result = {
            "p95_abs_error_tick": percentile(errors, 0.95),
            "p95_current_a": percentile(values, 0.95),
            "max_current_a": max(values or [0.0]),
        }
        current_limit = motor_current_limit_a(config, motor_id, default_current_limit)
        allowed_current = float(tuning["max_current_ratio"]) * current_limit
        if result["max_current_a"] > allowed_current:
            reasons.append("motor %d current %.3fA exceeds %.3fA" % (
                motor_id, result["max_current_a"], allowed_current,
            ))
        motor_results[str(motor_id)] = result

    return {
        "csv": path,
        "tracking_summary_csv": summary_path if os.path.isfile(summary_path) else None,
        "fault_seen": fault_seen,
        "passed": not reasons,
        "reasons": reasons,
        "phases": phase_results,
        "lag_ms_by_leg": lag_by_leg,
        "motors": motor_results,
        "legs": {
            leg_name: {
                "p95_tangent_error_mm": None if not errors else percentile([item[0] * 1000.0 for item in errors], 0.95),
                "p95_normal_error_mm": None if not errors else percentile([item[1] * 1000.0 for item in errors], 0.95),
                "p95_total_error_mm": None if not errors else percentile([item[2] * 1000.0 for item in errors], 0.95),
            }
            for leg_name, errors in leg_errors.items()
        },
    }


def parse_single_leg_tracking(path, config, leg_name):
    tuning = autotune_config(config)
    phase_by_id = {5: "LIFT", 6: "TRANSFER", 2: "PRELOAD"}
    errors = {phase: [] for phase in phase_by_id.values()}
    segments = {phase: [] for phase in phase_by_id.values()}
    active_phase = None
    active_segment = []
    with open(path, newline="") as stream:
        for source in csv.DictReader(stream):
            try:
                phase = phase_by_id.get(int(round(float(source.get("servo_phase_id", -1)))))
            except (TypeError, ValueError):
                phase = None
            row = dict(source)
            for axis in "xyz":
                row["%s_cmd_%s" % (leg_name, axis)] = source.get("cmd_%s" % axis, "0")
                row["%s_ujc_%s" % (leg_name, axis)] = source.get("ujc_%s" % axis, "0")
            if phase != active_phase:
                if active_phase in segments and active_segment:
                    segments[active_phase].append(active_segment)
                active_phase = phase
                active_segment = []
            if phase in errors:
                active_segment.append(row)
                try:
                    errors[phase].append(transformed_ujc_error(row, leg_name, config))
                except (KeyError, TypeError, ValueError):
                    pass
    if active_phase in segments and active_segment:
        segments[active_phase].append(active_segment)

    phase_results = {}
    reasons = []
    score_parts = []
    for phase in ("LIFT", "TRANSFER", "PRELOAD"):
        values = errors[phase]
        overshoot = [segment_overshoot_metrics(segment, leg_name, config) for segment in segments[phase]]
        result = {
            "sample_count": len(values),
            "cycle_count": len(segments[phase]),
            "p95_tangent_error_mm": percentile([item[0] * 1000.0 for item in values], 0.95),
            "p95_normal_error_mm": percentile([item[1] * 1000.0 for item in values], 0.95),
            "max_overshoot_mm": max([item["overshoot_mm"] for item in overshoot] or [0.0]),
            "max_zero_crossings": max([item["zero_crossings"] for item in overshoot] or [0]),
        }
        phase_results[phase] = result
        if result["cycle_count"] < int(tuning["minimum_cycles_per_leg"]):
            reasons.append("%s has %d/%d cycles" % (
                phase, result["cycle_count"], int(tuning["minimum_cycles_per_leg"]),
            ))
        if result["p95_normal_error_mm"] is None or result["p95_normal_error_mm"] > float(tuning["normal_p95_limit_mm"]):
            reasons.append("%s normal P95 failed" % phase)
        if result["p95_tangent_error_mm"] is None or result["p95_tangent_error_mm"] > float(tuning["tangent_p95_limit_mm"]):
            reasons.append("%s tangent P95 failed" % phase)
        if result["max_overshoot_mm"] > float(tuning["max_foot_overshoot_mm"]):
            reasons.append("%s overshoot failed" % phase)
        if result["max_zero_crossings"] > int(tuning["max_zero_crossings"]):
            reasons.append("%s oscillation failed" % phase)
        score_parts.extend([
            (result["p95_normal_error_mm"] if result["p95_normal_error_mm"] is not None else 1000.0) / float(tuning["normal_p95_limit_mm"]),
            (result["p95_tangent_error_mm"] if result["p95_tangent_error_mm"] is not None else 1000.0) / float(tuning["tangent_p95_limit_mm"]),
            result["max_overshoot_mm"] / max(float(tuning["max_foot_overshoot_mm"]), 1e-6),
        ])
    return {
        "passed": not reasons,
        "reasons": reasons,
        "score": mean(score_parts),
        "phases": phase_results,
    }


def parse_single_leg_motor_errors(path, motor_ids):
    errors = {str(int(motor_id)): [] for motor_id in motor_ids}
    with open(path, newline="") as stream:
        for row in csv.DictReader(stream):
            for motor_id in motor_ids:
                command = row.get("ticks_cmd_%d" % int(motor_id), "")
                actual = row.get("ticks_act_%d" % int(motor_id), "")
                if command != "" and actual != "":
                    errors[str(int(motor_id))].append(abs(float(actual) - float(command)))
    return {
        motor_id: percentile(values, 0.95)
        for motor_id, values in errors.items() if values
    }


def compare_crawl_metrics(baseline, candidate):
    reasons = list(candidate.get("reasons", []))
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
        self.autotune = autotune_config(self.config)
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

    @staticmethod
    def _validate_tuning_mode(motor_id, tuning):
        if int(tuning["operating_mode"]) != 3:
            raise RuntimeError("motor %d must be in Position Control Mode (3), got %d" % (
                motor_id, int(tuning["operating_mode"]),
            ))
        if int(tuning.get("drive_mode", 0)) & 0x04:
            raise RuntimeError(
                "motor %d uses time-based profile; restart the updated multi_dxl_node "
                "to select velocity-based profile" % motor_id
            )

    def _set_tuning(self, motor_id, tuning):
        board_name = self.board_by_motor[int(motor_id)]
        response = self.set_clients[board_name](
            int(motor_id), int(tuning["p"]), int(tuning["i"]), int(tuning["d"]),
            int(tuning["velocity"]), int(tuning["acceleration"]),
        )
        if not response.success:
            raise RuntimeError("motor %d tuning write failed: %s" % (motor_id, response.message))

    def _run_step(self, motor_id, step_ticks, label, session_dir, max_current_a):
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
            "--pass-error-ticks", str(self.autotune["pass_error_ticks"]),
            "--sample-rate-hz", "100",
            "--feedback-age-limit-s", str(self.autotune["feedback_age_limit_s"]),
            "--max-current-a", str(max_current_a),
            "--max-overshoot-ticks", str(self.autotune["max_overshoot_ticks"]),
            "--no-confirm",
            "--output-dir", output_dir,
        ]
        result = subprocess.call(command)
        paths = glob.glob(os.path.join(output_dir, "dxl_position_step_*.csv"))
        if not paths:
            raise RuntimeError("position step produced no CSV for motor %d" % motor_id)
        metrics = parse_step_csv(
            max(paths, key=os.path.getmtime),
            self.autotune["pass_error_ticks"],
            self.autotune["feedback_age_limit_s"],
            self.autotune["max_overshoot_ticks"],
            self.autotune["max_zero_crossings"],
        )
        metrics["process_returncode"] = int(result)
        return metrics

    def _run_trial(self, motor_id, tuning, step_ticks, label, session_dir):
        self._set_tuning(motor_id, tuning)
        rospy.sleep(0.10)
        hardware_limit_a = float(tuning["current_limit"]) * self.current_lsb_ma / 1000.0
        current_limit_a = motor_current_limit_a(self.config, motor_id, hardware_limit_a)
        max_current_a = float(self.autotune["max_current_ratio"]) * current_limit_a
        positive = self._run_step(
            motor_id, abs(step_ticks), label + "_plus", session_dir, max_current_a,
        )
        positive_hard_safe = (
            positive.get("feedback_safe") and positive.get("overshoot_safe") and
            positive.get("oscillation_safe") and
            float(positive.get("max_current_a", float("inf"))) <= max_current_a
        )
        if not positive_hard_safe:
            combined = {"safe": False, "hard_safe": False, "directions": [positive], "score": None}
            combined["tuning"] = dict(tuning)
            combined["label"] = label
            return combined
        negative = self._run_step(
            motor_id, -abs(step_ticks), label + "_minus", session_dir, max_current_a,
        )
        negative_hard_safe = (
            negative.get("feedback_safe") and negative.get("overshoot_safe") and
            negative.get("oscillation_safe") and
            float(negative.get("max_current_a", float("inf"))) <= max_current_a
        )
        if not negative_hard_safe:
            combined = {
                "safe": False,
                "hard_safe": False,
                "directions": [positive, negative],
                "score": None,
            }
            combined["tuning"] = dict(tuning)
            combined["label"] = label
            return combined
        combined = combine_step_metrics(
            [positive, negative], current_limit_a, step_ticks,
            self.autotune["pass_error_ticks"], self.autotune["max_current_ratio"],
            float(self.autotune["lag_limit_ms"]) / 1000.0,
        )
        combined["tuning"] = dict(tuning)
        combined["label"] = label
        return combined

    @staticmethod
    def _improves(candidate, baseline):
        candidate_score = candidate.get("score")
        baseline_score = baseline.get("score")
        return (
            candidate.get("safe") and candidate_score is not None and
            (baseline_score is None or candidate_score < baseline_score)
        )

    def _tune_motor(self, motor_id, baseline_tuning, session_dir):
        try:
            step_ticks = int(self.autotune["bench_step_ticks"])
            baseline = self._run_trial(motor_id, baseline_tuning, step_ticks, "baseline", session_dir)
            if not baseline.get("hard_safe"):
                raise RuntimeError("motor %d baseline exceeded the hard current or feedback safety limit" % motor_id)
            best = baseline
            trials = [baseline]

            for pass_index in range(2):
                for key, values in tuning_axis_values(baseline_tuning, self.autotune):
                    axis_best = best
                    for value in values:
                        if int(best["tuning"][key]) == int(value):
                            continue
                        candidate = dict(best["tuning"])
                        candidate[key] = int(value)
                        label = "pass%d_%s_%d" % (pass_index + 1, key, int(value))
                        trial = self._run_trial(motor_id, candidate, step_ticks, label, session_dir)
                        trials.append(trial)
                        if self._improves(trial, axis_best):
                            axis_best = trial
                    best = axis_best
                    self._set_tuning(motor_id, best["tuning"])

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
        baseline_trial = self._run_trial(
            motor_id, baseline["tuning"], int(self.autotune["validation_step_ticks"]),
            "extended_baseline", session_dir,
        )
        selected = baseline if baseline.get("safe") and baseline_trial.get("safe") else None
        selected_trial = baseline_trial if selected is not None else None
        attempts = []
        candidates = ranked_safe_tuning_trials(result)
        limit = max(1, int(self.autotune.get("extended_max_candidates", 8)))
        baseline_signature = tuple(int(baseline["tuning"].get(key, 0)) for key in (
            "p", "i", "d", "velocity", "acceleration",
        ))
        for index, candidate in enumerate(candidates[:limit]):
            candidate_signature = tuple(int(candidate["tuning"].get(key, 0)) for key in (
                "p", "i", "d", "velocity", "acceleration",
            ))
            if candidate_signature == baseline_signature:
                candidate_trial = baseline_trial
            else:
                candidate_trial = self._run_trial(
                    motor_id, candidate["tuning"], int(self.autotune["validation_step_ticks"]),
                    "extended_candidate_%02d" % (index + 1), session_dir,
                )
            attempts.append({"bench": candidate, "validation": candidate_trial})
            if not candidate_trial.get("safe"):
                continue
            if selected_trial is None or (
                    candidate_trial.get("score") is not None and
                    (selected_trial.get("score") is None or
                     candidate_trial["score"] < selected_trial["score"])):
                selected = candidate
                selected_trial = candidate_trial

        result["extended"] = {
            "baseline": baseline_trial,
            "candidate": selected_trial,
            "attempts": attempts,
        }
        if selected is None:
            self._set_tuning(motor_id, baseline["tuning"])
            result["selected"] = baseline
            result["extended_reverted"] = True
            return False
        self._set_tuning(motor_id, selected["tuning"])
        result["selected"] = selected
        result["extended_reverted"] = selected is baseline
        return True


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
    config = load_yaml(args.robot_config)
    tuning = autotune_config(config)
    default_current_limit = float(config.get("dynamixel_telemetry", {}).get("max_goal_current_a", 4.8))
    current_limits = []
    for motor_id in motor_ids_for_leg(config, args.leg):
        limit_a = motor_current_limit_a(config, motor_id, default_current_limit)
        current_limits.append("%d:%.6f" % (
            motor_id, float(tuning["max_current_ratio"]) * limit_a,
        ))
    command = [
        "rosrun", "climbing_control_core", "test_single_leg_swing.py",
        "--leg", args.leg,
        "--cycles", str(args.leg_cycles),
        "--swing-wait", str(args.leg_swing_wait_s),
        "--hold-before", str(args.leg_hold_before_s),
        "--hold-after", str(args.leg_hold_after_s),
        "--output-dir", output_dir,
        "--motor-current-limits", ",".join(current_limits),
    ]
    returncode = subprocess.call(command)
    paths = glob.glob(os.path.join(output_dir, "single_swing_%s_*.csv" % args.leg))
    if not paths:
        raise RuntimeError("single-leg endpoint test produced no CSV")
    endpoint_tolerance = float(config.get("swing_leg_controller", {}).get(
        "servo_tracking_tolerance_ticks", args.pass_error_ticks,
    ))
    metrics = parse_single_leg_csv(max(paths, key=os.path.getmtime), endpoint_tolerance)
    metrics["endpoint_tolerance_ticks"] = endpoint_tolerance
    motor_ids = motor_ids_for_leg(config, args.leg)
    metrics["motor_p95_abs_error_tick"] = parse_single_leg_motor_errors(metrics["csv"], motor_ids)
    metrics["tracking"] = parse_single_leg_tracking(metrics["csv"], load_yaml(args.robot_config), args.leg)
    metrics["process_returncode"] = int(returncode)
    metrics["passed"] = (
        bool(metrics.get("passed")) and bool(metrics["tracking"].get("passed")) and
        returncode == 0
    )
    if returncode == 5:
        raise RuntimeError("single-leg test triggered a current or feedback safety abort")
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
    if len(motor_ids) != SERVO_JOINT_COUNT:
        raise RuntimeError("leg %s must define exactly four motor_ids" % args.leg)

    endpoints = session.setdefault("leg_endpoints", {})
    if session.get("leg_endpoint") is not None and session["leg_endpoint"].get("leg") == args.leg:
        endpoints.setdefault(args.leg, session.pop("leg_endpoint"))
    endpoint = endpoints.get(args.leg)
    if endpoint is None:
        if not tuner._confirm(
                "Robot is supported and normal bringup has servo_tracking_gate_enabled:=true. "
                "Run the single-leg test/tune/test loop for %s now?" % args.leg):
            return 0
        original_tuning = {}
        for motor_id in motor_ids:
            tuning = tuner._get_tuning(motor_id)
            tuner._validate_tuning_mode(motor_id, tuning)
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
        endpoints[args.leg] = endpoint
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
        if candidate_trials >= int(tuner.autotune["leg_max_trials"]):
            endpoint["status"] = "failed_max_trials"
            break

        failed_motor = failed_leg_motor_id(
            current, motor_ids, current.get("endpoint_tolerance_ticks", args.pass_error_ticks),
        )
        if failed_motor is None:
            endpoint["status"] = "failed_without_identifiable_tick_error"
            break

        motor_key = str(failed_motor)
        original = endpoint["original_tuning"][motor_key]
        active = endpoint["active_tuning"][motor_key]
        accepted = False
        if not tuner._confirm("Run loaded candidate search for motor %d on leg %s?" % (failed_motor, args.leg)):
            endpoint["status"] = "paused_by_operator"
            save_session(session_path, session)
            return 0
        for label, candidate in tuning_axis_candidates(original, active, tuner.autotune):
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
            if candidate_trials >= int(tuner.autotune["leg_max_trials"]):
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
    parser.add_argument("--stage", required=True, choices=["crawl-baseline", "bench", "crawl-candidate", "commit", "leg-endpoint", "status"])
    parser.add_argument("--session", default="", help="Session JSON path; omitted for crawl-baseline or leg-endpoint")
    parser.add_argument("--output-dir", default="test_logs")
    parser.add_argument("--robot-config", default="src/climbing_description/config/robot.yaml")
    parser.add_argument("--motor-ids", default=",".join(str(value) for value in DEFAULT_MOTOR_IDS))
    parser.add_argument("--crawl-duration-s", type=float, default=120.0)
    parser.add_argument("--step-ticks", type=int, default=40)
    parser.add_argument("--extended-step-ticks", type=int, default=100)
    parser.add_argument("--pass-error-ticks", type=float, default=3.0)
    parser.add_argument("--stream-timeout-s", type=float, default=10.0)
    parser.add_argument("--leg", default="lf", choices=["lf", "rf", "rr", "lr", "all"])
    parser.add_argument("--leg-cycles", type=int, default=2)
    parser.add_argument("--leg-swing-wait-s", type=float, default=15.0)
    parser.add_argument("--leg-hold-before-s", type=float, default=1.0)
    parser.add_argument("--leg-hold-after-s", type=float, default=1.0)
    parser.add_argument("--leg-max-trials", type=int, default=12)
    parser.add_argument("--no-confirm", action="store_true")
    parser.add_argument("--dry-run", action="store_true", help="Print the bounded search plan without ROS or motor writes")
    return parser.parse_args()


def main():
    args = parse_args()
    if args.dry_run:
        config = load_yaml(args.robot_config)
        tuning = autotune_config(config)
        motor_ids = [int(value) for value in args.motor_ids.split(",") if value.strip()]
        print("Dynamixel autotune dry run")
        print("  stage: %s" % args.stage)
        print("  motors: %s" % ",".join(str(value) for value in motor_ids))
        print("  bench/validation step: %d/%d tick" % (
            int(tuning["bench_step_ticks"]), int(tuning["validation_step_ticks"]),
        ))
        print("  hard limits: overshoot<=%.1f tick foot<=%.1f mm current<=%.0f%%" % (
            float(tuning["max_overshoot_ticks"]), float(tuning["max_foot_overshoot_mm"]),
            100.0 * float(tuning["max_current_ratio"]),
        ))
        print("  per-motor candidates per pass: %d" % sum(
            len(values) for _, values in tuning_axis_values(
                {"p": 1000, "i": 0, "d": 0, "velocity": 200, "acceleration": 300}, tuning,
            )
        ))
        return 0
    if args.stage in ("crawl-baseline", "leg-endpoint", "bench") and not args.session:
        session_path, session = new_session(args)
        print("Session: %s" % session_path)
    elif args.session:
        session_path = os.path.abspath(args.session)
        session = load_session(session_path)
        args.robot_config = session.get("robot_config", args.robot_config)
        args.motor_ids = ",".join(str(value) for value in session.get("motor_ids", args.motor_ids.split(",")))
    else:
        raise RuntimeError("--session is required after the baseline stage")

    if args.stage == "status":
        print(json.dumps(session, indent=2, sort_keys=True))
        return 0

    if args.stage == "leg-endpoint" and args.leg != "all":
        return run_leg_endpoint_stage(args, session_path, session)

    if args.stage == "leg-endpoint" and args.leg == "all":
        for leg_name in ("lf", "rf", "rr", "lr"):
            command = [
                "rosrun", "climbing_hw_bridge", "tune_dxl_position.py",
                "--stage", "leg-endpoint", "--session", session_path,
                "--leg", leg_name, "--leg-cycles", str(args.leg_cycles),
                "--leg-swing-wait-s", str(args.leg_swing_wait_s),
                "--leg-hold-before-s", str(args.leg_hold_before_s),
                "--leg-hold-after-s", str(args.leg_hold_after_s),
            ]
            if args.no_confirm:
                command.append("--no-confirm")
            if subprocess.call(command) != 0:
                return 1
        return 0

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
        if not tuner._confirm("Robot is supported; auto position/mode/current control is disabled. Start guarded 16-motor tuning?"):
            return 0
        baseline_tuning = dict(session.get("baseline_tuning", {}))
        bench_results = dict(session.get("bench", {}).get("motors", {}))
        for motor_id in tuner.motor_ids:
            existing = bench_results.get(str(motor_id))
            if existing is not None and bench_result_passed(existing):
                tuner._set_tuning(motor_id, existing["selected"]["tuning"])
                rospy.loginfo("tune_dxl_position: restored completed motor %d from session", motor_id)
                continue
            tuning = tuner._get_tuning(motor_id)
            tuner._validate_tuning_mode(motor_id, tuning)
            if not tuner._confirm("Motor %d is mechanically clear for +/- %d tick tests. Continue?" % (
                    motor_id, int(tuner.autotune["validation_step_ticks"]))):
                session["stage"] = "bench_paused"
                save_session(session_path, session)
                return 0
            baseline_tuning[str(motor_id)] = tuning
            bench_results[str(motor_id)] = tuner._tune_motor(motor_id, tuning, session["session_dir"])
            if not bench_results[str(motor_id)]["selected"].get("safe"):
                session["stage"] = "bench_failed"
                session["baseline_tuning"] = baseline_tuning
                session["bench"] = {"motors": bench_results}
                save_session(session_path, session)
                raise RuntimeError("motor %d has no candidate satisfying strict endpoint and overshoot limits" % motor_id)
            extended_passed = tuner._validate_extended(
                motor_id, bench_results[str(motor_id)], session["session_dir"],
            )
            session["baseline_tuning"] = baseline_tuning
            session["bench"] = {"motors": bench_results}
            save_session(session_path, session)
            if not extended_passed:
                session["stage"] = "bench_failed_extended"
                save_session(session_path, session)
                raise RuntimeError(
                    "motor %d has no candidate satisfying the extended validation limits" % motor_id
                )

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
        if "candidate_override_file" not in session:
            raise RuntimeError("bench candidate is required before crawl-candidate")
        endpoint_status = session.get("leg_endpoints", {})
        missing_legs = [
            leg_name for leg_name in ("lf", "rf", "rr", "lr")
            if endpoint_status.get(leg_name, {}).get("status") != "passed"
        ]
        if missing_legs:
            raise RuntimeError("loaded single-leg validation is incomplete: %s" % ",".join(missing_legs))
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
        comparison = compare_crawl_metrics(session.get("baseline_crawl", {}), candidate_metrics)
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
