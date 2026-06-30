#!/usr/bin/env python3

import math


def _dot(lhs, rhs):
    return sum(float(lhs[index]) * float(rhs[index]) for index in range(3))


def _sub(lhs, rhs):
    return [float(lhs[index]) - float(rhs[index]) for index in range(3)]


def _norm(vector):
    return math.sqrt(max(_dot(vector, vector), 0.0))


def _normal_component(vector, wall_normal):
    return [float(wall_normal[index]) * _dot(vector, wall_normal) for index in range(3)]


def _tangent_error(lhs, rhs, wall_normal):
    diff = _sub(lhs, rhs)
    normal = _normal_component(diff, wall_normal)
    return _norm([diff[index] - normal[index] for index in range(3)])


def ujc_body_to_command_position(
    ujc_body,
    leg_yaw_rad,
    base_radius_m,
    nominal_x_m,
    nominal_y_m,
):
    """Convert an FK UJC in body coordinates to LegCenterCommand coordinates."""
    cos_yaw = math.cos(float(leg_yaw_rad))
    sin_yaw = math.sin(float(leg_yaw_rad))
    hip_x = float(base_radius_m) * cos_yaw
    hip_y = float(base_radius_m) * sin_yaw
    dx_base = float(ujc_body[0]) - hip_x
    dy_base = float(ujc_body[1]) - hip_y
    dx_leg = cos_yaw * dx_base + sin_yaw * dy_base
    dy_leg = -sin_yaw * dx_base + cos_yaw * dy_base
    cmd_dx_leg = dx_leg - float(nominal_x_m)
    cmd_dy_leg = dy_leg - float(nominal_y_m)
    return [
        cos_yaw * cmd_dx_leg - sin_yaw * cmd_dy_leg,
        sin_yaw * cmd_dx_leg + cos_yaw * cmd_dy_leg,
        float(ujc_body[2]),
    ]


def trajectory_tracking_metrics(command_position, actual_position, wall_normal):
    """Return signed XYZ and norm errors for one synchronized trajectory sample."""
    normal_norm = _norm(wall_normal)
    if normal_norm <= 1e-9:
        normal = [0.0, 0.0, 1.0]
    else:
        normal = [float(value) / normal_norm for value in wall_normal]
    error = _sub(actual_position, command_position)
    normal_signed = _dot(error, normal)
    tangent = [error[index] - normal_signed * normal[index] for index in range(3)]
    return {
        "error_x_m": float(error[0]),
        "error_y_m": float(error[1]),
        "error_z_m": float(error[2]),
        "total_error_m": float(_norm(error)),
        "normal_error_m": float(abs(normal_signed)),
        "tangent_error_m": float(_norm(tangent)),
    }


def _percentile(values, fraction):
    ordered = sorted(float(value) for value in values)
    if not ordered:
        return 0.0
    position = max(0.0, min(float(fraction), 1.0)) * float(len(ordered) - 1)
    lower = int(math.floor(position))
    upper = int(math.ceil(position))
    if lower == upper:
        return ordered[lower]
    weight = position - float(lower)
    return ordered[lower] + weight * (ordered[upper] - ordered[lower])


def _wrapped_tick_error(actual_tick, target_tick):
    error = float(actual_tick) - float(target_tick)
    while error > 2048.0:
        error -= 4096.0
    while error < -2048.0:
        error += 4096.0
    return error


def estimate_tick_lag(samples, motor_ids, moving_phases, maximum_lag_s=0.3):
    """Estimate command-to-feedback lag by minimizing joint tick RMSE."""
    default = {
        "lag_s": 0.0,
        "shift_samples": 0,
        "zero_lag_tick_rmse": 0.0,
        "aligned_tick_rmse": 0.0,
        "pair_count": 0,
    }
    if len(samples) < 2 or not motor_ids:
        return default

    intervals = []
    for index in range(1, len(samples)):
        dt = float(samples[index].get("stamp_s", 0.0)) - float(samples[index - 1].get("stamp_s", 0.0))
        if dt > 1e-6:
            intervals.append(dt)
    if not intervals:
        return default
    sample_period_s = _percentile(intervals, 0.5)
    max_shift = min(
        len(samples) - 1,
        max(0, int(round(float(maximum_lag_s) / sample_period_s))),
    )
    moving = set(str(phase).upper() for phase in moving_phases)

    def score(shift):
        errors = []
        pair_count = 0
        for index in range(shift, len(samples)):
            actual_sample = samples[index]
            if not bool(actual_sample.get("valid", False)):
                continue
            if moving and str(actual_sample.get("phase", "")).upper() not in moving:
                continue
            actual_ticks = actual_sample.get("actual_ticks", {})
            target_ticks = samples[index - shift].get("target_ticks", {})
            if not all(int(motor_id) in actual_ticks and int(motor_id) in target_ticks for motor_id in motor_ids):
                continue
            pair_count += 1
            for motor_id in motor_ids:
                errors.append(_wrapped_tick_error(
                    actual_ticks[int(motor_id)], target_ticks[int(motor_id)],
                ))
        if not errors:
            return float("inf"), 0
        return math.sqrt(sum(error * error for error in errors) / float(len(errors))), pair_count

    zero_rmse, _ = score(0)
    best_shift = 0
    best_rmse = zero_rmse
    best_pair_count = 0
    for shift in range(max_shift + 1):
        rmse, pair_count = score(shift)
        if rmse < best_rmse:
            best_shift = shift
            best_rmse = rmse
            best_pair_count = pair_count
        elif shift == 0:
            best_pair_count = pair_count
    if math.isinf(best_rmse):
        return default
    return {
        "lag_s": float(best_shift) * sample_period_s,
        "shift_samples": best_shift,
        "zero_lag_tick_rmse": zero_rmse,
        "aligned_tick_rmse": best_rmse,
        "pair_count": best_pair_count,
    }


def lag_compensated_tracking_samples(samples, shift_samples, phase_name, wall_normal):
    """Build diagnostic tracking samples against the delayed command stream."""
    compensated = []
    shift = max(0, int(shift_samples))
    phase = str(phase_name).upper()
    for index, sample in enumerate(samples):
        if str(sample.get("phase", "")).upper() != phase:
            continue
        if index < shift or not bool(sample.get("valid", False)):
            compensated.append({"valid": False})
            continue
        command = samples[index - shift].get("command_position")
        actual = sample.get("actual_position")
        if command is None or actual is None:
            compensated.append({"valid": False})
            continue
        metrics = trajectory_tracking_metrics(command, actual, wall_normal)
        compensated.append({
            "valid": True,
            "total_error_m": metrics["total_error_m"],
            "normal_error_m": metrics["normal_error_m"],
            "tangent_error_m": metrics["tangent_error_m"],
        })
    return compensated


def phase_endpoint_overshoot(samples, phase_name, deadband_m=0.001):
    phase = str(phase_name).upper()
    segments = []
    active = []
    for sample in samples:
        if str(sample.get("phase", "")).upper() == phase:
            active.append(sample)
        elif active:
            segments.append(active)
            active = []
    if active:
        segments.append(active)

    max_overshoot_m = 0.0
    max_zero_crossings = 0
    for segment in segments:
        usable = [
            sample for sample in segment
            if sample.get("command_position") is not None and sample.get("actual_position") is not None
        ]
        if len(usable) < 2:
            continue
        start = usable[0]["command_position"]
        target = usable[-1]["command_position"]
        direction = [float(target[index]) - float(start[index]) for index in range(3)]
        span = _norm(direction)
        if span <= 1e-9:
            continue
        unit = [value / span for value in direction]
        projected = [
            _dot(_sub(sample["actual_position"], target), unit)
            for sample in usable
        ]
        max_overshoot_m = max(max_overshoot_m, max([0.0] + projected))
        signs = []
        for error in projected:
            if abs(error) <= float(deadband_m):
                continue
            signs.append(1 if error > 0.0 else -1)
        crossings = sum(1 for index in range(1, len(signs)) if signs[index] != signs[index - 1])
        max_zero_crossings = max(max_zero_crossings, crossings)
    return {
        "segment_count": len(segments),
        "max_overshoot_m": max_overshoot_m,
        "max_zero_crossings": max_zero_crossings,
    }


def summarize_tracking_samples(
    samples,
    tangent_tolerance_m=0.006,
    normal_tolerance_m=0.004,
    minimum_valid_samples=5,
    minimum_valid_ratio=0.95,
):
    total_count = len(samples)
    valid = [sample for sample in samples if bool(sample.get("valid", False))]
    valid_count = len(valid)
    valid_ratio = float(valid_count) / float(total_count) if total_count else 0.0
    result = {
        "status": "INCOMPLETE",
        "total_samples": total_count,
        "valid_samples": valid_count,
        "valid_ratio": valid_ratio,
    }
    for prefix, key in [
        ("total", "total_error_m"),
        ("normal", "normal_error_m"),
        ("tangent", "tangent_error_m"),
    ]:
        values = [float(sample[key]) for sample in valid]
        result[prefix + "_rmse_m"] = math.sqrt(sum(value * value for value in values) / float(len(values))) if values else 0.0
        result[prefix + "_p95_m"] = _percentile(values, 0.95)
        result[prefix + "_max_m"] = max(values) if values else 0.0
    if total_count <= 0 or valid_count < int(minimum_valid_samples):
        return result
    passed = (
        valid_ratio >= float(minimum_valid_ratio)
        and result["tangent_p95_m"] <= float(tangent_tolerance_m)
        and result["normal_p95_m"] <= float(normal_tolerance_m)
    )
    result["status"] = "PASS" if passed else "FAIL"
    return result


def tracking_sample_is_valid(
    command_available,
    estimated_available,
    required_motor_ids,
    motor_feedback_age_s,
    estimated_feedback_age_s,
    time_skew_s,
    maximum_feedback_age_s=0.1,
    maximum_time_skew_s=0.05,
):
    if not command_available or not estimated_available or len(required_motor_ids) != 4:
        return False
    ages = []
    for motor_id in required_motor_ids:
        if int(motor_id) not in motor_feedback_age_s:
            return False
        ages.append(float(motor_feedback_age_s[int(motor_id)]))
    ages.append(float(estimated_feedback_age_s))
    return (
        max(ages) <= float(maximum_feedback_age_s)
        and abs(float(time_skew_s)) <= float(maximum_time_skew_s)
    )


def tracking_readiness(
    phase_name,
    start_position,
    target_position,
    actual_position,
    wall_normal,
    tangent_tolerance_m,
    normal_tolerance_m,
    lift_min_ratio,
):
    """Return readiness and error metrics for actual UJC phase tracking."""
    phase = str(phase_name).upper()
    target_normal = _dot(target_position, wall_normal)
    actual_normal = _dot(actual_position, wall_normal)
    start_normal = _dot(start_position, wall_normal)
    normal_error = abs(target_normal - actual_normal)
    tangent_error = _tangent_error(target_position, actual_position, wall_normal)
    total_error = _norm(_sub(target_position, actual_position))

    tangent_ready = tangent_error <= float(tangent_tolerance_m)
    normal_ready = normal_error <= float(normal_tolerance_m)

    if phase in ["LIFT", "TRANSFER"]:
        lift_span = max(target_normal - start_normal, 0.0)
        if lift_span <= 1e-9:
            normal_ready = normal_error <= float(normal_tolerance_m)
        else:
            required_lift = float(lift_min_ratio) * lift_span
            allowed_normal_error = max(float(normal_tolerance_m), lift_span - required_lift)
            actual_lift = actual_normal - start_normal
            normal_ready = actual_lift >= required_lift and normal_error <= allowed_normal_error

    if phase == "LIFT":
        ready = normal_ready and tangent_ready
    elif phase in ["TRANSFER", "PRELOAD"]:
        ready = normal_ready and tangent_ready
    else:
        ready = True

    return {
        "ready": bool(ready),
        "total_error_m": float(total_error),
        "normal_error_m": float(normal_error),
        "tangent_error_m": float(tangent_error),
    }
