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
