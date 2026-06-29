#!/usr/bin/env python3

import math

from actual_tracking import (
    estimate_tick_lag,
    lag_compensated_tracking_samples,
    summarize_tracking_samples,
    tracking_sample_is_valid,
    tracking_readiness,
    trajectory_tracking_metrics,
    ujc_body_to_command_position,
)


WALL_NORMAL = [0.0, 0.0, 1.0]


def _ready(phase, start, target, actual):
    return tracking_readiness(
        phase_name=phase,
        start_position=start,
        target_position=target,
        actual_position=actual,
        wall_normal=WALL_NORMAL,
        tangent_tolerance_m=0.006,
        normal_tolerance_m=0.004,
        lift_min_ratio=0.75,
    )


def run_tests():
    start = [0.10, 0.00, -0.223]
    lift = [0.10, 0.00, -0.203]
    transfer = [0.13, 0.00, -0.203]
    preload = [0.13, 0.00, -0.224]

    assert _ready("LIFT", start, lift, [0.10, 0.00, -0.204])["ready"]
    assert not _ready("LIFT", start, lift, [0.10, 0.00, -0.222])["ready"]

    assert _ready("TRANSFER", start, transfer, [0.128, 0.001, -0.204])["ready"]
    assert not _ready("TRANSFER", start, transfer, [0.115, 0.000, -0.204])["ready"]
    assert not _ready("TRANSFER", start, transfer, [0.130, 0.000, -0.218])["ready"]

    assert _ready("PRELOAD", transfer, preload, [0.129, 0.001, -0.223])["ready"]
    assert not _ready("PRELOAD", transfer, preload, [0.129, 0.001, -0.210])["ready"]

    # ADMIT keeps the existing fan/adhesion decision path and ignores position error.
    assert _ready("ADMIT", preload, preload, [0.20, 0.05, -0.200])["ready"]

    command = [0.021, -0.013, -0.240]
    nominal_x = 0.12775
    nominal_y = 0.0
    base_radius = 0.20306
    for yaw_deg in [45.0, -45.0, -135.0, 135.0]:
        yaw = math.radians(yaw_deg)
        cos_yaw = math.cos(yaw)
        sin_yaw = math.sin(yaw)
        dx_leg = cos_yaw * command[0] + sin_yaw * command[1]
        dy_leg = -sin_yaw * command[0] + cos_yaw * command[1]
        local_x = nominal_x + dx_leg
        local_y = nominal_y + dy_leg
        ujc_body = [
            base_radius * cos_yaw + cos_yaw * local_x - sin_yaw * local_y,
            base_radius * sin_yaw + sin_yaw * local_x + cos_yaw * local_y,
            command[2],
        ]
        recovered = ujc_body_to_command_position(
            ujc_body, yaw, base_radius, nominal_x, nominal_y,
        )
        assert max(abs(recovered[index] - command[index]) for index in range(3)) < 1e-9

    metrics = trajectory_tracking_metrics(
        [0.0, 0.0, 0.0], [0.003, 0.004, 0.012], WALL_NORMAL,
    )
    assert abs(metrics["total_error_m"] - 0.013) < 1e-9
    assert abs(metrics["tangent_error_m"] - 0.005) < 1e-9
    assert abs(metrics["normal_error_m"] - 0.012) < 1e-9
    assert metrics["error_z_m"] == 0.012

    passing = [
        {"valid": True, "total_error_m": 0.005, "normal_error_m": 0.003, "tangent_error_m": 0.004}
        for _ in range(20)
    ]
    assert summarize_tracking_samples(passing)["status"] == "PASS"
    failing = [dict(sample, tangent_error_m=0.007) for sample in passing]
    assert summarize_tracking_samples(failing)["status"] == "FAIL"
    assert summarize_tracking_samples(passing[:4])["status"] == "INCOMPLETE"
    stale = list(passing[:18]) + [dict(passing[0], valid=False) for _ in range(2)]
    assert summarize_tracking_samples(stale)["status"] == "FAIL"

    motor_ages = {15: 0.01, 1: 0.02, 2: 0.03, 11: 0.01}
    assert tracking_sample_is_valid(True, True, [11, 1, 2, 15], motor_ages, 0.02, 0.01)
    assert not tracking_sample_is_valid(True, True, [11, 1, 2, 18], motor_ages, 0.02, 0.01)
    old_motor_ages = dict(motor_ages)
    old_motor_ages[15] = 0.2
    assert not tracking_sample_is_valid(True, True, [11, 1, 2, 15], old_motor_ages, 0.02, 0.01)
    assert not tracking_sample_is_valid(True, True, [11, 1, 2, 15], motor_ages, 0.02, 0.08)

    lag_samples = []
    motor_ids = [11, 1, 2, 15]
    for index in range(30):
        delayed_index = max(0, index - 6)
        lag_samples.append({
            "stamp_s": 0.02 * index,
            "phase": "LIFT",
            "valid": True,
            "command_position": [0.001 * index, 0.0, -0.25],
            "actual_position": [0.001 * delayed_index, 0.0, -0.25],
            "target_ticks": {
                motor_id: 1000.0 + 4.0 * index + motor_offset
                for motor_offset, motor_id in enumerate(motor_ids)
            },
            "actual_ticks": {
                motor_id: 1000.0 + 4.0 * delayed_index + motor_offset
                for motor_offset, motor_id in enumerate(motor_ids)
            },
        })
    lag = estimate_tick_lag(lag_samples, motor_ids, ["LIFT"], maximum_lag_s=0.3)
    assert lag["shift_samples"] == 6
    assert abs(lag["lag_s"] - 0.12) < 1e-9
    assert lag["aligned_tick_rmse"] < lag["zero_lag_tick_rmse"]
    compensated = lag_compensated_tracking_samples(
        lag_samples, lag["shift_samples"], "LIFT", WALL_NORMAL,
    )
    compensated_summary = summarize_tracking_samples(compensated)
    assert compensated_summary["valid_samples"] == 24
    assert compensated_summary["total_max_m"] < 1e-9

    print("actual_tracking unit tests passed")


if __name__ == "__main__":
    run_tests()
