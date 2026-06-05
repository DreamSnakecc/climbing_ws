#!/usr/bin/env python3

from actual_tracking import tracking_readiness


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

    print("actual_tracking unit tests passed")


if __name__ == "__main__":
    run_tests()
