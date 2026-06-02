#!/usr/bin/env python3

import math

from workspace_guard import workspace_guard


def _build_model():
    return {
        "nominal_x_m": 0.11875,
        "nominal_y_m": 0.0,
        "nominal_z_m": -0.1955,
        "operating_x_m": 0.23704,
        "operating_y_m": 0.0,
        "operating_z_m": -0.1500,
        "l_coxa": 0.0625,
        "l_femur": 0.0830,
        "l_tibia": 0.1540,
        "l_a3": 0.0415,
        "leg_yaw_rad": {"lf": math.radians(45.0)},
    }


def _joint_limits():
    return {"j1": [-90.0, 90.0], "j2": [-10.0, 190.0], "j3": [-100.0, 100.0]}


def _reachable_center_body_from_joint(model, leg_name, joint_deg):
    q1 = math.radians(float(joint_deg[0]))
    q2r = math.radians(float(joint_deg[1]) - 90.0)
    q3 = math.radians(float(joint_deg[2]))
    r_prime = model["l_tibia"] * math.cos(q2r) + model["l_a3"] * math.cos(q2r + q3)
    pz = model["l_tibia"] * math.sin(q2r) + model["l_a3"] * math.sin(q2r + q3)
    r_total = model["l_femur"] + r_prime
    x_prime = r_total * math.cos(q1)
    y_leg = r_total * math.sin(q1)
    tx_leg = model["l_coxa"] + x_prime
    ty_leg = y_leg
    tz_leg = pz

    dx_leg = tx_leg - model["nominal_x_m"]
    dy_leg = ty_leg - model["nominal_y_m"]
    yaw = model["leg_yaw_rad"][leg_name]
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx_base = cos_yaw * dx_leg - sin_yaw * dy_leg
    dy_base = sin_yaw * dx_leg + cos_yaw * dy_leg
    return [dx_base, dy_base, tz_leg]


def run_tests():
    model = _build_model()
    limits = _joint_limits()
    anchor = [0.0, 0.0, -0.1500]

    # reachable target: should not clamp
    reachable = _reachable_center_body_from_joint(model, "lf", [0.0, 95.0, -35.0])
    checked, clamped, margin, joint, _ = workspace_guard(
        leg_name="lf",
        candidate_center_body_m=reachable,
        reference_center_body_m=anchor,
        last_joint_deg=[0.0, 0.0, 0.0],
        model=model,
        joint_limits_deg=limits,
        clamp_max_iter=10,
        fk_tol_m=0.003,
    )
    assert not clamped, "reachable point should not clamp"
    assert len(joint) == 3
    assert margin > -0.05
    assert max(abs(checked[i] - reachable[i]) for i in [0, 1, 2]) < 1e-6

    # q2 + q3 constraint: individual joints are valid, but the sum must be limited.
    q23_sum_violation = _reachable_center_body_from_joint(model, "lf", [0.0, 40.0, 0.0])
    _, clamped, _, joint, _ = workspace_guard(
        leg_name="lf",
        candidate_center_body_m=q23_sum_violation,
        reference_center_body_m=anchor,
        last_joint_deg=[0.0, 0.0, 0.0],
        model=model,
        joint_limits_deg=limits,
        clamp_max_iter=10,
        fk_tol_m=0.003,
        q23_sum_limit_deg=[-10.0, 10.0],
    )
    assert clamped, "q2+q3 violation should clamp"
    assert -10.0 <= joint[1] + joint[2] <= 10.0

    # unreachable far target: should clamp
    unreachable = [0.60, 0.50, -0.45]
    checked, clamped, margin, joint, _ = workspace_guard(
        leg_name="lf",
        candidate_center_body_m=unreachable,
        reference_center_body_m=anchor,
        last_joint_deg=joint,
        model=model,
        joint_limits_deg=limits,
        clamp_max_iter=12,
        fk_tol_m=0.003,
    )
    assert clamped, "unreachable point should clamp"
    assert len(joint) == 3
    assert all(abs(checked[i]) <= abs(unreachable[i]) + 1e-6 for i in [0, 1, 2])

    # branch continuity sanity: near-boundary perturbation should stay valid
    near = [checked[0] + 0.002, checked[1] - 0.002, checked[2]]
    checked2, clamped2, _, joint2, _ = workspace_guard(
        leg_name="lf",
        candidate_center_body_m=near,
        reference_center_body_m=anchor,
        last_joint_deg=joint,
        model=model,
        joint_limits_deg=limits,
        clamp_max_iter=12,
        fk_tol_m=0.003,
    )
    assert len(joint2) == 3
    assert not any(math.isnan(value) for value in joint2)
    assert clamped2 in [True, False]
    assert len(checked2) == 3

    print("workspace_guard unit tests passed")


if __name__ == "__main__":
    run_tests()
