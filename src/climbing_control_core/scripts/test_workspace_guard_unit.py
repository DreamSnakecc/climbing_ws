#!/usr/bin/env python3

import math

from workspace_guard import (
    _fk_from_joint_deg,
    _ik_candidates_deg,
    _within_limits,
    joint_transfer_path,
    joint_transfer_path_free_end_z,
    workspace_guard,
)


def _build_model():
    return {
        "nominal_x_m": 0.12775,
        "nominal_y_m": 0.0,
        "nominal_z_m": -0.3758,
        "operating_x_m": 0.26713,
        "operating_y_m": 0.0,
        "operating_z_m": -0.2873,
        "l_coxa": 0.04475,
        "l_femur": 0.0830,
        "l_tibia": 0.1540,
        "l_a3": 0.0700,
        "l_a4": 0.1518,
        "q234_sum_limit_deg": [-5.0, 5.0],
        "leg_yaw_rad": {"lf": math.radians(45.0)},
    }


def _joint_limits():
    return {"j1": [-90.0, 90.0], "j2": [-10.0, 150.0], "j3": [-100.0, 100.0], "j4": [-100.0, 100.0]}


def _reachable_center_body_from_joint(model, leg_name, joint_deg):
    q1 = math.radians(float(joint_deg[0]))
    q2r = math.radians(float(joint_deg[1]) - 90.0)
    q3 = math.radians(float(joint_deg[2]))
    q4 = math.radians(float(joint_deg[3]))
    r_prime = model["l_tibia"] * math.cos(q2r) + model["l_a3"] * math.cos(q2r + q3) + model["l_a4"] * math.cos(q2r + q3 + q4)
    pz = model["l_tibia"] * math.sin(q2r) + model["l_a3"] * math.sin(q2r + q3) + model["l_a4"] * math.sin(q2r + q3 + q4)
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
    anchor = _reachable_center_body_from_joint(model, "lf", [0.0, 65.0, -65.0, 0.0])

    nominal_fk = _fk_from_joint_deg([0.0, 0.0, 0.0, 0.0], model)
    assert max(abs(nominal_fk[index] - [0.12775, 0.0, -0.3758][index]) for index in range(3)) < 1e-9
    operating = [0.26713, 0.0, -0.2873]
    operating_candidates = _ik_candidates_deg(operating[0], operating[1], operating[2], model)
    operating_joint = min(operating_candidates, key=lambda candidate: abs(candidate[2] + 64.824))
    operating_fk = _fk_from_joint_deg(operating_joint, model)
    assert max(abs(operating_fk[index] - operating[index]) for index in range(3)) < 0.002
    assert abs(sum(operating_joint[1:4])) < 1e-9
    assert not _within_limits([0.0, 40.0, -30.0, 0.0], limits, [-5.0, 5.0])

    # reachable target: should not clamp
    reachable = _reachable_center_body_from_joint(model, "lf", [0.0, 75.0, -75.0, 0.0])
    checked, clamped, margin, joint, _ = workspace_guard(
        leg_name="lf",
        candidate_center_body_m=reachable,
        reference_center_body_m=anchor,
        last_joint_deg=[0.0, 0.0, 0.0, 0.0],
        model=model,
        joint_limits_deg=limits,
        clamp_max_iter=10,
        fk_tol_m=0.003,
    )
    assert not clamped, "reachable point should not clamp"
    assert len(joint) == 4
    assert abs(sum(joint[1:4])) <= 5.0
    assert margin > -0.05
    assert max(abs(checked[i] - reachable[i]) for i in [0, 1, 2]) < 1e-6

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
    assert len(joint) == 4
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
    assert len(joint2) == 4
    assert not any(math.isnan(value) for value in joint2)
    assert clamped2 in [True, False]
    assert len(checked2) == 3

    # Joint-space transfer keeps the endpoints but allows intermediate z to vary.
    transfer_start = _reachable_center_body_from_joint(model, "lf", [25.0, 75.0, -78.0, 3.0])
    transfer_end = [transfer_start[0] + 0.03, transfer_start[1], transfer_start[2]]
    path = joint_transfer_path(
        leg_name="lf",
        start_position_m=transfer_start,
        end_position_m=transfer_end,
        model=model,
        joint_limits_deg=limits,
        reference_joint_deg=[25.0, 75.0, -78.0, 3.0],
        sample_count=31,
        fk_tol_m=0.003,
    )
    assert path is not None, "joint-space transfer should have a reachable path"
    assert max(abs(path[0]["position"][axis] - transfer_start[axis]) for axis in range(3)) < 1e-9
    assert max(abs(path[-1]["position"][axis] - transfer_end[axis]) for axis in range(3)) < 1e-9
    assert any(abs(point["position"][2] - transfer_start[2]) > 1e-6 for point in path[1:-1])
    assert all(point["joint_deg"][2] < 0.0 for point in path)
    assert any(abs(point["joint_deg"][1] + point["joint_deg"][2]) > 5.0 for point in path)
    assert all(abs(point["q234_sum_deg"]) <= 5.0 + 1e-6 for point in path)
    assert all(point["fk_error_m"] <= 0.003 for point in path)

    # No fallback is permitted when a requested endpoint is unreachable.
    impossible = joint_transfer_path(
        leg_name="lf",
        start_position_m=transfer_start,
        end_position_m=[transfer_start[0] + 1.0, transfer_start[1], transfer_start[2]],
        model=model,
        joint_limits_deg=limits,
        reference_joint_deg=[25.0, 75.0, -78.0, 3.0],
        sample_count=31,
        fk_tol_m=0.003,
    )
    assert impossible is None, "unreachable transfer endpoint must fail closed"

    # The real LF lift point cannot move 45 mm at fixed Z, but transfer Z is free.
    lf_lift = [0.1436, 0.0986, -0.2573]
    fixed_end = [lf_lift[0] + 0.045, lf_lift[1], lf_lift[2]]
    assert joint_transfer_path(
        "lf", lf_lift, fixed_end, model, limits, [0.0, 65.0, -65.0, 0.0], 41, 0.002,
    ) is None
    free_z_path = joint_transfer_path_free_end_z(
        "lf", lf_lift, fixed_end, model, limits, [0.0, 65.0, -65.0, 0.0],
        sample_count=41, fk_tol_m=0.002, end_z_search_m=0.08,
        end_z_step_m=0.005, min_abs_knee_deg=10.0,
    )
    assert free_z_path is not None
    assert abs(free_z_path[-1]["position"][0] - fixed_end[0]) < 1e-9
    assert free_z_path[-1]["position"][2] > fixed_end[2]
    assert abs(free_z_path[-1]["joint_deg"][2]) >= 10.0

    print("workspace_guard unit tests passed")


if __name__ == "__main__":
    run_tests()
