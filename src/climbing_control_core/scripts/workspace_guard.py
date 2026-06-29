#!/usr/bin/env python

import math
import time


def _clamp(value, lo, hi):
    return max(lo, min(hi, value))


def _ik_candidates_deg(x_m, y_m, z_m, geom):
    l_coxa = float(geom["l_coxa"])
    l_femur = float(geom["l_femur"])
    l_tibia = float(geom["l_tibia"])
    l_a3 = float(geom["l_a3"])
    l_a4 = float(geom["l_a4"])

    x_prime = float(x_m) - l_coxa
    q1 = math.atan2(float(y_m), x_prime)
    r_total = math.hypot(x_prime, float(y_m))
    r_prime = max(r_total - l_femur, 1e-9)
    joint4_z = float(z_m) + l_a4
    d_sq = r_prime ** 2 + joint4_z ** 2
    cos_theta3 = (d_sq - l_tibia ** 2 - l_a3 ** 2) / (2.0 * l_tibia * l_a3)
    cos_theta3 = _clamp(cos_theta3, -1.0, 1.0)
    sin_theta3_mag = math.sqrt(max(0.0, 1.0 - cos_theta3 ** 2))

    candidates = []
    for sign in [-1.0, 1.0]:
        theta3 = math.atan2(sign * sin_theta3_mag, cos_theta3)
        q2 = (
            math.atan2(joint4_z, r_prime)
            - math.atan2(l_a3 * math.sin(theta3), l_tibia + l_a3 * math.cos(theta3))
            + math.radians(90.0)
        )
        q2_deg = math.degrees(q2)
        q3_deg = math.degrees(theta3)
        candidates.append([math.degrees(q1), q2_deg, q3_deg, -(q2_deg + q3_deg)])
    return candidates


def _fk_from_joint_deg(joint_deg, geom):
    l_coxa = float(geom["l_coxa"])
    l_femur = float(geom["l_femur"])
    l_tibia = float(geom["l_tibia"])
    l_a3 = float(geom["l_a3"])
    l_a4 = float(geom["l_a4"])

    q1 = math.radians(float(joint_deg[0]))
    q2r = math.radians(float(joint_deg[1]) - 90.0)
    q3 = math.radians(float(joint_deg[2]))
    q4 = math.radians(float(joint_deg[3]))

    r_prime = (
        l_tibia * math.cos(q2r)
        + l_a3 * math.cos(q2r + q3)
        + l_a4 * math.cos(q2r + q3 + q4)
    )
    pz = (
        l_tibia * math.sin(q2r)
        + l_a3 * math.sin(q2r + q3)
        + l_a4 * math.sin(q2r + q3 + q4)
    )
    r_total = l_femur + r_prime
    x_prime = r_total * math.cos(q1)
    y = r_total * math.sin(q1)
    x = l_coxa + x_prime
    return [x, y, pz]


def _joint_cost(candidate_deg, reference_deg):
    return sum((float(candidate_deg[i]) - float(reference_deg[i])) ** 2 for i in range(4))


def _within_limits(candidate_deg, joint_limits_deg, q234_sum_limit_deg=None):
    for idx, key in enumerate(["j1", "j2", "j3", "j4"]):
        lo = float(joint_limits_deg[key][0])
        hi = float(joint_limits_deg[key][1])
        if float(candidate_deg[idx]) < lo or float(candidate_deg[idx]) > hi:
            return False
    if q234_sum_limit_deg is not None:
        q234_sum = float(candidate_deg[1]) + float(candidate_deg[2]) + float(candidate_deg[3])
        lo = float(q234_sum_limit_deg[0])
        hi = float(q234_sum_limit_deg[1])
        if q234_sum < lo or q234_sum > hi:
            return False
    return True


def _coarse_margin_m(x_m, y_m, z_m, geom):
    l_coxa = float(geom["l_coxa"])
    l_femur = float(geom["l_femur"])
    l_tibia = float(geom["l_tibia"])
    l_a3 = float(geom["l_a3"])
    l_a4 = float(geom["l_a4"])
    x_prime = float(x_m) - l_coxa
    r_total = math.hypot(x_prime, float(y_m))
    r_prime = max(r_total - l_femur, 0.0)
    d = math.hypot(r_prime, float(z_m) + l_a4)
    lower = abs(l_tibia - l_a3)
    upper = l_tibia + l_a3
    return min(upper - d, d - lower)


def _solve_reachable(point_leg_m, geom, joint_limits_deg, reference_joint_deg, fk_tol_m, q234_sum_limit_deg):
    candidates = _ik_candidates_deg(point_leg_m[0], point_leg_m[1], point_leg_m[2], geom)
    valid = []
    for candidate in candidates:
        if not _within_limits(candidate, joint_limits_deg, q234_sum_limit_deg):
            continue
        fk = _fk_from_joint_deg(candidate, geom)
        err = math.sqrt(
            (fk[0] - point_leg_m[0]) ** 2
            + (fk[1] - point_leg_m[1]) ** 2
            + (fk[2] - point_leg_m[2]) ** 2
        )
        if err <= fk_tol_m:
            valid.append((candidate, err))
    if not valid:
        return None
    valid.sort(key=lambda item: (_joint_cost(item[0], reference_joint_deg), item[1]))
    return list(valid[0][0])


def _to_leg_frame_m(leg_name, center_body_m, model):
    yaw = float(model["leg_yaw_rad"][leg_name])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)

    dx_base = float(center_body_m[0])
    dy_base = float(center_body_m[1])
    dz_base = float(center_body_m[2]) - float(model["nominal_z_m"])

    dx_leg = cos_yaw * dx_base + sin_yaw * dy_base
    dy_leg = -sin_yaw * dx_base + cos_yaw * dy_base

    return [
        float(model["nominal_x_m"]) + dx_leg,
        float(model["nominal_y_m"]) + dy_leg,
        float(model["nominal_z_m"]) + dz_base,
    ]


def _from_leg_frame_m(leg_name, point_leg_m, model):
    yaw = float(model["leg_yaw_rad"][leg_name])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx_leg = float(point_leg_m[0]) - float(model["nominal_x_m"])
    dy_leg = float(point_leg_m[1]) - float(model["nominal_y_m"])
    return [
        cos_yaw * dx_leg - sin_yaw * dy_leg,
        sin_yaw * dx_leg + cos_yaw * dy_leg,
        float(point_leg_m[2]),
    ]


def joint_transfer_path(
    leg_name,
    start_position_m,
    end_position_m,
    model,
    joint_limits_deg,
    reference_joint_deg,
    sample_count=41,
    fk_tol_m=0.002,
):
    """Generate a reachable transfer by interpolating joints and applying FK."""
    count = max(int(sample_count), 2)
    reference = list(reference_joint_deg) if len(reference_joint_deg) == 4 else [0.0, 90.0, -60.0, -30.0]
    q234_limit = model.get("q234_sum_limit_deg", [-5.0, 5.0])
    start_leg = _to_leg_frame_m(leg_name, start_position_m, model)
    end_leg = _to_leg_frame_m(leg_name, end_position_m, model)
    start_joint = _solve_reachable(
        start_leg, model, joint_limits_deg, reference, fk_tol_m, q234_limit,
    )
    if start_joint is None:
        return None
    end_joint = _solve_reachable(
        end_leg, model, joint_limits_deg, start_joint, fk_tol_m, q234_limit,
    )
    if end_joint is None:
        return None

    path = []
    for index in range(count):
        fraction = float(index) / float(count - 1)
        joint_deg = [
            float(start_joint[joint_index])
            + fraction * (float(end_joint[joint_index]) - float(start_joint[joint_index]))
            for joint_index in range(4)
        ]
        if not _within_limits(joint_deg, joint_limits_deg, q234_limit):
            return None
        point_leg = _fk_from_joint_deg(joint_deg, model)
        position = _from_leg_frame_m(leg_name, point_leg, model)
        roundtrip_leg = _to_leg_frame_m(leg_name, position, model)
        fk_error = math.sqrt(sum((roundtrip_leg[axis] - point_leg[axis]) ** 2 for axis in range(3)))
        path.append({
            "position": position,
            "joint_deg": joint_deg,
            "q234_sum_deg": sum(joint_deg[1:4]),
            "fk_error_m": fk_error,
        })
    return path


def workspace_guard(
    leg_name,
    candidate_center_body_m,
    reference_center_body_m,
    last_joint_deg,
    model,
    joint_limits_deg,
    clamp_max_iter=12,
    fk_tol_m=0.002,
    q234_sum_limit_deg=None,
):
    """
    Returns:
      checked_center_body_m, is_clamped, distance_to_boundary_m, joint_solution_deg, elapsed_us
    """
    start = time.time()
    if q234_sum_limit_deg is None:
        q234_sum_limit_deg = model.get("q234_sum_limit_deg", [-5.0, 5.0])
    ref_joint = list(last_joint_deg) if isinstance(last_joint_deg, (list, tuple)) and len(last_joint_deg) == 4 else [0.0, 0.0, 0.0, 0.0]
    candidate_leg = _to_leg_frame_m(leg_name, candidate_center_body_m, model)
    margin = _coarse_margin_m(candidate_leg[0], candidate_leg[1], candidate_leg[2], model)
    direct = _solve_reachable(candidate_leg, model, joint_limits_deg, ref_joint, fk_tol_m, q234_sum_limit_deg)
    if direct is not None:
        return list(candidate_center_body_m), False, float(margin), direct, (time.time() - start) * 1e6

    anchor = list(reference_center_body_m)
    anchor_leg = _to_leg_frame_m(leg_name, anchor, model)
    anchor_joint = _solve_reachable(anchor_leg, model, joint_limits_deg, ref_joint, fk_tol_m, q234_sum_limit_deg)
    if anchor_joint is None:
        fallback = list(model.get("operating_center_body_m", reference_center_body_m))
        anchor = fallback
        anchor_leg = _to_leg_frame_m(leg_name, anchor, model)
        anchor_joint = _solve_reachable(anchor_leg, model, joint_limits_deg, ref_joint, fk_tol_m, q234_sum_limit_deg)
        if anchor_joint is None:
            return list(anchor), True, float(margin), ref_joint, (time.time() - start) * 1e6

    lo = 0.0
    hi = 1.0
    best = list(anchor)
    best_joint = list(anchor_joint)
    for _ in range(max(int(clamp_max_iter), 1)):
        alpha = 0.5 * (lo + hi)
        point = [
            anchor[0] + alpha * (candidate_center_body_m[0] - anchor[0]),
            anchor[1] + alpha * (candidate_center_body_m[1] - anchor[1]),
            anchor[2] + alpha * (candidate_center_body_m[2] - anchor[2]),
        ]
        point_leg = _to_leg_frame_m(leg_name, point, model)
        joint = _solve_reachable(point_leg, model, joint_limits_deg, best_joint, fk_tol_m, q234_sum_limit_deg)
        if joint is not None:
            lo = alpha
            best = point
            best_joint = joint
        else:
            hi = alpha

    best_leg = _to_leg_frame_m(leg_name, best, model)
    best_margin = _coarse_margin_m(best_leg[0], best_leg[1], best_leg[2], model)
    return list(best), True, float(best_margin), best_joint, (time.time() - start) * 1e6
