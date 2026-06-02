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

    x_prime = float(x_m) - l_coxa
    q1 = math.atan2(float(y_m), x_prime)
    r_total = math.hypot(x_prime, float(y_m))
    r_prime = max(r_total - l_femur, 1e-9)
    d_sq = r_prime ** 2 + float(z_m) ** 2
    cos_theta3 = (d_sq - l_tibia ** 2 - l_a3 ** 2) / (2.0 * l_tibia * l_a3)
    cos_theta3 = _clamp(cos_theta3, -1.0, 1.0)
    sin_theta3_mag = math.sqrt(max(0.0, 1.0 - cos_theta3 ** 2))

    candidates = []
    for sign in [-1.0, 1.0]:
        theta3 = math.atan2(sign * sin_theta3_mag, cos_theta3)
        q2 = (
            math.atan2(float(z_m), r_prime)
            - math.atan2(l_a3 * math.sin(theta3), l_tibia + l_a3 * math.cos(theta3))
            + math.radians(90.0)
        )
        candidates.append([math.degrees(q1), math.degrees(q2), math.degrees(theta3)])
    return candidates


def _fk_from_joint_deg(joint_deg, geom):
    l_coxa = float(geom["l_coxa"])
    l_femur = float(geom["l_femur"])
    l_tibia = float(geom["l_tibia"])
    l_a3 = float(geom["l_a3"])

    q1 = math.radians(float(joint_deg[0]))
    q2r = math.radians(float(joint_deg[1]) - 90.0)
    q3 = math.radians(float(joint_deg[2]))

    r_prime = l_tibia * math.cos(q2r) + l_a3 * math.cos(q2r + q3)
    pz = l_tibia * math.sin(q2r) + l_a3 * math.sin(q2r + q3)
    r_total = l_femur + r_prime
    x_prime = r_total * math.cos(q1)
    y = r_total * math.sin(q1)
    x = l_coxa + x_prime
    return [x, y, pz]


def _joint_cost(candidate_deg, reference_deg):
    return sum((float(candidate_deg[i]) - float(reference_deg[i])) ** 2 for i in [0, 1, 2])


def _within_limits(candidate_deg, joint_limits_deg, q23_sum_limit_deg=None):
    for idx, key in enumerate(["j1", "j2", "j3"]):
        lo = float(joint_limits_deg[key][0])
        hi = float(joint_limits_deg[key][1])
        if float(candidate_deg[idx]) < lo or float(candidate_deg[idx]) > hi:
            return False
    if q23_sum_limit_deg is not None:
        q23_sum = float(candidate_deg[1]) + float(candidate_deg[2])
        lo = float(q23_sum_limit_deg[0])
        hi = float(q23_sum_limit_deg[1])
        if q23_sum < lo or q23_sum > hi:
            return False
    return True


def _coarse_margin_m(x_m, y_m, z_m, geom):
    l_coxa = float(geom["l_coxa"])
    l_femur = float(geom["l_femur"])
    l_tibia = float(geom["l_tibia"])
    l_a3 = float(geom["l_a3"])
    x_prime = float(x_m) - l_coxa
    r_total = math.hypot(x_prime, float(y_m))
    r_prime = max(r_total - l_femur, 0.0)
    d = math.hypot(r_prime, float(z_m))
    lower = abs(l_tibia - l_a3)
    upper = l_tibia + l_a3
    return min(upper - d, d - lower)


def _solve_reachable(point_leg_m, geom, joint_limits_deg, reference_joint_deg, fk_tol_m, q23_sum_limit_deg):
    candidates = _ik_candidates_deg(point_leg_m[0], point_leg_m[1], point_leg_m[2], geom)
    valid = []
    for candidate in candidates:
        if not _within_limits(candidate, joint_limits_deg, q23_sum_limit_deg):
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


def workspace_guard(
    leg_name,
    candidate_center_body_m,
    reference_center_body_m,
    last_joint_deg,
    model,
    joint_limits_deg,
    clamp_max_iter=12,
    fk_tol_m=0.002,
    q23_sum_limit_deg=None,
):
    """
    Returns:
      checked_center_body_m, is_clamped, distance_to_boundary_m, joint_solution_deg, elapsed_us
    """
    start = time.time()
    ref_joint = list(last_joint_deg) if isinstance(last_joint_deg, (list, tuple)) and len(last_joint_deg) == 3 else [0.0, 0.0, 0.0]
    candidate_leg = _to_leg_frame_m(leg_name, candidate_center_body_m, model)
    margin = _coarse_margin_m(candidate_leg[0], candidate_leg[1], candidate_leg[2], model)
    direct = _solve_reachable(candidate_leg, model, joint_limits_deg, ref_joint, fk_tol_m, q23_sum_limit_deg)
    if direct is not None:
        return list(candidate_center_body_m), False, float(margin), direct, (time.time() - start) * 1e6

    anchor = list(reference_center_body_m)
    anchor_leg = _to_leg_frame_m(leg_name, anchor, model)
    anchor_joint = _solve_reachable(anchor_leg, model, joint_limits_deg, ref_joint, fk_tol_m, q23_sum_limit_deg)
    if anchor_joint is None:
        fallback = [
            float(model["operating_x_m"]),
            float(model["operating_y_m"]),
            float(model["operating_z_m"]),
        ]
        anchor = fallback
        anchor_leg = _to_leg_frame_m(leg_name, anchor, model)
        anchor_joint = _solve_reachable(anchor_leg, model, joint_limits_deg, ref_joint, fk_tol_m, q23_sum_limit_deg)
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
        joint = _solve_reachable(point_leg, model, joint_limits_deg, best_joint, fk_tol_m, q23_sum_limit_deg)
        if joint is not None:
            lo = alpha
            best = point
            best_joint = joint
        else:
            hi = alpha

    best_leg = _to_leg_frame_m(leg_name, best, model)
    best_margin = _coarse_margin_m(best_leg[0], best_leg[1], best_leg[2], model)
    return list(best), True, float(best_margin), best_joint, (time.time() - start) * 1e6
