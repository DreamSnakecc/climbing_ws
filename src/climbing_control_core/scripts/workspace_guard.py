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


def _from_leg_frame_xy_m(leg_name, local_x_m, local_y_m, fixed_z_m, model):
    yaw = float(model["leg_yaw_rad"][leg_name])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    dx_leg = float(local_x_m) - float(model["nominal_x_m"])
    dy_leg = float(local_y_m) - float(model["nominal_y_m"])
    return [
        cos_yaw * dx_leg - sin_yaw * dy_leg,
        sin_yaw * dx_leg + cos_yaw * dy_leg,
        float(fixed_z_m),
    ]


def _transfer_q23_values(q23_sum_limit_deg, sample_count):
    lo = float(q23_sum_limit_deg[0])
    hi = float(q23_sum_limit_deg[1])
    count = max(int(sample_count), 2)
    if hi < lo:
        lo, hi = hi, lo
    return [lo + (hi - lo) * float(index) / float(count - 1) for index in range(count)]


def _constrained_transfer_candidates(
    leg_name,
    fixed_x_m,
    desired_y_m,
    fixed_z_m,
    model,
    joint_limits_deg,
    q23_sum_limit_deg,
    fk_tol_m,
    q23_sample_count,
):
    """Return negative-q3 solutions at a fixed body-frame x/z coordinate."""
    l_coxa = float(model["l_coxa"])
    l_femur = float(model["l_femur"])
    l_tibia = float(model["l_tibia"])
    l_a3 = float(model["l_a3"])
    yaw = float(model["leg_yaw_rad"][leg_name])
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    nominal_x = float(model["nominal_x_m"])
    nominal_y = float(model["nominal_y_m"])
    fixed_z = float(fixed_z_m)
    fixed_x = float(fixed_x_m)
    candidates = []

    # The fixed-z constraint leaves q2 solvable from each q2+q3 sample.
    # q3 is constrained to the knee-backward branch used by leg_ik_executor.
    for q23_sum_deg in _transfer_q23_values(q23_sum_limit_deg, q23_sample_count):
        q23_sum_rad = math.radians(q23_sum_deg)
        cos_q2 = (l_a3 * math.sin(q23_sum_rad - math.pi / 2.0) - fixed_z) / l_tibia
        if cos_q2 < -1.0 - 1e-9 or cos_q2 > 1.0 + 1e-9:
            continue
        cos_q2 = _clamp(cos_q2, -1.0, 1.0)
        q2_magnitude_deg = math.degrees(math.acos(cos_q2))
        q2_options = [q2_magnitude_deg, -q2_magnitude_deg]
        for q2_deg in q2_options:
            q3_deg = q23_sum_deg - q2_deg
            if q3_deg >= 0.0:
                continue
            q2_rad = math.radians(q2_deg)
            radial_total = (
                l_femur
                + l_tibia * math.sin(q2_rad)
                + l_a3 * math.cos(q23_sum_rad - math.pi / 2.0)
            )
            # In body coordinates the local radial circle becomes a quadratic
            # in y once x and z are fixed.
            local_x_offset = nominal_x + cos_yaw * fixed_x - l_coxa
            local_y_offset = nominal_y - sin_yaw * fixed_x
            linear = 2.0 * (sin_yaw * local_x_offset + cos_yaw * local_y_offset)
            constant = (
                local_x_offset * local_x_offset
                + local_y_offset * local_y_offset
                - radial_total * radial_total
            )
            discriminant = linear * linear - 4.0 * constant
            if discriminant < -1e-12:
                continue
            root = math.sqrt(max(0.0, discriminant))
            for y_value in [(-linear + root) / 2.0, (-linear - root) / 2.0]:
                local_x = nominal_x + cos_yaw * fixed_x + sin_yaw * y_value
                local_y = nominal_y - sin_yaw * fixed_x + cos_yaw * y_value
                q1_deg = math.degrees(math.atan2(local_y, local_x - l_coxa))
                joint_deg = [q1_deg, q2_deg, q3_deg]
                if not _within_limits(joint_deg, joint_limits_deg, q23_sum_limit_deg):
                    continue
                fk = _fk_from_joint_deg(joint_deg, {
                    "l_coxa": l_coxa,
                    "l_femur": l_femur,
                    "l_tibia": l_tibia,
                    "l_a3": l_a3,
                })
                fk_error = math.sqrt(
                    (fk[0] - local_x) ** 2
                    + (fk[1] - local_y) ** 2
                    + (fk[2] - fixed_z) ** 2
                )
                if fk_error > float(fk_tol_m):
                    continue
                candidates.append({
                    "position": _from_leg_frame_xy_m(
                        leg_name, local_x, local_y, fixed_z, model,
                    ),
                    "joint_deg": joint_deg,
                    "q23_sum_deg": q2_deg + q3_deg,
                    "lateral_offset_m": y_value - float(desired_y_m),
                    "fk_error_m": fk_error,
                })
    return candidates


def constrained_transfer_path(
    leg_name,
    start_x_m,
    end_x_m,
    start_y_m,
    end_y_m,
    fixed_z_m,
    model,
    joint_limits_deg,
    reference_joint_deg,
    q23_sum_limit_deg,
    sample_count=41,
    q23_sample_count=101,
    fk_tol_m=0.002,
):
    """Plan a fixed-z, fixed-x-progress transfer path on the q2+q3 band.

    At each x sample the nearest lateral solution is chosen. Joint-space
    distance to the previous point is the tie-breaker, preserving continuity.
    Returns a list of point dictionaries, or None when any sample has no
    valid negative-q3 solution.
    """
    count = max(int(sample_count), 2)
    reference = list(reference_joint_deg) if len(reference_joint_deg) == 3 else [0.0, 90.0, -60.0]
    path = []
    for index in range(count):
        fraction = float(index) / float(count - 1)
        fixed_x = float(start_x_m) + fraction * (float(end_x_m) - float(start_x_m))
        desired_y = float(start_y_m) + fraction * (float(end_y_m) - float(start_y_m))
        candidates = _constrained_transfer_candidates(
            leg_name,
            fixed_x,
            desired_y,
            fixed_z_m,
            model,
            joint_limits_deg,
            q23_sum_limit_deg,
            fk_tol_m,
            q23_sample_count,
        )
        if not candidates:
            return None
        candidates.sort(key=lambda item: (
            abs(float(item["lateral_offset_m"])),
            _joint_cost(item["joint_deg"], reference),
            float(item["fk_error_m"]),
        ))
        selected = candidates[0]
        path.append(selected)
        reference = list(selected["joint_deg"])
    return path


def pre_lift_q23_align_point(
    leg_name,
    current_position_m,
    model,
    joint_limits_deg,
    reference_joint_deg,
    q23_target_deg=0.0,
    q23_tolerance_deg=5.0,
    q23_sample_count=101,
    fk_tol_m=0.002,
):
    """Return the nearest fixed-x/z point with negative q3 and q2+q3 near target."""
    if len(current_position_m) < 3:
        return None
    target = float(q23_target_deg)
    tolerance = abs(float(q23_tolerance_deg))
    limit = [target - tolerance, target + tolerance]
    candidates = _constrained_transfer_candidates(
        leg_name,
        float(current_position_m[0]),
        float(current_position_m[1]),
        float(current_position_m[2]),
        model,
        joint_limits_deg,
        limit,
        fk_tol_m,
        q23_sample_count,
    )
    if not candidates:
        return None
    reference = list(reference_joint_deg) if len(reference_joint_deg) == 3 else [0.0, 90.0, -60.0]
    candidates.sort(key=lambda item: (
        abs(float(item["q23_sum_deg"]) - target),
        abs(float(item["lateral_offset_m"])),
        _joint_cost(item["joint_deg"], reference),
        float(item["fk_error_m"]),
    ))
    return candidates[0]


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
        fallback = list(model.get("operating_center_body_m", reference_center_body_m))
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
