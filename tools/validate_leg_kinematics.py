#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import yaml


def clamp(value, lower, upper):
    return max(lower, min(upper, value))


def matmul4(lhs, rhs):
    result = [[0.0] * 4 for _ in range(4)]
    for row in range(4):
        for column in range(4):
            result[row][column] = sum(lhs[row][k] * rhs[k][column] for k in range(4))
    return result


def mdh_transform(alpha_prev_deg, a_prev_mm, theta_deg, d_mm):
    alpha = math.radians(alpha_prev_deg)
    theta = math.radians(theta_deg)
    ca = math.cos(alpha)
    sa = math.sin(alpha)
    ct = math.cos(theta)
    st = math.sin(theta)
    return [
        [ct, -st, 0.0, a_prev_mm],
        [st * ca, ct * ca, -sa, -d_mm * sa],
        [st * sa, ct * sa, ca, d_mm * ca],
        [0.0, 0.0, 0.0, 1.0],
    ]


def format_vector(vector):
    return "[{:.3f}, {:.3f}, {:.3f}]".format(vector[0], vector[1], vector[2])


class LegKinematicsConfig(object):
    def __init__(self, config_dict):
        gait = config_dict["gait_controller"]
        self.link_coxa = float(gait["link_coxa"])
        self.link_femur = float(gait["link_femur"])
        self.link_tibia = float(gait["link_tibia"])
        self.link_a3 = float(gait["link_a3"])
        self.link_d6 = float(gait["link_d6"])
        self.link_d7 = float(gait["link_d7"])
        self.base_radius = float(gait["base_radius"])
        self.nominal_x = float(gait["nominal_x"])
        self.nominal_y = float(gait["nominal_y"])
        self.nominal_universal_joint_center_z = float(gait["nominal_universal_joint_center_z"])
        self.joint_limits = gait["joint_limit_deg"]
        self.legs = config_dict["legs"]


def load_config(config_path):
    with open(config_path, "r") as stream:
        return LegKinematicsConfig(yaml.safe_load(stream))


def legacy_code_fk_local_mm(cfg, joint_deg):
    q1 = math.radians(float(joint_deg[0]))
    q2 = math.radians(float(joint_deg[1]))
    q3 = math.radians(float(joint_deg[2]))
    alpha = q2 - math.radians(90.0)
    radial_prime = cfg.link_tibia * math.cos(alpha) + cfg.link_a3 * math.cos(alpha + q3)
    p_z = cfg.link_tibia * math.sin(alpha) + cfg.link_a3 * math.sin(alpha + q3)
    radial_total = cfg.link_femur + radial_prime
    return [radial_total * math.cos(q1), radial_total * math.sin(q1), p_z]


def corrected_dh_fk_local_mm(cfg, joint_deg):
    q1_deg = float(joint_deg[0])
    q2_deg = float(joint_deg[1])
    q3_deg = float(joint_deg[2])
    chain = [
        (0.0, cfg.link_coxa, q1_deg, 0.0),
        (90.0, cfg.link_femur, q2_deg - 90.0, 0.0),
        (0.0, cfg.link_tibia, q3_deg, 0.0),
        (0.0, cfg.link_a3, 0.0, 0.0),
    ]
    transform = [
        [1.0, 0.0, 0.0, 0.0],
        [0.0, 1.0, 0.0, 0.0],
        [0.0, 0.0, 1.0, 0.0],
        [0.0, 0.0, 0.0, 1.0],
    ]
    for alpha_prev_deg, a_prev_mm, theta_deg, d_mm in chain:
        transform = matmul4(transform, mdh_transform(alpha_prev_deg, a_prev_mm, theta_deg, d_mm))
    return [transform[0][3], transform[1][3], transform[2][3]]


def implemented_fk_local_mm(cfg, joint_deg):
    return corrected_dh_fk_local_mm(cfg, joint_deg)


def vector_sub(lhs, rhs):
    return [lhs[index] - rhs[index] for index in range(3)]


def vector_norm(vector):
    return math.sqrt(sum(value * value for value in vector))


def legacy_code_ik_deg(cfg, target_local_mm, q3_branch=-1.0):
    x_mm = float(target_local_mm[0])
    y_mm = float(target_local_mm[1])
    z_mm = float(target_local_mm[2])
    q1 = math.atan2(y_mm, x_mm)
    r_total = math.hypot(x_mm, y_mm)
    r_prime = max(r_total - cfg.link_femur, 1.0e-9)
    d_sq = r_prime ** 2 + z_mm ** 2
    cos_theta3 = (d_sq - cfg.link_tibia ** 2 - cfg.link_a3 ** 2) / (2.0 * cfg.link_tibia * cfg.link_a3)
    cos_theta3 = clamp(cos_theta3, -1.0, 1.0)
    sin_theta3 = q3_branch * math.sqrt(max(0.0, 1.0 - cos_theta3 ** 2))
    theta3 = math.atan2(sin_theta3, cos_theta3)
    q2 = math.atan2(z_mm, r_prime) - math.atan2(cfg.link_a3 * math.sin(theta3), cfg.link_tibia + cfg.link_a3 * math.cos(theta3)) + math.radians(90.0)
    return [math.degrees(q1), math.degrees(q2), math.degrees(theta3)]


def implemented_ik_deg(cfg, target_local_mm, reference_deg=None):
    candidates = corrected_dh_ik_candidates_deg(cfg, target_local_mm)
    if reference_deg is None:
        reference_deg = [0.0, 0.0, 0.0]
    return min(candidates, key=lambda candidate: sum((candidate[index] - reference_deg[index]) ** 2 for index in range(3)))


def corrected_dh_ik_candidates_deg(cfg, target_local_mm):
    negative = corrected_dh_ik_deg(cfg, target_local_mm, q3_branch=-1.0)
    positive = corrected_dh_ik_deg(cfg, target_local_mm, q3_branch=1.0)
    return [negative, positive]


def corrected_dh_ik_deg(cfg, target_local_mm, q3_branch=-1.0):
    target = [float(target_local_mm[0]), float(target_local_mm[1]), float(target_local_mm[2])]
    q1_seed = math.degrees(math.atan2(target[1], max(target[0] - cfg.link_coxa, 1.0e-6)))
    state = [q1_seed, 0.0, 20.0 if q3_branch > 0.0 else -20.0]
    damping = 1.0e-3

    for _ in range(80):
        current = corrected_dh_fk_local_mm(cfg, state)
        error = vector_sub(target, current)
        if vector_norm(error) < 1.0e-6:
            break

        jacobian = [[0.0, 0.0, 0.0] for _ in range(3)]
        step_deg = 1.0e-4
        for column in range(3):
            perturbed = list(state)
            perturbed[column] += step_deg
            next_position = corrected_dh_fk_local_mm(cfg, perturbed)
            for row in range(3):
                jacobian[row][column] = (next_position[row] - current[row]) / step_deg

        jt = [[jacobian[row][column] for row in range(3)] for column in range(3)]
        normal = [[0.0, 0.0, 0.0] for _ in range(3)]
        rhs = [0.0, 0.0, 0.0]
        for row in range(3):
            for column in range(3):
                normal[row][column] = sum(jt[row][k] * jacobian[k][column] for k in range(3))
            normal[row][row] += damping
            rhs[row] = sum(jt[row][k] * error[k] for k in range(3))

        delta = solve_3x3(normal, rhs)
        state = [state[index] + delta[index] for index in range(3)]

    return state


def solve_3x3(matrix, rhs):
    augmented = [list(matrix[row]) + [rhs[row]] for row in range(3)]
    for pivot in range(3):
        pivot_row = max(range(pivot, 3), key=lambda row: abs(augmented[row][pivot]))
        if abs(augmented[pivot_row][pivot]) < 1.0e-12:
            return [0.0, 0.0, 0.0]
        if pivot_row != pivot:
            augmented[pivot], augmented[pivot_row] = augmented[pivot_row], augmented[pivot]
        pivot_value = augmented[pivot][pivot]
        for column in range(pivot, 4):
            augmented[pivot][column] /= pivot_value
        for row in range(3):
            if row == pivot:
                continue
            scale = augmented[row][pivot]
            for column in range(pivot, 4):
                augmented[row][column] -= scale * augmented[pivot][column]
    return [augmented[row][3] for row in range(3)]


def rotate_z_mm(vector_mm, yaw_deg):
    yaw = math.radians(float(yaw_deg))
    cos_yaw = math.cos(yaw)
    sin_yaw = math.sin(yaw)
    x_value = vector_mm[0]
    y_value = vector_mm[1]
    return [
        cos_yaw * x_value - sin_yaw * y_value,
        sin_yaw * x_value + cos_yaw * y_value,
        vector_mm[2],
    ]


def body_position_from_leg0(cfg, leg_name, local_mm):
    hip_yaw_deg = float(cfg.legs[leg_name]["hip_yaw_deg"])
    offset = rotate_z_mm([cfg.base_radius, 0.0, 0.0], hip_yaw_deg)
    local_body = rotate_z_mm(local_mm, hip_yaw_deg)
    return [offset[0] + local_body[0], offset[1] + local_body[1], local_body[2]]


def nominal_body_position_from_code_path(cfg, leg_name):
    hip_yaw_deg = float(cfg.legs[leg_name]["hip_yaw_deg"])
    offset = rotate_z_mm([cfg.base_radius, 0.0, 0.0], hip_yaw_deg)
    nominal_local = [cfg.nominal_x, cfg.nominal_y, cfg.nominal_universal_joint_center_z]
    nominal_body = rotate_z_mm(nominal_local, hip_yaw_deg)
    return [offset[0] + nominal_body[0], offset[1] + nominal_body[1], nominal_body[2]]


def print_joint_case_report(cfg, joint_samples):
    print("Joint-space FK comparison (leg-local, frame-0 origin, mm)")
    print("-" * 72)
    for joint_deg in joint_samples:
        legacy_fk = legacy_code_fk_local_mm(cfg, joint_deg)
        implemented_fk = implemented_fk_local_mm(cfg, joint_deg)
        corrected_fk = corrected_dh_fk_local_mm(cfg, joint_deg)
        legacy_delta = [corrected_fk[index] - legacy_fk[index] for index in range(3)]
        implemented_delta = [corrected_fk[index] - implemented_fk[index] for index in range(3)]
        print("q = {}".format(tuple(float(value) for value in joint_deg)))
        print("  legacy_code_fk  : {}".format(format_vector(legacy_fk)))
        print("  implemented_fk  : {}".format(format_vector(implemented_fk)))
        print("  corrected_dh_fk : {}".format(format_vector(corrected_fk)))
        print("  dh_minus_legacy : {}".format(format_vector(legacy_delta)))
        print("  dh_minus_impl   : {}".format(format_vector(implemented_delta)))
    print("")


def print_nominal_report(cfg):
    zero_pose = [0.0, 0.0, 0.0]
    legacy_zero = legacy_code_fk_local_mm(cfg, zero_pose)
    implemented_zero = implemented_fk_local_mm(cfg, zero_pose)
    corrected_zero = corrected_dh_fk_local_mm(cfg, zero_pose)
    print("Zero-pose interpretation")
    print("-" * 72)
    print("  configured nominal_x                     : {:.3f} mm".format(cfg.nominal_x))
    print("  legacy_code zero-pose local endpoint     : {}".format(format_vector(legacy_zero)))
    print("  implemented zero-pose local endpoint     : {}".format(format_vector(implemented_zero)))
    print("  corrected_dh zero-pose local endpoint    : {}".format(format_vector(corrected_zero)))
    print("  legacy zero-pose x discrepancy           : {:.3f} mm".format(corrected_zero[0] - legacy_zero[0]))
    print("  implemented zero-pose x discrepancy      : {:.3f} mm".format(corrected_zero[0] - implemented_zero[0]))
    print("")

    print("Nominal body-frame endpoint positions from config path")
    print("-" * 72)
    for leg_name in ["lf", "rf", "rr", "lr"]:
        legacy_body = body_position_from_leg0(cfg, leg_name, legacy_zero)
        implemented_body = body_position_from_leg0(cfg, leg_name, implemented_zero)
        corrected_body = body_position_from_leg0(cfg, leg_name, corrected_zero)
        configured_body = nominal_body_position_from_code_path(cfg, leg_name)
        print("{}".format(leg_name))
        print("  configured_nominal_body : {}".format(format_vector(configured_body)))
        print("  legacy_zero_body        : {}".format(format_vector(legacy_body)))
        print("  implemented_zero_body   : {}".format(format_vector(implemented_body)))
        print("  corrected_zero_body     : {}".format(format_vector(corrected_body)))
    print("")


def print_roundtrip_report(cfg, joint_samples):
    print("IK branch consistency")
    print("-" * 72)
    for joint_deg in joint_samples:
        corrected_target = corrected_dh_fk_local_mm(cfg, joint_deg)
        legacy_recovered = legacy_code_ik_deg(cfg, legacy_code_fk_local_mm(cfg, joint_deg), q3_branch=-1.0)
        implemented_recovered = implemented_ik_deg(cfg, corrected_target, joint_deg)
        corrected_negative = corrected_dh_ik_deg(cfg, corrected_target, q3_branch=-1.0)
        corrected_positive = corrected_dh_ik_deg(cfg, corrected_target, q3_branch=1.0)
        print("q_src = {}".format(tuple(float(value) for value in joint_deg)))
        print("  legacy round-trip       : {}".format(format_vector(legacy_recovered)))
        print("  implemented round-trip  : {}".format(format_vector(implemented_recovered)))
        print("  corrected IK q3<0       : {}".format(format_vector(corrected_negative)))
        print("  corrected IK q3>0       : {}".format(format_vector(corrected_positive)))
    print("")


def parse_args():
    default_config = Path(__file__).resolve().parents[1] / "src" / "climbing_description" / "config" / "robot.yaml"
    parser = argparse.ArgumentParser(description="Compare current leg kinematics implementation against a corrected DH-based model.")
    parser.add_argument("--config", default=str(default_config), help="Path to robot.yaml")
    parser.add_argument(
        "--joint",
        action="append",
        nargs=3,
        metavar=("Q1_DEG", "Q2_DEG", "Q3_DEG"),
        help="Joint sample in degrees; may be passed multiple times.",
    )
    return parser.parse_args()


def main():
    args = parse_args()
    cfg = load_config(args.config)
    if args.joint:
        joint_samples = [[float(value) for value in sample] for sample in args.joint]
    else:
        joint_samples = [
            [0.0, 0.0, 0.0],
            [0.0, 10.0, 0.0],
            [0.0, 0.0, 10.0],
            [0.0, 30.0, -20.0],
            [0.0, 30.0, 20.0],
            [20.0, 30.0, -20.0],
        ]

    print("Loaded config: {}".format(args.config))
    print("")
    print_nominal_report(cfg)
    print_joint_case_report(cfg, joint_samples)
    print_roundtrip_report(cfg, joint_samples)


if __name__ == "__main__":
    main()