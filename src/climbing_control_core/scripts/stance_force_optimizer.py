#!/usr/bin/env python

import math

import rospy
from climbing_msgs.msg import BodyReference, EstimatedState, StanceWrenchCommand
from geometry_msgs.msg import Wrench

try:
    import numpy as np
except ImportError:
    np = None


def quaternion_to_xyzw(quaternion_msg):
    return [
        float(quaternion_msg.x),
        float(quaternion_msg.y),
        float(quaternion_msg.z),
        float(quaternion_msg.w),
    ]


def quaternion_conjugate(quaternion):
    return [-quaternion[0], -quaternion[1], -quaternion[2], quaternion[3]]


def quaternion_multiply(lhs, rhs):
    lx, ly, lz, lw = lhs
    rx, ry, rz, rw = rhs
    return [
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ]


def normalize_quaternion(quaternion):
    norm = math.sqrt(sum([value * value for value in quaternion]))
    if norm < 1e-9:
        return [0.0, 0.0, 0.0, 1.0]
    return [value / norm for value in quaternion]


def rotate_vector(quaternion, vector):
    q = normalize_quaternion(quaternion)
    pure = [float(vector[0]), float(vector[1]), float(vector[2]), 0.0]
    rotated = quaternion_multiply(quaternion_multiply(q, pure), quaternion_conjugate(q))
    return np.array(rotated[:3], dtype=float)


def world_to_body(vector_world, body_orientation):
    return rotate_vector(quaternion_conjugate(normalize_quaternion(body_orientation)), vector_world)


def vector_dot(lhs, rhs):
    return float(sum([float(lhs[index]) * float(rhs[index]) for index in range(3)]))


def parse_unit_vector(values, fallback):
    try:
        parsed = np.array([float(value) for value in values], dtype=float)
    except (TypeError, ValueError):
        parsed = np.array(fallback, dtype=float)
    if parsed.shape != (3,):
        parsed = np.array(fallback, dtype=float)
    norm = np.linalg.norm(parsed)
    if norm < 1e-9:
        parsed = np.array(fallback, dtype=float)
        norm = np.linalg.norm(parsed)
    return parsed / max(norm, 1e-9)


class StanceForceOptimizer(object):
    def __init__(self):
        rospy.init_node("stance_force_optimizer", anonymous=False)

        if np is None:
            raise ImportError("stance_force_optimizer requires numpy for the QP solve")

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/stance_optimizer/" + name, default))

        self.mass_kg = float(get_cfg("mass_kg", rospy.get_param("/robot/mass_kg", 8.5)))
        self.mu = float(get_cfg("friction_coefficient", rospy.get_param("/wall/default_friction_coefficient", 0.5)))
        self.adhesion_mu = float(get_cfg("adhesion_friction_coefficient", rospy.get_param("/wall/adhesion_friction_coefficient", self.mu)))
        self.adhesion_safety_factor = float(get_cfg("adhesion_safety_factor", rospy.get_param("/wall/adhesion_safety_factor", 1.15)))
        self.max_normal_force = float(get_cfg("max_normal_force", rospy.get_param("/wall/max_normal_force_n", 150.0)))
        self.min_normal_force = float(get_cfg("min_normal_force_n", get_cfg("min_normal_force", 5.0)))
        self.leg_names = get_cfg("leg_names", ["lf", "rf", "rr", "lr"])
        self.contact_point_source = str(get_cfg("contact_point_source", rospy.get_param("/robot/active_endpoint", "universal_joint_center"))).lower()
        self.regularization = float(get_cfg("regularization", 1e-3))
        self.gradient_step_size = float(get_cfg("gradient_step_size", 0.02))
        self.qp_iterations = int(get_cfg("qp_iterations", 80))

        self.kp_linear = np.array(get_cfg("kp_linear", [60.0, 60.0, 80.0]), dtype=float)
        self.kd_linear = np.array(get_cfg("kd_linear", [18.0, 18.0, 20.0]), dtype=float)
        self.kp_angular = np.array(get_cfg("kp_angular", [18.0, 18.0, 12.0]), dtype=float)
        self.kd_angular = np.array(get_cfg("kd_angular", [2.5, 2.5, 1.5]), dtype=float)
        self.gravity_world = np.array(get_cfg("gravity_world", [0.0, 0.0, -9.81]), dtype=float)

        self.force_tracking_weight = np.array(get_cfg("force_tracking_weight", [1.0, 1.0, 1.0]), dtype=float)
        self.torque_tracking_weight = np.array(get_cfg("torque_tracking_weight", [0.15, 0.15, 0.1]), dtype=float)
        legacy_axis = str(get_cfg("contact_normal_axis", rospy.get_param("/wall/legacy_contact_normal_axis", "z"))).lower()
        legacy_fallback = {
            "x": [1.0, 0.0, 0.0],
            "y": [0.0, 1.0, 0.0],
            "z": [0.0, 0.0, 1.0],
        }.get(legacy_axis, [0.0, 0.0, 1.0])
        self.wall_normal_body = parse_unit_vector(
            get_cfg("wall_normal_body", rospy.get_param("/wall/normal_body", legacy_fallback)),
            legacy_fallback,
        )

        self.contact_positions = self._build_contact_positions()
        self.last_state = EstimatedState()
        self.has_state = False

        if self.contact_point_source != "universal_joint_center":
            rospy.logwarn("stance_force_optimizer forcing contact_point_source to universal_joint_center for non-adhesive quadruped-style support-force QP")
            self.contact_point_source = "universal_joint_center"

        self.pubs = {
            leg: rospy.Publisher("/control/stance_wrench/" + leg, StanceWrenchCommand, queue_size=20)
            for leg in self.leg_names
        }
        rospy.Subscriber("/state/estimated", EstimatedState, self.state_callback, queue_size=20)
        rospy.Subscriber("/control/body_reference", BodyReference, self.reference_callback, queue_size=20)

    def _build_contact_positions(self):
        base_radius = float(rospy.get_param("/gait_controller/base_radius", 203.06)) / 1000.0
        nominal_x = float(rospy.get_param("/gait_controller/nominal_x", 120.0)) / 1000.0
        nominal_y = float(rospy.get_param("/gait_controller/nominal_y", 0.0)) / 1000.0
        nominal_z = float(
            rospy.get_param(
                "/gait_controller/nominal_universal_joint_center_z",
                float(rospy.get_param("/gait_controller/nominal_z", -299.2))
                + abs(float(rospy.get_param("/gait_controller/link_d6", -13.5)) + float(rospy.get_param("/gait_controller/link_d7", -106.7))),
            )
        ) / 1000.0
        operating_x = float(
            rospy.get_param(
                "/gait_controller/operating_universal_joint_center_x",
                nominal_x * 1000.0,
            )
        ) / 1000.0
        operating_y = float(
            rospy.get_param(
                "/gait_controller/operating_universal_joint_center_y",
                nominal_y * 1000.0,
            )
        ) / 1000.0
        operating_z = float(
            rospy.get_param(
                "/gait_controller/operating_universal_joint_center_z",
                nominal_z * 1000.0,
            )
        ) / 1000.0
        legs = rospy.get_param("/legs", {})

        positions = {}
        for leg_name in self.leg_names:
            leg_cfg = legs.get(leg_name, {})
            hip_yaw_deg = float(leg_cfg.get("hip_yaw_deg", 0.0))
            hip_yaw = math.radians(hip_yaw_deg)
            cos_yaw = math.cos(hip_yaw)
            sin_yaw = math.sin(hip_yaw)

            hip_origin = np.array([base_radius * cos_yaw, base_radius * sin_yaw, 0.0], dtype=float)
            local_nominal = np.array([operating_x, operating_y, operating_z], dtype=float)
            rotation = np.array(
                [
                    [cos_yaw, -sin_yaw, 0.0],
                    [sin_yaw, cos_yaw, 0.0],
                    [0.0, 0.0, 1.0],
                ],
                dtype=float,
            )
            positions[leg_name] = hip_origin + rotation.dot(local_nominal)
        return positions

    def _contact_position_for_leg(self, leg_name):
        if self.has_state and len(self.last_state.universal_joint_center_positions) == len(self.leg_names):
            try:
                leg_index = self.leg_names.index(leg_name)
                point = self.last_state.universal_joint_center_positions[leg_index]
                return np.array([float(point.x), float(point.y), float(point.z)], dtype=float)
            except (ValueError, AttributeError, IndexError):
                pass
        return np.array(self.contact_positions[leg_name], dtype=float)

    def _split_force_components(self, force_vector):
        force = np.array(force_vector, dtype=float)
        normal_force = max(0.0, vector_dot(force, self.wall_normal_body))
        tangential_force = np.linalg.norm(force - normal_force * self.wall_normal_body)
        return normal_force, tangential_force

    def _required_adhesion_force(self, force_vector):
        normal_force, tangential_force = self._split_force_components(force_vector)
        tangential_term = tangential_force / max(self.adhesion_mu, 1e-6)
        required_force = self.adhesion_safety_factor * (normal_force + tangential_term)
        return required_force, tangential_force

    def state_callback(self, msg):
        self.last_state = msg
        self.has_state = True

    def _pose_position_vector(self, pose_msg):
        return np.array([pose_msg.position.x, pose_msg.position.y, pose_msg.position.z], dtype=float)

    def _twist_linear_vector(self, twist_msg):
        return np.array([twist_msg.linear.x, twist_msg.linear.y, twist_msg.linear.z], dtype=float)

    def _twist_angular_vector(self, twist_msg):
        return np.array([twist_msg.angular.x, twist_msg.angular.y, twist_msg.angular.z], dtype=float)

    def _orientation_error(self, desired_quat_msg, current_quat_msg):
        desired = normalize_quaternion(quaternion_to_xyzw(desired_quat_msg))
        current = normalize_quaternion(quaternion_to_xyzw(current_quat_msg))
        error_quat = normalize_quaternion(quaternion_multiply(quaternion_conjugate(current), desired))
        sign = 1.0 if error_quat[3] >= 0.0 else -1.0
        return 2.0 * sign * np.array(error_quat[:3], dtype=float)

    def _desired_support_mask(self, body_reference):
        if len(body_reference.support_mask) == len(self.leg_names):
            return [bool(value) for value in body_reference.support_mask]
        return [True] * len(self.leg_names)

    def _state_mask_or_default(self, attr_name, default_mask):
        values = getattr(self.last_state, attr_name, [])
        if len(values) == len(self.leg_names):
            return [bool(value) for value in values]
        return list(default_mask)

    def _estimated_mask_or_default(self, values, default_mask):
        if len(values) == len(self.leg_names) and any([bool(value) for value in values]):
            return [bool(value) for value in values]
        return list(default_mask)

    def _compute_target_wrench(self, body_reference):
        desired_pos = self._pose_position_vector(body_reference.pose)
        desired_vel = self._twist_linear_vector(body_reference.twist)
        desired_omega_world = self._twist_angular_vector(body_reference.twist)

        body_orientation = [0.0, 0.0, 0.0, 1.0]

        if self.has_state:
            current_pos = self._pose_position_vector(self.last_state.pose)
            current_vel = self._twist_linear_vector(self.last_state.twist)
            current_omega = self._twist_angular_vector(self.last_state.twist)
            body_orientation = quaternion_to_xyzw(self.last_state.pose.orientation)
            orientation_error = self._orientation_error(body_reference.pose.orientation, self.last_state.pose.orientation)
        else:
            current_pos = np.zeros(3, dtype=float)
            current_vel = np.zeros(3, dtype=float)
            current_omega = np.zeros(3, dtype=float)
            orientation_error = np.zeros(3, dtype=float)

        position_error = desired_pos - current_pos
        velocity_error = desired_vel - current_vel
        desired_omega = world_to_body(desired_omega_world, body_orientation)
        omega_error = desired_omega - current_omega

        desired_force_world = self.mass_kg * (
            self.kp_linear * position_error + self.kd_linear * velocity_error - self.gravity_world
        )
        desired_force = world_to_body(desired_force_world, body_orientation)
        desired_torque = self.kp_angular * orientation_error + self.kd_angular * omega_error
        return desired_force, desired_torque

    def _build_qp_matrices(self, active_legs, desired_force, desired_torque):
        active_count = len(active_legs)
        a_matrix = np.zeros((6, 3 * active_count), dtype=float)

        for leg_index, leg_name in enumerate(active_legs):
            column = 3 * leg_index
            a_matrix[0:3, column:column + 3] = np.eye(3)
            contact_position = self._contact_position_for_leg(leg_name)
            skew = np.array(
                [
                    [0.0, -contact_position[2], contact_position[1]],
                    [contact_position[2], 0.0, -contact_position[0]],
                    [-contact_position[1], contact_position[0], 0.0],
                ],
                dtype=float,
            )
            a_matrix[3:6, column:column + 3] = skew

        weight_matrix = np.diag(np.concatenate([self.force_tracking_weight, self.torque_tracking_weight]))
        desired_wrench = np.concatenate([desired_force, desired_torque])
        hessian = a_matrix.T.dot(weight_matrix).dot(a_matrix) + self.regularization * np.eye(3 * active_count)
        gradient = a_matrix.T.dot(weight_matrix).dot(desired_wrench)
        return hessian, gradient

    def _project_contact_force(self, force_vector):
        projected = np.array(force_vector, dtype=float)
        normal_value = vector_dot(projected, self.wall_normal_body)
        tangent_vector = projected - normal_value * self.wall_normal_body

        normal_value = min(self.max_normal_force, max(self.min_normal_force, normal_value))
        tangent_norm = np.linalg.norm(tangent_vector)
        max_tangent = self.mu * max(normal_value, 0.0)
        if tangent_norm > max_tangent and tangent_norm > 1e-9:
            tangent_vector = tangent_vector * (max_tangent / tangent_norm)
        return normal_value * self.wall_normal_body + tangent_vector

    def _solve_projected_qp(self, hessian, gradient, active_count):
        solution = np.zeros(3 * active_count, dtype=float)
        if active_count == 0:
            return solution

        normal_guess = min(self.max_normal_force, max(self.min_normal_force, (self.mass_kg * 9.81) / float(active_count)))
        for leg_index in range(active_count):
            start = 3 * leg_index
            solution[start:start + 3] = normal_guess * self.wall_normal_body

        for _ in range(self.qp_iterations):
            gradient_now = hessian.dot(solution) - gradient
            trial = solution - self.gradient_step_size * gradient_now
            for leg_index in range(active_count):
                start = 3 * leg_index
                trial[start:start + 3] = self._project_contact_force(trial[start:start + 3])
            solution = trial
        return solution

    def _publish_leg_commands(self, stamp, active_legs, support_mask, solution, planned_support_mask, early_contact_mask, actual_contact_mask):
        force_by_leg = {}
        for leg_index, leg_name in enumerate(active_legs):
            force_by_leg[leg_name] = solution[3 * leg_index:3 * leg_index + 3]

        for leg_index, leg_name in enumerate(self.leg_names):
            out = StanceWrenchCommand()
            out.header.stamp = stamp
            out.leg_name = leg_name
            out.normal_force_limit = self.max_normal_force
            out.planned_support = leg_index < len(planned_support_mask) and bool(planned_support_mask[leg_index])
            out.early_contact = leg_index < len(early_contact_mask) and bool(early_contact_mask[leg_index])
            out.actual_contact = leg_index < len(actual_contact_mask) and bool(actual_contact_mask[leg_index])
            out.tangential_force_magnitude = 0.0
            out.required_adhesion_force = 0.0
            out.wrench = Wrench()
            is_active = leg_index < len(support_mask) and bool(support_mask[leg_index]) and leg_name in force_by_leg
            out.active = is_active
            if is_active:
                force_vector = force_by_leg[leg_name]
                out.wrench.force.x = float(force_vector[0])
                out.wrench.force.y = float(force_vector[1])
                out.wrench.force.z = float(force_vector[2])
                required_adhesion_force, tangential_force = self._required_adhesion_force(force_vector)
                out.tangential_force_magnitude = float(tangential_force)
                out.required_adhesion_force = float(required_adhesion_force)
            self.pubs[leg_name].publish(out)

    def reference_callback(self, msg):
        desired_support_mask = self._desired_support_mask(msg)
        estimated_support_mask = self._estimated_mask_or_default(self.last_state.support_mask, desired_support_mask)
        estimated_attachment_mask = self._estimated_mask_or_default(
            getattr(self.last_state, "attachment_ready_mask", []),
            self._estimated_mask_or_default(self.last_state.adhesion_mask, [True] * len(self.leg_names)),
        )
        planned_support_mask = self._state_mask_or_default("plan_support_mask", desired_support_mask)
        early_contact_mask = self._state_mask_or_default("early_contact_mask", [False] * len(self.leg_names))
        actual_contact_mask = self._state_mask_or_default("contact_mask", estimated_support_mask)

        combined_mask = [
            bool(actual_contact_mask[index]) and bool(estimated_attachment_mask[index])
            for index in range(len(self.leg_names))
        ]
        active_legs = [leg_name for leg_name, active in zip(self.leg_names, combined_mask) if active]

        if not active_legs:
            self._publish_leg_commands(
                rospy.Time.now(),
                [],
                combined_mask,
                np.zeros(0, dtype=float),
                planned_support_mask,
                early_contact_mask,
                actual_contact_mask,
            )
            return

        desired_force, desired_torque = self._compute_target_wrench(msg)
        hessian, gradient = self._build_qp_matrices(active_legs, desired_force, desired_torque)
        solution = self._solve_projected_qp(hessian, gradient, len(active_legs))
        self._publish_leg_commands(
            rospy.Time.now(),
            active_legs,
            combined_mask,
            solution,
            planned_support_mask,
            early_contact_mask,
            actual_contact_mask,
        )


if __name__ == "__main__":
    StanceForceOptimizer()
    rospy.spin()