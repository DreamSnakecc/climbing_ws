#!/usr/bin/env python

import math

import rospy
from climbing_msgs.msg import BodyReference, EstimatedState
from geometry_msgs.msg import Pose, Twist
from std_msgs.msg import Bool


def clamp(value, lower_bound, upper_bound):
    return max(lower_bound, min(upper_bound, value))


def smoothstep5(phase):
    phase = clamp(phase, 0.0, 1.0)
    return phase * phase * phase * (10.0 + phase * (-15.0 + 6.0 * phase))


def quaternion_from_rpy(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


def vector_scale(vector, scalar):
    return [scalar * value for value in vector]


def vector_add(lhs, rhs):
    return [lhs[index] + rhs[index] for index in range(len(lhs))]


class BodyPlanner(object):
    GAIT_STANCE = 0
    GAIT_CRAWL = 1
    GAIT_TROT = 2

    def __init__(self):
        rospy.init_node("body_planner", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/body_planner/" + name, default))

        self.rate_hz = float(get_cfg("publish_rate_hz", 50.0))
        self.leg_names = [str(name) for name in get_cfg("leg_names", ["lf", "rf", "lr", "rr"])]
        self.gait = str(get_cfg("gait", rospy.get_param("/gait_controller/gait", "crawl"))).lower()
        self.gait_frequency_hz = float(get_cfg("gait_frequency_hz", rospy.get_param("/gait_controller/gait_frequency", 0.25)))
        self.crawl_swing_ratio = float(get_cfg("crawl_swing_ratio", rospy.get_param("/gait_controller/crawl_swing_ratio", 0.25)))
        self.trot_swing_ratio = float(get_cfg("trot_swing_ratio", rospy.get_param("/gait_controller/trot_swing_ratio", 0.50)))
        self.startup_hold_s = max(float(get_cfg("startup_hold_s", 1.0)), 0.0)
        self.step_transition_margin = float(get_cfg("step_transition_margin", 0.08))
        self.nominal_position = [float(value) for value in get_cfg("nominal_position_m", [0.0, 0.0, 0.0])]
        self.nominal_rpy = [math.radians(float(value)) for value in get_cfg("nominal_rpy_deg", [0.0, 0.0, 0.0])]
        self.linear_velocity_world = [float(value) for value in get_cfg("linear_velocity_world_mps", [0.0, 0.0, 0.0])]
        self.angular_velocity_world = [float(value) for value in get_cfg("angular_velocity_world_rps", [0.0, 0.0, 0.0])]
        self.com_shift_gain = [float(value) for value in get_cfg("com_shift_gain_m", [0.025, 0.03, 0.0])]
        self.orientation_shift_gain = [
            math.radians(float(value)) for value in get_cfg("orientation_shift_gain_deg", [6.0, 4.0, 0.0])
        ]
        self.slip_pause_threshold = float(get_cfg("slip_pause_threshold", 0.92))
        self.slip_hold_all_support = bool(get_cfg("slip_hold_all_support", True))

        self.phase_offsets = self._build_phase_offsets()
        self.hip_offsets = self._build_hip_offsets()
        self.start_time = rospy.Time.now().to_sec()
        self.estimated_state = EstimatedState()
        self.has_estimated_state = False
        self.mission_active = False

        self.pub = rospy.Publisher("/control/body_reference", BodyReference, queue_size=20)
        rospy.Subscriber("/state/estimated", EstimatedState, self.estimated_state_callback, queue_size=20)
        rospy.Subscriber("/control/mission_active", Bool, self.mission_active_callback, queue_size=20)

    def _build_phase_offsets(self):
        default_offsets = {"lf": 0.0, "rf": 0.25, "lr": 0.75, "rr": 0.50}
        if self.gait == "trot":
            default_offsets = {"lf": 0.0, "rf": 0.5, "lr": 0.5, "rr": 0.0}

        phase_cfg = rospy.get_param("/body_planner/phase_offsets", {})
        offsets = {}
        for leg_name in self.leg_names:
            offsets[leg_name] = float(phase_cfg.get(leg_name, default_offsets.get(leg_name, 0.0)))
        return offsets

    def _build_hip_offsets(self):
        base_radius = float(rospy.get_param("/gait_controller/base_radius", 203.06)) / 1000.0
        leg_cfg = rospy.get_param("/legs", {})
        offsets = {}
        for leg_name in self.leg_names:
            hip_yaw_deg = float(leg_cfg.get(leg_name, {}).get("hip_yaw_deg", 0.0))
            hip_yaw = math.radians(hip_yaw_deg)
            offsets[leg_name] = [base_radius * math.cos(hip_yaw), base_radius * math.sin(hip_yaw), 0.0]
        return offsets

    def estimated_state_callback(self, msg):
        self.estimated_state = msg
        self.has_estimated_state = True

    def mission_active_callback(self, msg):
        self.mission_active = bool(msg.data)

    def _gait_mode_id(self):
        if self.gait == "trot":
            return self.GAIT_TROT
        if self.gait == "crawl":
            return self.GAIT_CRAWL
        return self.GAIT_STANCE

    def _swing_ratio(self):
        if self.gait == "trot":
            return clamp(self.trot_swing_ratio, 0.05, 0.95)
        if self.gait == "crawl":
            return clamp(self.crawl_swing_ratio, 0.05, 0.95)
        return 0.0

    def _slip_paused(self):
        if not self.has_estimated_state or not self.slip_hold_all_support:
            return False
        return any([float(value) >= self.slip_pause_threshold for value in self.estimated_state.slip_risk])

    def _support_mask(self, elapsed_s):
        if not self.mission_active:
            return [True] * len(self.leg_names)
        if self.gait not in ["crawl", "trot"]:
            return [True] * len(self.leg_names)
        if elapsed_s < self.startup_hold_s or self._slip_paused():
            return [True] * len(self.leg_names)

        swing_ratio = self._swing_ratio()
        cycle_time = elapsed_s - self.startup_hold_s
        cycle_phase = (cycle_time * self.gait_frequency_hz) % 1.0
        support_mask = []
        for leg_name in self.leg_names:
            leg_phase = (cycle_phase + self.phase_offsets.get(leg_name, 0.0)) % 1.0
            in_transition = leg_phase < self.step_transition_margin or leg_phase > (1.0 - self.step_transition_margin)
            support_mask.append(not (leg_phase < swing_ratio and not in_transition))
        return support_mask

    def _body_shift(self, support_mask):
        swing_legs = [self.leg_names[index] for index, is_support in enumerate(support_mask) if not is_support]
        if not swing_legs:
            return [0.0, 0.0, 0.0]

        shift = [0.0, 0.0, 0.0]
        for leg_name in swing_legs:
            hip = self.hip_offsets.get(leg_name, [0.0, 0.0, 0.0])
            shift[0] -= hip[0]
            shift[1] -= hip[1]
        scale = 1.0 / max(float(len(swing_legs)), 1.0)
        averaged = vector_scale(shift, scale)

        normalized = [0.0, 0.0, 0.0]
        for axis in [0, 1, 2]:
            denominator = max(abs(averaged[axis]), 1e-6)
            normalized[axis] = averaged[axis] / denominator if axis < 2 and abs(averaged[axis]) > 1e-6 else 0.0

        return [
            self.com_shift_gain[0] * normalized[0],
            self.com_shift_gain[1] * normalized[1],
            self.com_shift_gain[2] * normalized[2],
        ]

    def _body_orientation_bias(self, shift):
        return [
            -self.orientation_shift_gain[0] * shift[1] / max(abs(self.com_shift_gain[1]), 1e-6) if abs(self.com_shift_gain[1]) > 1e-6 else 0.0,
            self.orientation_shift_gain[1] * shift[0] / max(abs(self.com_shift_gain[0]), 1e-6) if abs(self.com_shift_gain[0]) > 1e-6 else 0.0,
            self.orientation_shift_gain[2] * shift[2] / max(abs(self.com_shift_gain[2]), 1e-6) if abs(self.com_shift_gain[2]) > 1e-6 else 0.0,
        ]

    def _build_pose_and_twist(self, elapsed_s, support_mask):
        if self.startup_hold_s <= 1e-6:
            hold_scale = 1.0
        else:
            hold_scale = smoothstep5(clamp(elapsed_s / self.startup_hold_s, 0.0, 1.0))

        if not self.mission_active:
            hold_scale = 0.0

        shift = self._body_shift(support_mask)
        progression = vector_scale(self.linear_velocity_world, max(elapsed_s - self.startup_hold_s, 0.0))
        position = vector_add(self.nominal_position, progression)
        position = vector_add(position, shift)

        orientation_bias = self._body_orientation_bias(shift)
        yaw_progress = hold_scale * self.angular_velocity_world[2] * max(elapsed_s - self.startup_hold_s, 0.0)
        rpy = [
            self.nominal_rpy[0] + orientation_bias[0],
            self.nominal_rpy[1] + orientation_bias[1],
            self.nominal_rpy[2] + yaw_progress + orientation_bias[2],
        ]
        quat = quaternion_from_rpy(rpy[0], rpy[1], rpy[2])

        pose = Pose()
        pose.position.x = position[0]
        pose.position.y = position[1]
        pose.position.z = position[2]
        pose.orientation.x = quat[0]
        pose.orientation.y = quat[1]
        pose.orientation.z = quat[2]
        pose.orientation.w = quat[3]

        twist = Twist()
        twist.linear.x = hold_scale * self.linear_velocity_world[0]
        twist.linear.y = hold_scale * self.linear_velocity_world[1]
        twist.linear.z = hold_scale * self.linear_velocity_world[2]
        twist.angular.x = hold_scale * self.angular_velocity_world[0]
        twist.angular.y = hold_scale * self.angular_velocity_world[1]
        twist.angular.z = hold_scale * self.angular_velocity_world[2]
        return pose, twist

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            elapsed_s = rospy.Time.now().to_sec() - self.start_time
            support_mask = self._support_mask(elapsed_s)
            pose, twist = self._build_pose_and_twist(elapsed_s, support_mask)

            msg = BodyReference()
            msg.header.stamp = rospy.Time.now()
            msg.pose = pose
            msg.twist = twist
            msg.gait_mode = self._gait_mode_id()
            msg.support_mask = support_mask
            self.pub.publish(msg)
            rate.sleep()


if __name__ == "__main__":
    node = BodyPlanner()
    node.spin()