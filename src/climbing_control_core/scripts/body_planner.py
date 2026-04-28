#!/usr/bin/env python

import math

import rospy
from climbing_msgs.msg import BodyReference, EstimatedState, LegCenterCommand
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
    GAIT_SEQUENTIAL = 3

    def __init__(self):
        rospy.init_node("body_planner", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/body_planner/" + name, default))

        self.rate_hz = float(get_cfg("publish_rate_hz", 50.0))
        self.leg_names = [str(name) for name in get_cfg("leg_names", ["lf", "rf", "rr", "lr"])]
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

        # Sequential gait parameters
        self.swing_timeout_s = max(float(get_cfg("swing_timeout_s", 3.0)), 0.5)
        self.gait_sequence = [str(name) for name in get_cfg("gait_sequence", self.leg_names)]
        self.all_hold_dwell_s = max(float(get_cfg("all_hold_dwell_s", 0.2)), 0.0)

        self.phase_offsets = self._build_phase_offsets()
        self.hip_offsets = self._build_hip_offsets()
        self.start_time = rospy.Time.now().to_sec()
        self.estimated_state = EstimatedState()
        self.has_estimated_state = False
        self.mission_active = False

        # Sequential gait state
        self._gait_state = "IDLE"            # IDLE, ALL_HOLD, SWING_ACTIVE
        self._current_swing_idx = -1         # index in self.gait_sequence, -1 = no active swing
        self._swing_start_time = None        # rospy.Time when current swing started
        self._swing_progression = [0.0, 0.0, 0.0]   # cumulative body progression (m) contributed by completed swings
        self._leg_support_state = {leg: True for leg in self.leg_names}  # from swing_leg_target
        self._all_hold_start_time = None     # when ALL_HOLD was entered (for dwell)

        self.pub = rospy.Publisher("/control/body_reference", BodyReference, queue_size=20)
        rospy.Subscriber("/state/estimated", EstimatedState, self.estimated_state_callback, queue_size=20)
        rospy.Subscriber("/control/mission_active", Bool, self.mission_active_callback, queue_size=20)
        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self.swing_target_callback, queue_size=20)

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
        now_active = bool(msg.data)
        if now_active and not self.mission_active:
            self.start_time = rospy.Time.now().to_sec()
            # Reset sequential gait state on mission activation
            self._gait_state = "IDLE"
            self._current_swing_idx = -1
            self._swing_start_time = None
            self._swing_progression = [0.0, 0.0, 0.0]
            self._leg_support_state = {leg: True for leg in self.leg_names}
        self.mission_active = now_active

    def swing_target_callback(self, msg):
        """Track per-leg support state from swing_leg_controller output."""
        if msg.leg_name in self._leg_support_state:
            self._leg_support_state[msg.leg_name] = bool(msg.support_leg)

    def _gait_mode_id(self):
        if self.gait == "trot":
            return self.GAIT_TROT
        if self.gait == "crawl":
            return self.GAIT_CRAWL
        if self.gait == "sequential":
            return self.GAIT_SEQUENTIAL
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

    # ------------------------------------------------------------------ #
    # support_mask dispatch
    # ------------------------------------------------------------------ #
    def _support_mask(self, elapsed_s):
        if not self.mission_active:
            self._gait_state = "IDLE"
            return [True] * len(self.leg_names)
        if self.gait == "sequential":
            return self._support_mask_sequential(elapsed_s)
        if self.gait not in ["crawl", "trot"]:
            return [True] * len(self.leg_names)
        return self._support_mask_timed(elapsed_s)

    def _support_mask_timed(self, elapsed_s):
        """Original time-phased crawl/trot gait logic (unchanged)."""
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

    def _support_mask_sequential(self, elapsed_s):
        """Event-driven sequential gait: one leg swings at a time, gated by adhesion.

        State machine:
          IDLE -> ALL_HOLD : on first tick with mission_active
          ALL_HOLD -> SWING_ACTIVE : dwell elapsed
          SWING_ACTIVE -> ALL_HOLD : swing leg returned to support, or timeout
        """
        # Startup hold: all support
        if elapsed_s < self.startup_hold_s or self._slip_paused():
            self._gait_state = "ALL_HOLD"
            self._current_swing_idx = -1
            self._swing_start_time = None
            self._all_hold_start_time = rospy.Time.now()
            return [True] * len(self.leg_names)

        now = rospy.Time.now()

        # --- IDLE: transition to ALL_HOLD ---
        if self._gait_state == "IDLE":
            self._gait_state = "ALL_HOLD"
            self._current_swing_idx = -1
            self._swing_start_time = None
            self._all_hold_start_time = now
            self._swing_progression = [0.0, 0.0, 0.0]
            return [True] * len(self.leg_names)

        # --- ALL_HOLD: wait for dwell, then start next swing ---
        if self._gait_state == "ALL_HOLD":
            # On first entry, record the start time
            if self._all_hold_start_time is None:
                self._all_hold_start_time = now

            dwell_ok = (now - self._all_hold_start_time).to_sec() >= self.all_hold_dwell_s

            if dwell_ok:
                # Advance to next leg in sequence
                self._current_swing_idx = (self._current_swing_idx + 1) % len(self.gait_sequence)
                swing_leg = self.gait_sequence[self._current_swing_idx]
                self._gait_state = "SWING_ACTIVE"
                self._swing_start_time = now
                # Build mask: only the swing leg is False
                mask = [True] * len(self.leg_names)
                mask[self.leg_names.index(swing_leg)] = False
                # Freeze the cumulative progression so the body holds position
                # (advancement is added by _build_pose_and_twist_sequential during SWING_ACTIVE)
                self._all_hold_start_time = None
                return mask
            return [True] * len(self.leg_names)

        # --- SWING_ACTIVE: wait for swing completion or timeout ---
        if self._gait_state == "SWING_ACTIVE":
            swing_leg = self.gait_sequence[self._current_swing_idx]
            swing_idx = self.leg_names.index(swing_leg)

            # Check timeout
            swing_elapsed = (now - self._swing_start_time).to_sec() if self._swing_start_time is not None else 0.0
            timed_out = swing_elapsed >= self.swing_timeout_s

            # Check if the swing leg has returned to support
            leg_support = self._leg_support_state.get(swing_leg, True)

            if leg_support or timed_out:
                if timed_out and not leg_support:
                    rospy.logwarn_throttle(
                        3.0,
                        "body_planner: swing timeout for %s (%.1fs elapsed), forcing completion",
                        swing_leg, swing_elapsed,
                    )
                # Swing complete (or timed out). Record progression from this swing.
                self._swing_progression = self._compute_swing_progression()
                self._gait_state = "ALL_HOLD"
                self._all_hold_start_time = now
                self._swing_start_time = None
                return [True] * len(self.leg_names)

            # Still swinging: keep the swing leg in swing mode
            mask = [True] * len(self.leg_names)
            mask[swing_idx] = False
            return mask

        # Fallback (should not reach here)
        self._gait_state = "ALL_HOLD"
        self._all_hold_start_time = now
        return [True] * len(self.leg_names)

    def _swing_leg_index(self, support_mask):
        """Return index of the leg whose support is False, or -1 if all support."""
        for i, is_support in enumerate(support_mask):
            if not is_support:
                return i
        return -1

    # ------------------------------------------------------------------ #
    # body shift / CoM
    # ------------------------------------------------------------------ #
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

    def _compute_swing_progression(self):
        """Compute the cumulative body position progression contributed by
        completed and in-progress swing cycles.

        For SWING_ACTIVE: base (completed swings) + incremental progress from current swing.
        For ALL_HOLD/IDLE: just the completed base.
        """
        if self._gait_state == "SWING_ACTIVE" and self._swing_start_time is not None:
            swing_elapsed = (rospy.Time.now() - self._swing_start_time).to_sec()
            increment = vector_scale(self.linear_velocity_world, swing_elapsed)
            return vector_add(self._swing_progression, increment)
        return list(self._swing_progression)

    # ------------------------------------------------------------------ #
    # pose and twist computation
    # ------------------------------------------------------------------ #
    def _build_pose_and_twist(self, elapsed_s, support_mask):
        if self.gait == "sequential":
            return self._build_pose_and_twist_sequential(elapsed_s, support_mask)

        # --- Original timed-gait logic (crawl / trot / stance) ---
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

    def _build_pose_and_twist_sequential(self, elapsed_s, support_mask):
        """Build body pose and twist for the sequential adhesion-driven gait.

        Body position advances only during SWING_ACTIVE (when a leg is being swung).
        During ALL_HOLD, position holds steady while all legs are in support.
        """
        if self.startup_hold_s <= 1e-6:
            hold_scale = 1.0
        else:
            hold_scale = smoothstep5(clamp(elapsed_s / self.startup_hold_s, 0.0, 1.0))

        if not self.mission_active:
            hold_scale = 0.0

        # Body shift toward support legs (away from swing leg)
        shift = self._body_shift(support_mask)

        # Progression: cumulative step-wise advancement
        progression = self._compute_swing_progression()
        position = vector_add(self.nominal_position, progression)
        position = vector_add(position, shift)

        # Orientation bias (same as timed gait)
        orientation_bias = self._body_orientation_bias(shift)
        # Yaw progress only applies during active swing
        yaw_progress = 0.0
        if self._gait_state == "SWING_ACTIVE" and self._swing_start_time is not None:
            swing_elapsed = (rospy.Time.now() - self._swing_start_time).to_sec()
            yaw_progress = hold_scale * self.angular_velocity_world[2] * swing_elapsed
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

        # Twist: non-zero only during active swing
        twist = Twist()
        if self._gait_state == "SWING_ACTIVE" and self._swing_start_time is not None:
            twist.linear.x = hold_scale * self.linear_velocity_world[0]
            twist.linear.y = hold_scale * self.linear_velocity_world[1]
            twist.linear.z = hold_scale * self.linear_velocity_world[2]
            twist.angular.x = hold_scale * self.angular_velocity_world[0]
            twist.angular.y = hold_scale * self.angular_velocity_world[1]
            twist.angular.z = hold_scale * self.angular_velocity_world[2]
        return pose, twist

    # ------------------------------------------------------------------ #
    # main loop
    # ------------------------------------------------------------------ #
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
