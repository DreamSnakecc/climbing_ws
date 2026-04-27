#!/usr/bin/env python

import rospy
from climbing_msgs.msg import AdhesionCommand, EstimatedState, LegCenterCommand, StanceWrenchCommand
from std_msgs.msg import Bool, String


class MissionSupervisor(object):
    STATE_INIT = "INIT"
    STATE_STICK = "STICK"
    STATE_CLIMB = "CLIMB"
    STATE_PAUSE = "PAUSE"
    STATE_FAULT = "FAULT"

    def __init__(self):
        rospy.init_node("mission_supervisor", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/mission_supervisor/" + name, default))

        self.rate_hz = float(get_cfg("publish_rate_hz", 20.0))
        self.auto_start = bool(get_cfg("auto_start", True))
        self.enable_auto_adhesion_commands = bool(get_cfg("enable_auto_adhesion_commands", True))
        self.auto_resume = bool(get_cfg("auto_resume", False))
        self.init_duration_s = max(float(get_cfg("init_duration_s", 1.0)), 0.0)
        self.stick_timeout_s = max(float(get_cfg("stick_timeout_s", 3.0)), 0.1)
        self.pause_on_safe_mode = bool(get_cfg("pause_on_safe_mode", True))
        self.pause_on_high_slip = bool(get_cfg("pause_on_high_slip", True))
        self.pause_on_adhesion_loss = bool(get_cfg("pause_on_adhesion_loss", True))
        self.high_slip_threshold = float(get_cfg("high_slip_threshold", 0.92))
        self.adhesion_required_count = int(get_cfg("adhesion_required_count", 4))
        self.attach_rpm = float(get_cfg("attach_rpm", 3200.0))
        self.climb_rpm = float(get_cfg("climb_rpm", 3200.0))
        self.pause_rpm = float(get_cfg("pause_rpm", 3200.0))
        self.fault_rpm = float(get_cfg("fault_rpm", 3400.0))
        self.release_rpm = float(get_cfg("release_rpm", 0.0))
        self.swing_release_rpm = float(get_cfg("swing_release_rpm", 0.0))
        self.touchdown_rpm = float(get_cfg("touchdown_rpm", 3400.0))
        self.touchdown_boost_duration_s = max(float(get_cfg("touchdown_boost_duration_s", 0.25)), 0.0)
        self.pre_attach_time_s = max(float(get_cfg("pre_attach_time_s", 0.12)), 0.0)
        self.boost_after_preload_without_contact = bool(
            get_cfg("boost_after_preload_without_contact", False)
        )
        self.swing_duration_s = max(float(get_cfg("swing_duration_s", rospy.get_param("/swing_leg_controller/swing_duration_s", 0.65))), 0.1)
        self.default_normal_force_limit = float(get_cfg("default_normal_force_limit", rospy.get_param("/wall/max_normal_force_n", 150.0)))
        self.preload_normal_force_limit_n = float(
            get_cfg("preload_normal_force_limit_n", rospy.get_param("/swing_leg_controller/preload_normal_force_limit_n", 15.0))
        )
        self.attach_normal_force_limit_n = float(
            get_cfg("attach_normal_force_limit_n", rospy.get_param("/swing_leg_controller/attach_normal_force_limit_n", 25.0))
        )
        self.release_on_swing = bool(get_cfg("release_on_swing", True))
        self.contact_hold_min_s = max(
            float(get_cfg("contact_hold_min_s", rospy.get_param("/swing_leg_controller/contact_hold_min_s", 0.04))),
            0.0,
        )
        self.adhesion_force_reference_n = max(float(get_cfg("adhesion_force_reference_n", 35.0)), 1e-3)
        self.adhesion_rpm_min_scale = float(get_cfg("adhesion_rpm_min_scale", 0.85))
        self.adhesion_rpm_max_scale = max(float(get_cfg("adhesion_rpm_max_scale", 1.35)), self.adhesion_rpm_min_scale)
        self.leg_count = int(get_cfg("leg_count", 4))
        self.leg_names = [str(name) for name in get_cfg("leg_names", ["lf", "rf", "rr", "lr"])]

        self.state = self.STATE_INIT
        self.state_since = rospy.Time.now()
        self.last_state_pub = None
        self.last_estimated_state = EstimatedState()
        self.has_estimated_state = False
        self.jetson_safe_mode = False
        self.manual_pause = False
        self.start_requested = self.auto_start
        self.reset_fault_requested = False
        self.leg_support_state = {leg_name: True for leg_name in self.leg_names}
        self.leg_support_transition_time = {leg_name: rospy.Time.now() for leg_name in self.leg_names}
        self.leg_contact_active_since = {leg_name: None for leg_name in self.leg_names}
        self.leg_last_normal_force_limit = {leg_name: self.default_normal_force_limit for leg_name in self.leg_names}
        self.leg_required_adhesion_force = {leg_name: self.adhesion_force_reference_n for leg_name in self.leg_names}
        self.leg_last_skirt_compression_target = {leg_name: 0.0 for leg_name in self.leg_names}
        self.leg_index_map = dict((leg_name, index) for index, leg_name in enumerate(self.leg_names))

        self.heartbeat_pub = rospy.Publisher("/jetson/local_safety_supervisor/heartbeat", Bool, queue_size=10)
        self.mission_active_pub = rospy.Publisher("/control/mission_active", Bool, queue_size=10, latch=True)
        self.state_pub = rospy.Publisher("/control/mission_state", String, queue_size=10, latch=True)
        self.adhesion_pub = rospy.Publisher("/jetson/fan_serial_bridge/adhesion_command", AdhesionCommand, queue_size=20)

        rospy.Subscriber("/state/estimated", EstimatedState, self.estimated_state_callback, queue_size=20)
        rospy.Subscriber("/jetson/local_safety_supervisor/safe_mode", Bool, self.safe_mode_callback, queue_size=20)
        rospy.Subscriber("/control/mission_pause", Bool, self.pause_callback, queue_size=20)
        rospy.Subscriber("/control/mission_start", Bool, self.start_callback, queue_size=20)
        rospy.Subscriber("/control/mission_reset_fault", Bool, self.reset_fault_callback, queue_size=20)
        rospy.Subscriber("/control/swing_leg_target", LegCenterCommand, self.swing_target_callback, queue_size=50)
        for leg_name in self.leg_names:
            rospy.Subscriber(
                "/control/stance_wrench/" + leg_name,
                StanceWrenchCommand,
                self.stance_wrench_callback,
                callback_args=leg_name,
                queue_size=20,
            )

        self._publish_state(force=True)

    def estimated_state_callback(self, msg):
        self.last_estimated_state = msg
        self.has_estimated_state = True

    def safe_mode_callback(self, msg):
        self.jetson_safe_mode = bool(msg.data)

    def pause_callback(self, msg):
        self.manual_pause = bool(msg.data)

    def start_callback(self, msg):
        self.start_requested = bool(msg.data)

    def reset_fault_callback(self, msg):
        self.reset_fault_requested = bool(msg.data)

    def swing_target_callback(self, msg):
        if msg.leg_name not in self.leg_support_state:
            return
        next_support = bool(msg.support_leg)
        if self.leg_support_state[msg.leg_name] != next_support:
            self.leg_support_state[msg.leg_name] = next_support
            self.leg_support_transition_time[msg.leg_name] = rospy.Time.now()
        self.leg_last_normal_force_limit[msg.leg_name] = float(msg.desired_normal_force_limit) if msg.desired_normal_force_limit > 0.0 else self.default_normal_force_limit
        self.leg_last_skirt_compression_target[msg.leg_name] = float(msg.skirt_compression_target)

    def stance_wrench_callback(self, msg, leg_name):
        if leg_name not in self.leg_required_adhesion_force:
            return
        if msg.leg_name and msg.leg_name != leg_name:
            leg_name = msg.leg_name
            if leg_name not in self.leg_required_adhesion_force:
                return
        self.leg_last_normal_force_limit[leg_name] = float(msg.normal_force_limit) if msg.normal_force_limit > 0.0 else self.default_normal_force_limit
        self.leg_required_adhesion_force[leg_name] = max(float(msg.required_adhesion_force), 0.0)

    def _set_state(self, new_state):
        if new_state == self.state:
            return
        rospy.loginfo("mission_supervisor transition: %s -> %s", self.state, new_state)
        self.state = new_state
        self.state_since = rospy.Time.now()
        self._publish_state(force=True)

    def _state_elapsed(self):
        return (rospy.Time.now() - self.state_since).to_sec()

    def _adhesion_count(self):
        if not self.has_estimated_state:
            return 0
        return sum([1 for value in self.last_estimated_state.adhesion_mask if bool(value)])

    def _max_slip_risk(self):
        if not self.has_estimated_state or len(self.last_estimated_state.slip_risk) == 0:
            return 0.0
        return max([float(value) for value in self.last_estimated_state.slip_risk])

    def _adhesion_loss_detected(self):
        if not self.has_estimated_state:
            return False
        support_mask = list(self.last_estimated_state.support_mask)
        adhesion_mask = list(self.last_estimated_state.adhesion_mask)
        paired_count = min(len(support_mask), len(adhesion_mask))
        for index in range(paired_count):
            if bool(support_mask[index]) and not bool(adhesion_mask[index]):
                return True
        return False

    def _publish_state(self, force=False):
        if force or self.last_state_pub != self.state:
            self.state_pub.publish(String(data=self.state))
            self.last_state_pub = self.state
        mission_active = self.state == self.STATE_CLIMB
        self.mission_active_pub.publish(Bool(data=mission_active))

    def _publish_heartbeat(self):
        self.heartbeat_pub.publish(Bool(data=True))

    def _leg_support_elapsed(self, leg_name):
        return (rospy.Time.now() - self.leg_support_transition_time[leg_name]).to_sec()

    def _contact_hold_satisfied(self, leg_name, contact_active):
        now = rospy.Time.now()
        if contact_active:
            if self.leg_contact_active_since[leg_name] is None:
                self.leg_contact_active_since[leg_name] = now
            return (now - self.leg_contact_active_since[leg_name]).to_sec() >= self.contact_hold_min_s
        self.leg_contact_active_since[leg_name] = None
        return False

    def _scale_rpm_by_required_adhesion(self, base_rpm, leg_name):
        if base_rpm <= 0.0:
            return 0.0
        required_force = max(float(self.leg_required_adhesion_force.get(leg_name, self.adhesion_force_reference_n)), 0.0)
        scale = required_force / self.adhesion_force_reference_n
        scale = min(self.adhesion_rpm_max_scale, max(self.adhesion_rpm_min_scale, scale))
        return base_rpm * scale

    def _estimated_leg_mask(self, attr_name, leg_name, default=False):
        if not self.has_estimated_state:
            return bool(default)
        leg_index = self.leg_index_map.get(leg_name, -1)
        values = getattr(self.last_estimated_state, attr_name, [])
        if leg_index < 0 or leg_index >= len(values):
            return bool(default)
        return bool(values[leg_index])

    def _estimated_leg_fan_current(self, leg_name):
        if not self.has_estimated_state:
            return 0.0
        leg_index = self.leg_index_map.get(leg_name, -1)
        if leg_index < 0 or leg_index >= len(self.last_estimated_state.fan_current):
            return 0.0
        return float(self.last_estimated_state.fan_current[leg_index])

    def _leg_fan_target(self, leg_name):
        if self.state == self.STATE_INIT:
            return 0, self.release_rpm
        if self.state == self.STATE_STICK:
            return 1, self._scale_rpm_by_required_adhesion(self.attach_rpm, leg_name)
        if self.state == self.STATE_PAUSE:
            return 1, self._scale_rpm_by_required_adhesion(self.pause_rpm, leg_name)
        if self.state == self.STATE_FAULT:
            return 2, self._scale_rpm_by_required_adhesion(self.fault_rpm, leg_name)

        support_leg = bool(self.leg_support_state.get(leg_name, True))
        elapsed = self._leg_support_elapsed(leg_name)

        # Release fan during swing unconditionally
        if not support_leg and self.release_on_swing:
            return 0, self.swing_release_rpm

        if support_leg:
            if elapsed <= self.touchdown_boost_duration_s:
                return 2, self._scale_rpm_by_required_adhesion(self.touchdown_rpm, leg_name)
            return 1, self._scale_rpm_by_required_adhesion(self.climb_rpm, leg_name)

        attachment_ready = self._estimated_leg_mask("attachment_ready_mask", leg_name, False)
        adhesion_ready = self._estimated_leg_mask("adhesion_mask", leg_name, attachment_ready)
        measured_contact = self._estimated_leg_mask("measured_contact_mask", leg_name, False)
        early_contact = self._estimated_leg_mask("early_contact_mask", leg_name, False)
        wall_touch = self._estimated_leg_mask("wall_touch_mask", leg_name, measured_contact or early_contact)
        skirt_target = max(float(self.leg_last_skirt_compression_target.get(leg_name, 0.0)), 0.0)
        fan_current = self._estimated_leg_fan_current(leg_name)
        post_preload = float(self.leg_last_normal_force_limit.get(leg_name, 0.0)) >= (self.attach_normal_force_limit_n - 1e-3)
        contact_active = measured_contact or wall_touch
        contact_stable = self._contact_hold_satisfied(leg_name, contact_active and post_preload)

        if attachment_ready or adhesion_ready:
            return 1, self._scale_rpm_by_required_adhesion(self.climb_rpm, leg_name)
        if self.boost_after_preload_without_contact and post_preload:
            return 2, self._scale_rpm_by_required_adhesion(self.touchdown_rpm, leg_name)
        if contact_stable:
            return 2, self._scale_rpm_by_required_adhesion(self.touchdown_rpm, leg_name)
        if fan_current > 0.05:
            return 1, self._scale_rpm_by_required_adhesion(self.attach_rpm, leg_name)
        if not post_preload and skirt_target > 0.5 and elapsed >= max(self.pre_attach_time_s, 0.0):
            return 0, self.swing_release_rpm

        pre_attach_start = max(self.swing_duration_s - self.pre_attach_time_s, 0.0)
        if elapsed >= pre_attach_start:
            return 0, self.swing_release_rpm
        return 0, self.swing_release_rpm

    def _publish_adhesion_command(self):
        if not self.enable_auto_adhesion_commands:
            return
        stamp = rospy.Time.now()
        for leg_name in self.leg_names:
            leg_index = self.leg_index_map[leg_name]
            mode, target_rpm = self._leg_fan_target(leg_name)
            msg = AdhesionCommand()
            msg.header.stamp = stamp
            msg.leg_index = leg_index
            msg.mode = int(mode)
            msg.target_rpm = float(target_rpm)
            msg.normal_force_limit = float(self.leg_last_normal_force_limit.get(leg_name, self.default_normal_force_limit))
            msg.required_adhesion_force = float(self.leg_required_adhesion_force.get(leg_name, self.adhesion_force_reference_n))
            self.adhesion_pub.publish(msg)

    def _update_state_machine(self):
        if self.state == self.STATE_FAULT:
            if self.reset_fault_requested and not self.jetson_safe_mode:
                self.reset_fault_requested = False
                self._set_state(self.STATE_INIT)
            return

        if self.pause_on_safe_mode and self.jetson_safe_mode:
            self._set_state(self.STATE_FAULT)
            return

        if self.manual_pause:
            self._set_state(self.STATE_PAUSE)
            return

        if self.state == self.STATE_INIT:
            if self.start_requested and self._state_elapsed() >= self.init_duration_s:
                self._set_state(self.STATE_STICK)
            return

        if self.state == self.STATE_STICK:
            if self._adhesion_count() >= self.adhesion_required_count:
                self._set_state(self.STATE_CLIMB)
                return
            if self._state_elapsed() >= self.stick_timeout_s:
                self._set_state(self.STATE_FAULT)
            return

        if self.state == self.STATE_CLIMB:
            if self.pause_on_high_slip and self._max_slip_risk() >= self.high_slip_threshold:
                self._set_state(self.STATE_PAUSE)
                return
            if self.pause_on_adhesion_loss and self._adhesion_loss_detected():
                self._set_state(self.STATE_PAUSE)
            return

        if self.state == self.STATE_PAUSE:
            if self.pause_on_safe_mode and self.jetson_safe_mode:
                self._set_state(self.STATE_FAULT)
                return
            if not self.manual_pause and self.auto_resume and self._max_slip_risk() < self.high_slip_threshold:
                self._set_state(self.STATE_STICK)

    def _state_rpm(self):
        if self.state == self.STATE_INIT:
            return self.release_rpm
        if self.state == self.STATE_STICK:
            return self.attach_rpm
        if self.state == self.STATE_CLIMB:
            return self.climb_rpm
        if self.state == self.STATE_PAUSE:
            return self.pause_rpm
        return self.fault_rpm

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            self._update_state_machine()
            self._publish_heartbeat()
            self._publish_state()
            self._publish_adhesion_command()
            rate.sleep()


if __name__ == "__main__":
    node = MissionSupervisor()
    node.spin()