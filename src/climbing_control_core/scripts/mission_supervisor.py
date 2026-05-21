#!/usr/bin/env python

import rospy
from climbing_msgs.msg import AdhesionCommand, EstimatedState, LegCenterCommand, StanceWrenchCommand
from std_msgs.msg import Bool, String


class MissionSupervisor(object):
    STATE_INIT = "INIT"
    STATE_STICK = "STICK"
    STATE_CLIMB = "CLIMB"

    def __init__(self):
        rospy.init_node("mission_supervisor", anonymous=False)

        def get_cfg(name, default):
            return rospy.get_param("~" + name, rospy.get_param("/mission_supervisor/" + name, default))

        self.rate_hz = float(get_cfg("publish_rate_hz", 20.0))
        self.auto_start = bool(get_cfg("auto_start", True))
        self.enable_auto_adhesion_commands = bool(get_cfg("enable_auto_adhesion_commands", True))
        self.init_duration_s = max(float(get_cfg("init_duration_s", 1.0)), 0.0)
        self.adhesion_required_count = int(get_cfg("adhesion_required_count", 4))
        self.attach_rpm = float(get_cfg("attach_rpm", 3200.0))
        self.climb_rpm = float(get_cfg("climb_rpm", 3200.0))
        self.release_rpm = float(get_cfg("release_rpm", 0.0))
        self.swing_release_rpm = float(get_cfg("swing_release_rpm", 0.0))
        self.touchdown_rpm = float(get_cfg("touchdown_rpm", 3400.0))
        self.touchdown_boost_duration_s = max(float(get_cfg("touchdown_boost_duration_s", 0.25)), 0.0)
        self.default_normal_force_limit = float(get_cfg("default_normal_force_limit", rospy.get_param("/wall/max_normal_force_n", 150.0)))
        self.attach_normal_force_limit_n = float(
            get_cfg("attach_normal_force_limit_n", rospy.get_param("/swing_leg_controller/attach_normal_force_limit_n", 25.0)))
        self.adhesion_force_reference_n = max(float(get_cfg("adhesion_force_reference_n", 35.0)), 1e-3)
        self.adhesion_rpm_min_scale = float(get_cfg("adhesion_rpm_min_scale", 0.85))
        self.adhesion_rpm_max_scale = max(float(get_cfg("adhesion_rpm_max_scale", 1.35)), self.adhesion_rpm_min_scale)
        self.leg_names = [str(name) for name in get_cfg("leg_names", ["lf", "rf", "rr", "lr"])]

        self.state = self.STATE_INIT
        self.state_since = rospy.Time.now()
        self.last_state_pub = None
        self.last_estimated_state = EstimatedState()
        self.has_estimated_state = False
        self.manual_pause = False
        self.start_requested = self.auto_start
        self.leg_support_state = {leg_name: True for leg_name in self.leg_names}
        self.leg_support_transition_time = {leg_name: rospy.Time.now() for leg_name in self.leg_names}
        self.leg_last_normal_force_limit = {leg_name: self.default_normal_force_limit for leg_name in self.leg_names}
        self.leg_required_adhesion_force = {leg_name: self.adhesion_force_reference_n for leg_name in self.leg_names}
        self.leg_index_map = dict((leg_name, index) for index, leg_name in enumerate(self.leg_names))

        self.heartbeat_pub = rospy.Publisher("/jetson/local_safety_supervisor/heartbeat", Bool, queue_size=10)
        self.mission_active_pub = rospy.Publisher("/control/mission_active", Bool, queue_size=10, latch=True)
        self.state_pub = rospy.Publisher("/control/mission_state", String, queue_size=10, latch=True)
        self.adhesion_pub = rospy.Publisher("/jetson/fan_serial_bridge/adhesion_command", AdhesionCommand, queue_size=20)

        rospy.Subscriber("/state/estimated", EstimatedState, self.estimated_state_callback, queue_size=20)
        rospy.Subscriber("/control/mission_pause", Bool, self.pause_callback, queue_size=20)
        rospy.Subscriber("/control/mission_start", Bool, self.start_callback, queue_size=20)
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

    def pause_callback(self, msg):
        self.manual_pause = bool(msg.data)

    def start_callback(self, msg):
        self.start_requested = bool(msg.data)

    def swing_target_callback(self, msg):
        if msg.leg_name not in self.leg_support_state:
            return
        next_support = bool(msg.support_leg)
        if self.leg_support_state[msg.leg_name] != next_support:
            self.leg_support_state[msg.leg_name] = next_support
            self.leg_support_transition_time[msg.leg_name] = rospy.Time.now()
        self.leg_last_normal_force_limit[msg.leg_name] = float(msg.desired_normal_force_limit) if msg.desired_normal_force_limit > 0.0 else self.default_normal_force_limit

    def stance_wrench_callback(self, msg, leg_name):
        if leg_name not in self.leg_required_adhesion_force:
            return
        if msg.leg_name and msg.leg_name != leg_name:
            leg_name = msg.leg_name
            if leg_name not in self.leg_required_adhesion_force:
                return
        
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
        return sum([1 for value in self.last_estimated_state.attachment_ready_mask if bool(value)])

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

    def _scale_rpm_by_required_adhesion(self, base_rpm, leg_name):
        if base_rpm <= 0.0:
            return 0.0
        required_force = max(float(self.leg_required_adhesion_force.get(leg_name, self.adhesion_force_reference_n)), 0.0)
        scale = required_force / self.adhesion_force_reference_n
        scale = min(self.adhesion_rpm_max_scale, max(self.adhesion_rpm_min_scale, scale))
        return base_rpm * scale

    def _leg_fan_target(self, leg_name):
        if self.state == self.STATE_INIT:
            return 0, self.release_rpm
        if self.state == self.STATE_STICK:
            return 1, self._scale_rpm_by_required_adhesion(self.attach_rpm, leg_name)

        # CLIMB
        support_leg = bool(self.leg_support_state.get(leg_name, True))
        if support_leg:
            elapsed = self._leg_support_elapsed(leg_name)
            if elapsed <= self.touchdown_boost_duration_s:
                return 2, self._scale_rpm_by_required_adhesion(self.touchdown_rpm, leg_name)
            return 1, self._scale_rpm_by_required_adhesion(self.climb_rpm, leg_name)

        # Swing leg: fan on only when preload done (detected via normal_force_limit)
        if self.leg_last_normal_force_limit.get(leg_name, 0.0) >= (self.attach_normal_force_limit_n - 1e-3):
            return 1, self._scale_rpm_by_required_adhesion(self.climb_rpm, leg_name)
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
        if self.manual_pause:
            self.manual_pause = False
            self._set_state(self.STATE_STICK)
            return

        if self.state == self.STATE_INIT:
            if self.start_requested and self._state_elapsed() >= self.init_duration_s:
                self._set_state(self.STATE_STICK)
            return

        if self.state == self.STATE_STICK:
            if self._adhesion_count() >= self.adhesion_required_count:
                self._set_state(self.STATE_CLIMB)
            return

        if self.state == self.STATE_CLIMB:
            return

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
