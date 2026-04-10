#!/usr/bin/env python
# -*- coding: utf-8 -*-

import io
import json
import os
import threading
import time
from collections import OrderedDict
from datetime import datetime

import rospy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu, JointState
from std_msgs.msg import Bool, Float32MultiArray, String

from climbing_msgs.msg import (
    AdhesionCommand,
    BodyReference,
    EstimatedState,
    LegCenterCommand,
    StanceWrenchCommand,
)

text_type = type(u'')


def message_to_dict(msg):
    if hasattr(msg, "__slots__") and hasattr(msg, "_slot_types"):
        data = OrderedDict()
        for slot in msg.__slots__:
            data[slot] = message_to_dict(getattr(msg, slot))
        return data
    if isinstance(msg, (list, tuple)):
        return [message_to_dict(item) for item in msg]
    if hasattr(msg, "to_sec"):
        return msg.to_sec()
    return msg


def point_to_dict(point):
    return {
        'x': point.x,
        'y': point.y,
        'z': point.z,
    }


def vector3_to_dict(vector):
    return {
        'x': vector.x,
        'y': vector.y,
        'z': vector.z,
    }


def quaternion_to_dict(quat):
    return {
        'x': quat.x,
        'y': quat.y,
        'z': quat.z,
        'w': quat.w,
    }


def pose_to_dict(pose):
    return {
        'position': point_to_dict(pose.position),
        'orientation': quaternion_to_dict(pose.orientation),
    }


def twist_to_dict(twist):
    return {
        'linear': vector3_to_dict(twist.linear),
        'angular': vector3_to_dict(twist.angular),
    }


def wrench_to_dict(wrench):
    return {
        'force': vector3_to_dict(wrench.force),
        'torque': vector3_to_dict(wrench.torque),
    }


class StateLogger(object):
    def __init__(self):
        rospy.init_node('state_logger', anonymous=False)

        self.output_dir = os.path.expanduser(rospy.get_param('~output_dir', '~/.ros/climbing_logs'))
        self.session_name = rospy.get_param('~session_name', 'climbing_session')
        self.snapshot_rate_hz = float(rospy.get_param('~snapshot_rate_hz', 10.0))
        self.flush_every_n = int(rospy.get_param('~flush_every_n_records', 10))
        self.leg_names = rospy.get_param('~leg_names', ['lf', 'rf', 'rr', 'lr'])
        self.compact_mode = bool(rospy.get_param('~compact_mode', True))
        self.record_events = bool(rospy.get_param('~record_events', True))
        self.record_snapshots = bool(rospy.get_param('~record_snapshots', True))
        self.event_topics = set(rospy.get_param('~event_topics', [
            '/state/estimated',
            '/control/body_reference',
            '/control/swing_leg_target',
            '/control/mission_state',
            '/control/mission_active',
            '/jetson/fan_serial_bridge/adhesion_command',
            '/jetson/fan_serial_bridge/fan_currents',
            '/jetson/fan_serial_bridge/leg_rpm',
            '/jetson/dynamixel_bridge/joint_currents',
            '/jetson/local_safety_supervisor/safe_mode',
        ]))
        self.snapshot_topics = set(rospy.get_param('~snapshot_topics', [
            '/state/estimated',
            '/control/body_reference',
            '/control/swing_leg_target',
            '/control/mission_state',
            '/control/mission_active',
            '/jetson/fan_serial_bridge/adhesion_command',
            '/jetson/fan_serial_bridge/fan_currents',
            '/jetson/fan_serial_bridge/leg_rpm',
            '/jetson/dynamixel_bridge/joint_state',
            '/jetson/dynamixel_bridge/joint_currents',
            '/jetson/imu_serial_bridge/imu',
            '/sensing/mocap_pose',
            '/jetson/local_safety_supervisor/safe_mode',
        ]))
        self.record_stance_wrench_events = bool(rospy.get_param('~record_stance_wrench_events', True))
        self.record_stance_wrench_snapshots = bool(rospy.get_param('~record_stance_wrench_snapshots', True))
        self.parameter_roots = rospy.get_param(
            '~parameter_roots',
            [
                '/robot',
                '/wall',
                '/gait_controller',
                '/body_planner',
                '/state_estimator',
                '/stance_optimizer',
                '/swing_leg_controller',
                '/mission_supervisor',
                '/state_logger',
            ],
        )

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(self.output_dir, '%s_%s' % (self.session_name, timestamp))
        if not os.path.exists(self.session_dir):
            os.makedirs(self.session_dir)

        self.events_file = None
        self.snapshots_file = None
        if self.record_events:
            self.events_file = io.open(os.path.join(self.session_dir, 'events.jsonl'), 'w', encoding='utf-8')
        if self.record_snapshots:
            self.snapshots_file = io.open(os.path.join(self.session_dir, 'snapshots.jsonl'), 'w', encoding='utf-8')
        self.record_count = 0
        self.file_lock = threading.Lock()
        self.cache_lock = threading.Lock()
        self.latest_msgs = {}
        self.subscribers = []

        self._write_params()
        self._setup_subscribers()

        self.timer = None
        if self.record_snapshots:
            timer_period = 1.0 / self.snapshot_rate_hz if self.snapshot_rate_hz > 0.0 else 0.1
            self.timer = rospy.Timer(rospy.Duration(timer_period), self._snapshot_callback)
        rospy.loginfo('State Logger initialized. Session: %s', self.session_dir)

    def _setup_subscribers(self):
        topic_specs = [
            ('/state/estimated', EstimatedState),
            ('/control/body_reference', BodyReference),
            ('/control/swing_leg_target', LegCenterCommand),
            ('/control/mission_state', String),
            ('/control/mission_active', Bool),
            ('/jetson/fan_serial_bridge/adhesion_command', AdhesionCommand),
            ('/jetson/fan_serial_bridge/fan_currents', Float32MultiArray),
            ('/jetson/fan_serial_bridge/leg_rpm', Float32MultiArray),
            ('/jetson/dynamixel_bridge/joint_state', JointState),
            ('/jetson/dynamixel_bridge/joint_currents', Float32MultiArray),
            ('/jetson/imu_serial_bridge/imu', Imu),
            ('/sensing/mocap_pose', PoseStamped),
            ('/jetson/local_safety_supervisor/safe_mode', Bool),
        ]

        for topic, msg_type in topic_specs:
            self.subscribers.append(rospy.Subscriber(topic, msg_type, self._topic_callback, callback_args=topic))

        for leg_name in self.leg_names:
            topic = '/control/stance_wrench/%s' % leg_name
            self.subscribers.append(
                rospy.Subscriber(topic, StanceWrenchCommand, self._topic_callback, callback_args=topic)
            )

    def _write_params(self):
        params = {}
        for root in self.parameter_roots:
            try:
                params[root] = rospy.get_param(root)
            except KeyError:
                rospy.logwarn('Parameter root %s not found', root)

        params_path = os.path.join(self.session_dir, 'params.json')
        with io.open(params_path, 'w', encoding='utf-8') as handle:
            handle.write(text_type(json.dumps(params, indent=2, ensure_ascii=False)))

    def _base_record(self):
        return {
            'wall_time': time.time(),
            'ros_time': rospy.get_time(),
        }

    def _should_record_event(self, topic):
        if topic.startswith('/control/stance_wrench/'):
            return self.record_stance_wrench_events
        return topic in self.event_topics

    def _should_cache_snapshot(self, topic):
        if topic.startswith('/control/stance_wrench/'):
            return self.record_stance_wrench_snapshots
        return topic in self.snapshot_topics

    def _serialize_message(self, topic, msg):
        if not self.compact_mode:
            return message_to_dict(msg)

        if topic == '/state/estimated':
            return {
                'pose': pose_to_dict(msg.pose),
                'twist': twist_to_dict(msg.twist),
                'skirt_compression_estimate': list(msg.skirt_compression_estimate),
                'normal_force_limit': list(msg.normal_force_limit),
                'slip_risk': list(msg.slip_risk),
                'contact_confidence': list(msg.contact_confidence),
                'fan_current': list(msg.fan_current),
                'plan_support_mask': list(msg.plan_support_mask),
                'measured_contact_mask': list(msg.measured_contact_mask),
                'wall_touch_mask': list(msg.wall_touch_mask),
                'compression_ready_mask': list(msg.compression_ready_mask),
                'preload_ready_mask': list(msg.preload_ready_mask),
                'early_contact_mask': list(msg.early_contact_mask),
                'contact_mask': list(msg.contact_mask),
                'support_mask': list(msg.support_mask),
                'adhesion_mask': list(msg.adhesion_mask),
                'attachment_ready_mask': list(msg.attachment_ready_mask),
                'seal_confidence': list(msg.seal_confidence),
                'leg_torque_sum': list(msg.leg_torque_sum),
                'leg_torque_contact_confidence': list(msg.leg_torque_contact_confidence),
                'recent_stable_foot_center_positions': [point_to_dict(p) for p in msg.recent_stable_foot_center_positions],
                'universal_joint_center_positions': [point_to_dict(p) for p in msg.universal_joint_center_positions],
            }

        if topic == '/control/body_reference':
            return {
                'pose': pose_to_dict(msg.pose),
                'twist': twist_to_dict(msg.twist),
                'gait_mode': msg.gait_mode,
                'support_mask': list(msg.support_mask),
            }

        if topic == '/control/swing_leg_target':
            return {
                'leg_name': msg.leg_name,
                'center': point_to_dict(msg.center),
                'center_velocity': vector3_to_dict(msg.center_velocity),
                'skirt_compression_target': msg.skirt_compression_target,
                'support_leg': msg.support_leg,
                'desired_normal_force_limit': msg.desired_normal_force_limit,
            }

        if topic == '/jetson/fan_serial_bridge/adhesion_command':
            return {
                'leg_index': msg.leg_index,
                'mode': msg.mode,
                'target_rpm': msg.target_rpm,
                'normal_force_limit': msg.normal_force_limit,
                'required_adhesion_force': msg.required_adhesion_force,
            }

        if topic == '/jetson/dynamixel_bridge/joint_state':
            return {
                'name': list(msg.name),
                'position': list(msg.position),
                'velocity': list(msg.velocity),
                'effort': list(msg.effort),
            }

        if topic == '/jetson/dynamixel_bridge/joint_currents':
            return {'data': list(msg.data)}

        if topic == '/jetson/fan_serial_bridge/fan_currents':
            return {'data': list(msg.data)}

        if topic == '/jetson/fan_serial_bridge/leg_rpm':
            return {'data': list(msg.data)}

        if topic == '/jetson/imu_serial_bridge/imu':
            return {
                'orientation': quaternion_to_dict(msg.orientation),
                'angular_velocity': vector3_to_dict(msg.angular_velocity),
                'linear_acceleration': vector3_to_dict(msg.linear_acceleration),
            }

        if topic == '/sensing/mocap_pose':
            return {'pose': pose_to_dict(msg.pose)}

        if topic == '/control/mission_state':
            return {'data': msg.data}

        if topic == '/control/mission_active':
            return {'data': msg.data}

        if topic == '/jetson/local_safety_supervisor/safe_mode':
            return {'data': msg.data}

        if topic.startswith('/control/stance_wrench/'):
            return {
                'leg_name': msg.leg_name,
                'wrench': wrench_to_dict(msg.wrench),
                'normal_force_limit': msg.normal_force_limit,
                'tangential_force_magnitude': msg.tangential_force_magnitude,
                'required_adhesion_force': msg.required_adhesion_force,
                'planned_support': msg.planned_support,
                'early_contact': msg.early_contact,
                'actual_contact': msg.actual_contact,
                'active': msg.active,
            }

        return message_to_dict(msg)

    def _topic_callback(self, msg, topic):
        serialized = self._serialize_message(topic, msg)

        if self.record_snapshots and self._should_cache_snapshot(topic):
            with self.cache_lock:
                self.latest_msgs[topic] = serialized

        if self.record_events and self._should_record_event(topic):
            record = self._base_record()
            record['topic'] = topic
            record['data'] = serialized
            self._log_record(self.events_file, record)

    def _snapshot_callback(self, _event):
        if not self.record_snapshots:
            return
        record = self._base_record()
        with self.cache_lock:
            record['data'] = dict(self.latest_msgs)
        self._log_record(self.snapshots_file, record)

    def _log_record(self, handle, record):
        if handle is None:
            return
        line = json.dumps(record, ensure_ascii=False) + u'\n'
        with self.file_lock:
            handle.write(text_type(line))
            self.record_count += 1
            if self.record_count % self.flush_every_n == 0:
                handle.flush()

    def cleanup(self):
        with self.file_lock:
            if self.events_file is not None:
                self.events_file.flush()
                self.events_file.close()
            if self.snapshots_file is not None:
                self.snapshots_file.flush()
                self.snapshots_file.close()


if __name__ == '__main__':
    try:
        LOGGER = StateLogger()
        rospy.on_shutdown(LOGGER.cleanup)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass