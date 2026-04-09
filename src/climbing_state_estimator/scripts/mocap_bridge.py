#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped


class MocapBridge(object):
    def __init__(self):
        rospy.init_node("mocap_bridge", anonymous=False)
        self.input_topic = rospy.get_param("~input_topic", "/mocap/pose")
        self.output_topic = rospy.get_param("~output_topic", "/sensing/mocap_pose")
        self.pub = rospy.Publisher(self.output_topic, PoseStamped, queue_size=20)
        rospy.Subscriber(self.input_topic, PoseStamped, self.callback, queue_size=20)

    def callback(self, msg):
        self.pub.publish(msg)


if __name__ == "__main__":
    MocapBridge()
    rospy.spin()