#!/usr/bin/env python

import rospy
from std_msgs.msg import Bool


class LocalSafetySupervisor(object):
    def __init__(self):
        rospy.init_node("local_safety_supervisor", anonymous=False)
        self.timeout_sec = float(rospy.get_param("~control_timeout_sec", 1.0))
        self.last_heartbeat = rospy.Time(0)
        self.safe_pub = rospy.Publisher("~safe_mode", Bool, queue_size=5, latch=True)
        rospy.Subscriber("~heartbeat", Bool, self.heartbeat_callback, queue_size=20)
        rospy.Timer(rospy.Duration(0.1), self.check_timeout)

    def heartbeat_callback(self, _msg):
        self.last_heartbeat = rospy.Time.now()
        self.safe_pub.publish(Bool(data=False))

    def check_timeout(self, _event):
        if self.last_heartbeat == rospy.Time(0):
            return
        if (rospy.Time.now() - self.last_heartbeat).to_sec() > self.timeout_sec:
            self.safe_pub.publish(Bool(data=True))


if __name__ == "__main__":
    LocalSafetySupervisor()
    rospy.spin()