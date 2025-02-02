#!/usr/bin/env python

import rospy
from mavros_msgs.msg import OverrideRCIn



class Vehicle:
    def __init__(self):
        rospy.init_node('usv_controller', anonymous=True)
        self.rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.rc_msg = OverrideRCIn()
        self.steering_channel = 0  # Adjust based on your RC configuration
        self.throttle_channel = 2  # Adjust based on your RC configuration
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_rc_override)  # 10 Hz

    def publish_rc_override(self, event):
        self.rc_override_pub.publish(self.rc_msg)

    def go_straight(self, throttle=1500):
        self.rc_msg.channels[self.steering_channel] = 1500  # Neutral steering
        self.rc_msg.channels[self.throttle_channel] = throttle

    def go_left(self, throttle=1500, steering=1300):
        self.rc_msg.channels[self.steering_channel] = steering  # Left steering
        self.rc_msg.channels[self.throttle_channel] = throttle

    def go_right(self, throttle=1500, steering=1700):
        self.rc_msg.channels[self.steering_channel] = steering  # Right steering
        self.rc_msg.channels[self.throttle_channel] = throttle

    def stop(self):
        self.rc_msg.channels[self.steering_channel] = 1500  # Neutral steering
        self.rc_msg.channels[self.throttle_channel] = 1500  # Neutral throttle

    def __del__(self):
        self.timer.shutdown()
        self.stop()