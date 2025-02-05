#!/usr/bin/env python

import rospy

from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode, SetModeRequest


class Vehicle:
    def __init__(self):
        assert rospy.core.is_initialized(), "ROS node is not initialized"   
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        self.linear_speed = 0.0 # max 1.0
        self.angular_speed = 0.0 # max 1.0

        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_speed)  # 10 Hz

        rospy.wait_for_service("/mavros/set_mode")  # Servis hazır olana kadar bekle
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.current_mode = None


    def set_mode(self, mode):
        if self.current_mode == mode:
            return

        req = SetModeRequest()
        req.custom_mode = mode
        resp = self.mode_srv(req)
        if resp.mode_sent:
            self.current_mode = mode
            rospy.loginfo("Mode changed to %s successfully", mode)
        else:
            rospy.logwarn("Failed to change mode to %s", mode)
        

    def publish_speed(self, event):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.angular_speed
        self.velocity_pub.publish(cmd)

    def turn_left(self, angular_speed=0.5):
        self.angular_speed = angular_speed

    def turn_right(self, angular_speed=0.5):
        self.angular_speed = -angular_speed

    def go_straight(self, speed=0.2):
        self.linear_speed = speed

    def go_left(self, speed=0.2, angular_speed=0.5):
        self.linear_speed = speed
        self.angular_speed = angular_speed

    def go_right(self, speed=0.2, angular_speed=0.5):
        self.linear_speed = speed
        self.angular_speed = -angular_speed
        

    def stop(self):
        self.rc_msg.channels[self.steering_channel] = self.scale_pwm(0.5)  # Neutral steering
        self.rc_msg.channels[self.throttle_channel] = self.scale_pwm(0.5)  # Neutral throttle

    def __del__(self):
        self.timer.shutdown()
        self.stop()
        self.velocity_pub.unregister()