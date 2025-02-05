#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode, SetModeRequest

set_mode_srv = None
current_mode = None

def go_straight(pub, speed=1.0):
    cmd = Twist()
    cmd.linear.x = speed
    pub.publish(cmd)

def turn_left(pub, angular_speed=0.5):
    cmd = Twist()
    cmd.angular.z = angular_speed
    pub.publish(cmd)

def turn_right(pub, angular_speed=0.5):
    cmd = Twist()
    cmd.angular.z = -angular_speed
    pub.publish(cmd)

def move_bit_left(pub, speed=0.2, angular_speed=0.5):
    cmd = Twist()
    cmd.linear.x = speed
    cmd.angular.z = angular_speed
    pub.publish(cmd)

def move_bit_right(pub, speed=0.2, angular_speed=0.5):
    cmd = Twist()
    cmd.linear.x = speed
    cmd.angular.z = -angular_speed
    pub.publish(cmd)

def set_mode(mode):
    global set_mode_srv, current_mode

    if current_mode == mode:
        return
    
    if set_mode_srv is None:
        rospy.wait_for_service("/mavros/set_mode")  # Servis hazır olana kadar bekle
        set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

    try:
        req = SetModeRequest()
        req.custom_mode = mode
        resp = set_mode_srv(req)
        if resp.mode_sent:
            current_mode = mode
            rospy.loginfo("Mode changed to %s successfully", mode)
        else:
            rospy.logwarn("Failed to change mode to %s", mode)
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def main():
    rospy.init_node('velocity_test', anonymous=True)
    
    velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)

    set_mode("GUIDED")

    go_straight(velocity_pub, 0.2)
    rospy.sleep()
    move_bit_left(velocity_pub, 0.2)
    rospy.sleep()

if __name__ == '__main__':
    main()
