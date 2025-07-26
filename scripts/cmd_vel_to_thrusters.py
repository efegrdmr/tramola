#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32

def callback(msg):
    L = rospy.get_param('~wheel_separation', 1.2)
    v = msg.linear.x
    omega = msg.angular.z
    left = v - omega * L / 2
    right = v + omega * L / 2

    pub_l.publish(Float32(left))
    pub_r.publish(Float32(right))

if __name__ == '__main__':
    rospy.init_node('cmd_vel_to_thrusters')
    pub_l = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=1)
    pub_r = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=1)
    rospy.Subscriber('/cmd_vel', Twist, callback)
    rospy.spin()
