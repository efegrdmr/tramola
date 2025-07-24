#!/usr/bin/env python

import rospy
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Odometry

class GpsOdomWrapper:
    def __init__(self):
        rospy.init_node('gps_odom_wrapper', anonymous=True)
        
        # Subscribers
        rospy.Subscriber('/fix', NavSatFix, self.gps_callback)
        rospy.Subscriber('/odometry/filtered', Odometry, self.odom_filtered_callback)
        rospy.Subscriber('/odometry/gps', Odometry, self.odom_gps_callback)
        
        # Publishers (if needed)
        self.pub_odom_filtered = rospy.Publisher('/wrapped/odometry/filtered', Odometry, queue_size=10)
        self.pub_odom_gps = rospy.Publisher('/wrapped/odometry/gps_corrected', Odometry, queue_size=10)

    def gps_callback(self, msg):
        rospy.loginfo_throttle(5, "GPS Fix: lat={}, lon={}, status={}".format(msg.latitude, msg.longitude, msg.status.status))

    def odom_filtered_callback(self, msg):
        rospy.loginfo_throttle(5, "Filtered Odom Pos: x={:.2f}, y={:.2f}".format(msg.pose.pose.position.x, msg.pose.pose.position.y))
        # Example republish with no changes (you can add frame corrections here)
        self.pub_odom_filtered.publish(msg)

    def odom_gps_callback(self, msg):
        rospy.loginfo_throttle(5, "GPS Odom Pos: x={:.2f}, y={:.2f}".format(msg.pose.pose.position.x, msg.pose.pose.position.y))
        self.pub_odom_gps.publish(msg)

    def spin(self):
        rospy.spin()

if __name__ == '__main__':
    node = GpsOdomWrapper()
    node.spin()
