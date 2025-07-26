#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan

class LaserScanScaler:
    def __init__(self):
        # Get scale factor (default = 0.5)
        self.scale_factor = rospy.get_param('~scale_factor', 0.5)

        # Subscribe to original scan
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_callback, queue_size=10)

        # Publish scaled scan
        self.scan_pub = rospy.Publisher('/scan_scaled', LaserScan, queue_size=10)

        rospy.loginfo("LaserScanScaler initialized with scale factor: %.2f", self.scale_factor)

    def scan_callback(self, msg):
        scaled = LaserScan()
        scaled.header = msg.header
        scaled.angle_min = msg.angle_min
        scaled.angle_max = msg.angle_max
        scaled.angle_increment = msg.angle_increment
        scaled.time_increment = msg.time_increment
        scaled.scan_time = msg.scan_time
        scaled.range_min = msg.range_min * self.scale_factor
        scaled.range_max = msg.range_max * self.scale_factor

        # Scale each range value
        scaled.ranges = [r * self.scale_factor if r < float('inf') else r for r in msg.ranges]
        scaled.intensities = msg.intensities

        self.scan_pub.publish(scaled)

if __name__ == '__main__':
    rospy.init_node('laser_scan_scaler')
    try:
        scaler = LaserScanScaler()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
