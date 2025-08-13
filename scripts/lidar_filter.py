#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math
import numpy as np

class ScanAngleFilter:
    def __init__(self):
        rospy.init_node("scan_angle_filter")

        # Parameters
        self.input_topic = rospy.get_param("~input_topic", "/scan")
        self.output_topic = rospy.get_param("~output_topic", "/scan_filtered")
        # angles in degrees you want to keep, can be multiple ranges
        self.keep_ranges = rospy.get_param("~keep_ranges", [[-60, -30], [30, 60]])

        # Convert degrees to radians
        self.keep_ranges = [[math.radians(r[0]), math.radians(r[1])] for r in self.keep_ranges]

        # Subscriber & Publisher
        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=1)
        rospy.Subscriber(self.input_topic, LaserScan, self.scan_callback)
        rospy.loginfo("Scan angle filter node started.")

    def scan_callback(self, msg):
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max

        # Mask ranges outside keep_ranges
        filtered_ranges = []
        angle = msg.angle_min
        for r in msg.ranges:
            keep = any(start <= angle <= end for start, end in self.keep_ranges)
            filtered_ranges.append(r if keep else float('inf'))
            angle += msg.angle_increment

        filtered.ranges = filtered_ranges
        filtered.intensities = msg.intensities  # optional, keep same

        self.pub.publish(filtered)

if __name__ == "__main__":
    try:
        node = ScanAngleFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
