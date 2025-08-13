#!/usr/bin/env python
import rospy
from sensor_msgs.msg import LaserScan
import math

class ScanAngleFilter:
    def __init__(self):
        rospy.init_node("scan_angle_filter")
        # Parameters
        self.input_topic = rospy.get_param("~input_topic", "/scan")
        self.output_topic = rospy.get_param("~output_topic", "/scan_filtered")
        # angles (deg) you want to keep (list of [start_deg, end_deg])
        self.keep_ranges = rospy.get_param("~keep_ranges", [[-90.0, 90.0]])
        # value to place for filtered out beams (Inf so downstream planners ignore)
        self.replacement_value = rospy.get_param("~replacement_value", float('inf'))
        # Whether to normalise scan angles to [-pi, pi] before comparison (recommended)
        self.normalize = rospy.get_param("~normalize", True)
        # Convert degrees to radians once
        self.keep_ranges = [[math.radians(r[0]), math.radians(r[1])] for r in self.keep_ranges]
        # Validate ranges
        valid = []
        for a, b in self.keep_ranges:
            if a > b:
                rospy.logwarn("keep_range start > end (%.3f > %.3f rad). Swapping." % (a, b))
                a, b = b, a
            valid.append([a, b])
        self.keep_ranges = valid
        # Subscriber & Publisher
        self.pub = rospy.Publisher(self.output_topic, LaserScan, queue_size=1)
        rospy.Subscriber(self.input_topic, LaserScan, self.scan_callback, queue_size=1)
        rospy.loginfo("Scan angle filter node started. keeping ranges (rad): %s" % str(self.keep_ranges))

    def _norm_angle(self, ang):
        if not self.normalize:
            return ang
        # Fast normalize to [-pi, pi]
        return math.atan2(math.sin(ang), math.cos(ang))

    def scan_callback(self, msg):
        # Prepare output message (same layout; we just mask unwanted beams)
        filtered = LaserScan()
        filtered.header = msg.header
        filtered.angle_min = msg.angle_min
        filtered.angle_max = msg.angle_max
        filtered.angle_increment = msg.angle_increment
        filtered.time_increment = msg.time_increment
        filtered.scan_time = msg.scan_time
        filtered.range_min = msg.range_min
        filtered.range_max = msg.range_max
        # Build filtered ranges
        angle = msg.angle_min
        keep_ranges = self.keep_ranges  # local ref
        inc = msg.angle_increment
        replacement = self.replacement_value
        filtered_ranges = []
        # Ensure intensities length match after masking
        has_intensity = len(msg.intensities) == len(msg.ranges)
        filtered_intensities = [] if has_intensity else []
        for i, r in enumerate(msg.ranges):
            test_angle = self._norm_angle(angle)
            keep = any(start <= test_angle <= end for start, end in keep_ranges)
            if keep:
                filtered_ranges.append(r)
                if has_intensity:
                    filtered_intensities.append(msg.intensities[i])
            else:
                filtered_ranges.append(replacement)
                if has_intensity:
                    filtered_intensities.append(0.0)  # zero intensity for removed beam
            angle += inc
        filtered.ranges = filtered_ranges
        if has_intensity:
            filtered.intensities = filtered_intensities
        self.pub.publish(filtered)

if __name__ == "__main__":
    try:
        node = ScanAngleFilter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
