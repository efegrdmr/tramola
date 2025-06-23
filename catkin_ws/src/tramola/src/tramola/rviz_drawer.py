#!/usr/bin/env python

import rospy
import math
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA

class RvizDrawer:
    def __init__(self, frame_id='base_link', topic='visualization_marker'):
        self.pub = rospy.Publisher(topic, Marker, queue_size=10)
        self.frame_id = frame_id
        self.marker_id = 0  # Unique ID counter

    def _polar_to_cartesian(self, angle_deg, length):
        angle_rad = math.radians(angle_deg)
        x = length * math.cos(angle_rad)
        y = length * math.sin(angle_rad)
        return x, y

    def _make_color(self, r, g, b, a=1.0):
        color = ColorRGBA()
        color.r = r
        color.g = g
        color.b = b
        color.a = a
        return color

    def draw_point(self, angle_deg, length, size, color):
        x, y = self._polar_to_cartesian(angle_deg, length)
        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "points"
        marker.id = self.marker_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0

        marker.scale.x = size
        marker.scale.y = size
        marker.scale.z = size

        marker.color = self._make_color(*color)

        self.pub.publish(marker)
        self.marker_id += 1

    def draw_arrow(self, angle1_deg, len1, angle2_deg, len2, size, color):
        x1, y1 = self._polar_to_cartesian(angle1_deg, len1)
        x2, y2 = self._polar_to_cartesian(angle2_deg, len2)

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "arrows"
        marker.id = self.marker_id
        marker.type = Marker.ARROW
        marker.action = Marker.ADD

        marker.points = [Point(x1, y1, 0.0), Point(x2, y2, 0.0)]

        marker.scale.x = size       # Shaft diameter
        marker.scale.y = size * 2   # Head diameter
        marker.scale.z = size * 2   # Head length

        marker.color = self._make_color(*color)

        self.pub.publish(marker)
        self.marker_id += 1
        
    def draw_line(self, angle1_deg, len1, angle2_deg, len2, width, color):
        x1, y1 = self._polar_to_cartesian(angle1_deg, len1)
        x2, y2 = self._polar_to_cartesian(angle2_deg, len2)

        marker = Marker()
        marker.header.frame_id = self.frame_id
        marker.header.stamp = rospy.Time.now()
        marker.ns = "lines"
        marker.id = self.marker_id
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        marker.points = [Point(x1, y1, 0.0), Point(x2, y2, 0.0)]

        marker.scale.x = width  # Line width

        marker.color = self._make_color(*color)

        self.pub.publish(marker)
        self.marker_id += 1