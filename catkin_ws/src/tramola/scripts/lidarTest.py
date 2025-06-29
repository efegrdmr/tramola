#!/usr/bin/env python
# -*- coding: utf-8 -*-

import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LidarToGrid:
    def __init__(self):
        rospy.init_node('lidar_to_grid')

        self.grid_size = 100
        self.max_range = 12.0  # max distance lidar can detect
        self.obstacle_max_distance = 1.0  # threshold for marking obstacles
        self.origin = self.grid_size // 2  # Center of the grid (for y axis)
        self.resolution = self.grid_size / self.obstacle_max_distance

        # Now a 1D array for the "front" (y axis at a certain x)
        self.objects_in_distance = np.zeros(self.grid_size + 1, dtype=int)

        self.left_min_dist = self.max_range
        self.right_min_dist = self.max_range

        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback, queue_size=1)

    def callback(self, scan):
        self.objects_in_distance.fill(0)
        self.left_min_dist = self.max_range
        self.right_min_dist = self.max_range

        angle = scan.angle_min
        for r in scan.ranges:
            if not np.isfinite(r) or r < scan.range_min or r > scan.range_max:
                angle += scan.angle_increment
                continue

            # Cartesian coordinates (in meters)
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            # Only consider obstacles in front (x > 0)
            if 0 < x <= self.obstacle_max_distance:
                # Convert y to 1D array index (centered)
                j = int(y * self.resolution + self.origin)
                if 0 <= j <= self.grid_size:
                    self.objects_in_distance[j] = 1

            # For left/right min distances
            ang_deg = (math.degrees(angle) + 360) % 360
            if 45 < ang_deg < 135:
                self.left_min_dist = min(self.left_min_dist, r)
            elif 225 < ang_deg < 315:
                self.right_min_dist = min(self.right_min_dist, r)

            angle += scan.angle_increment

        self.objects_in_distance = self.objects_in_distance[::-1]
        print("\033c")  # clear console
        self.print_obstacles()
        print('left_min: {:.2f}m   right_min: {:.2f}m'.format(self.left_min_dist, self.right_min_dist))
        print('Objects in distance (normalized):', self.find_places_on_screen(self.objects_in_distance))

    def find_places_on_screen(self, obstacles):
        # Return normalized positions of detected obstacles
        return [(i + 1) / float(self.grid_size) for i, val in enumerate(obstacles) if val]

    def print_obstacles(self):
        # Print the 1D obstacle array as a line
        line = ""
        for val in self.objects_in_distance:
            line += "#" if val else "."
        print(line)

if __name__ == '__main__':
    node = LidarToGrid()
    rospy.spin()