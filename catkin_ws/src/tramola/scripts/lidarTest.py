#!/usr/bin/env python
import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LidarToGrid:
    def __init__(self):
        rospy.init_node('lidar_to_grid')

        self.grid_size = 30
        self.max_range = 12.0  # max distance lidar can detect
        self.obstacle_max_distance = 2.0  # threshold for marking obstacles
        self.origin = self.obstacle_max_distance // 2 
        self.resolution = self.grid_size / self.obstacle_max_distance
        self.objects_in_distance = np.zeros((self.grid_size + 1, 2), dtype=bool)

        self.left_min_dist = self.max_range
        self.right_min_dist = self.max_range

        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback, queue_size=1)

    def callback(self, scan):
        self.objects_in_distance.fill(False)
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

            # Convert to grid indices
            i = int(x*self.resolution + self.origin)
            j = int(y*self.resolution + self.origin)

            # Get angle in degrees for direction-based logic
            ang_deg = (math.degrees(angle) + 360) % 360
            if 45 < ang_deg < 135:
                self.left_min_dist = min(self.left_min_dist, r)
            elif 225 < ang_deg < 315:
                self.right_min_dist = min(self.right_min_dist, r)

            if 0 <= i <= self.grid_size and 0 <= j <= self.grid_size:
                if 0 < x <= self.obstacle_max_distance / 2:
                    self.objects_in_distance[j, 1] = True
                if 0 < x <= self.obstacle_max_distance:
                    self.objects_in_distance[j, 0] = True

            angle += scan.angle_increment

        # Print results
        print("\033c")  # clear console
        self.print_obstacles()
        print('left_min: {:.2f}m   right_min: {:.2f}m'.format(self.left_min_dist, self.right_min_dist))
        print('Objects in distance (normalized):', self.find_places_on_screen(self.objects_in_distance[:, 1]))

    def find_places_on_screen(self, obstacles):
        average_indices = []
        start = None

        for i, val in enumerate(obstacles):
            if val:
                if start is None:
                    start = i
            else:
                if start is not None:
                    avg = (start + i - 1) / 2
                    average_indices.append(avg)
                    start = None

        if start is not None:
            avg = (start + len(obstacles) - 1) / 2
            average_indices.append(avg)

        return [(i + 1) / float(self.grid_size) for i in average_indices]

    def print_obstacles(self):
        for row in self.objects_in_distance:
            print("   ".join(
                "#" if point else "0"
                for point in row
            ))

if __name__ == '__main__':
    node = LidarToGrid()
    rospy.spin()
