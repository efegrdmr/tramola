#!/usr/bin/env python
import math
import rospy
import numpy as np
from sensor_msgs.msg import LaserScan

class LidarToGrid:
    def __init__(self):
        rospy.init_node('lidar_to_grid')

        # Parameters for grid
        self.grid_size = 100  # original size (n)
        self.grid = np.zeros((self.grid_size + 1, self.grid_size + 1), dtype=np.int8)  # size = (n+1)x(n+1)
        self.resolution = 0.1  # each cell = 0.1 meters
        
        # constants
        self.obstacle_max_distance = 2.0  # max distance for obstacles in meters

        # Origin in the grid (center)
        self.origin_x = (self.grid_size + 1) // 2
        self.origin_y = (self.grid_size + 1) // 2
        
        self.left_min_dist = 30.0
        self.right_min_dist = 30.0

        # index 0 objects less than half max distance, index 1 less then half distance
        self.objects_in_distance = np.zeros((self.grid_size, 2), dtype=bool) 

        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback, queue_size=1)

    def callback(self, scan):
        # Clear grid each scan
        self.grid.fill(0)
        self.objects_in_distance.fill(0)
        # Fill borders with 2
        self.grid[0, :] = 2
        self.grid[-1, :] = 2
        self.grid[:, 0] = 2
        self.grid[:, -1] = 2

        self.left_min_dist = 30.0
        self.right_min_dist = 30.0


        angle = scan.angle_min
        for r in scan.ranges:
            if np.isfinite(r) and scan.range_min < r < scan.range_max:
                x = r * np.cos(angle)
                y = r * np.sin(angle)
                # if an obstacle add to grid
                if x < self.obstacle_max_distance:
                    i = int(self.origin_x + x / self.resolution)
                    j = int(self.origin_y + y / self.resolution)
                    self.objects_in_distance[i, 1] = 1

                    if 0 <= i < self.grid.shape[1] and 0 <= j < self.grid.shape[0]:
                        self.grid[j, i] = 1  # Mark occupied cell
                
                if x < self.obstacle_max_distance / 2:
                    i = int(self.origin_x + x / self.resolution)
                    self.objects_in_distance[i, 0] = 1

                ang_deg = (math.degrees(angle) + 360) % 360
                if 45 < ang_deg  < 135:
                    self.left_min_dist = min(self.left_min_dist, r)
                elif 225 < ang_deg < 315:
                    self.right_min_dist = min(self.right_min_dist, r)

            angle += scan.angle_increment
        
        self.grid[self.origin_y:(self.origin_y + 2), self.origin_x:(self.origin_x + 2)] = 3

        front = self.grid[:, (self.origin_y + 1):]


        print("\033c")  # Clear terminal
        #self.print_grid(front)
        print('left_min: {} right_min: {}'.format(self.left_min_dist, self.right_min_dist))
        print('Objects in distance: ' + self.obstacle_max_distance.__str__() + " " + self.find_places_on_screen(self.objects_in_distance[:, 1]).__str__())
        #self.print_obstacles()
        rospy.sleep(0.5)

    def find_places_on_screen(self, obstacles):
        average_indices = []
        start = None

        for i, val in enumerate(obstacles):
            if val:
                if start is None:
                    start = i
            elif start is not None:
                avg = (start + i - 1) / 2
                average_indices.append(avg)
                start = None

        # In case the last value was part of a cluster
        if start is not None:
            avg = (start + len(obstacles) - 1) / 2
            average_indices.append(avg)

        # Normalize
        return [(i + 1) / float(self.grid_size) for i in average_indices]


    def print_obstacles(self):
        for obstacles in self.objects_in_distance:
            print("   ".join(
                "#" if point == 1 else
                "0" 
                for point in obstacles
            ))
                            

    def print_grid(self, grid):
        for row in grid:
            line = ''.join(
                '##' if cell == 1 else
                '..' if cell == 2 else
                '00' if cell == 3 else
                '  '
                for cell in row
            )
            print(line)
        print("\n" + "="*(2 * grid.shape[1]))


if __name__ == '__main__':
    node = LidarToGrid()
    rospy.spin()
