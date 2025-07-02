import math
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty

import numpy as np


class Lidar:
    def __init__(self):    
        assert rospy.core.is_initialized(), "ROS node is not initialized"  
        self.grid_size = 30
        self.max_range = 12.0  # max distance lidar can detect
        self.obstacle_max_distance = 1.0  # threshold for marking obstacles
        self.origin = self.grid_size // 2  # Center of the grid (for y axis)
        self.resolution = self.grid_size / self.obstacle_max_distance
        self.max_scan_angle_from_front = math.degrees(math.acos(self.obstacle_max_distance / self.max_range))

        # Now a 1D array for the "front" (y axis at a certain x)
        self.objects_in_distance = np.zeros(self.grid_size + 1, dtype=int)
        self.angles = np.zeros(self.grid_size + 1, dtype=float)  # List to store angles of detected obstacles

        self.left_min_dist = self.max_range
        self.right_min_dist = self.max_range


        self.stop_service = rospy.Service('/lidar/stop', Empty, self.stop)
        self.start_service = rospy.Service('/lidar/start', Empty, self.start)
        self.scan = None


    def callback(self, scan):
        self.scan = scan
    
    def print_obstacles(self):
        # Print the 1D obstacle array as a line
        line = ""
        for val in self.objects_in_distance:
            line += "#" if val else "."
        print(line)

    def _find_free_angles(self, min_width=0.1):
        free_spaces = []  # (start_angle, end_angle)
        min_index_dist = int(min_width * self.resolution)
        start = None
        self.angles[0] = self.max_scan_angle_from_front
        self.angles[self.grid_size] = -self.max_scan_angle_from_front

        for i, val in enumerate(self.objects_in_distance):
            if val:
                if start is not None and i - start >= min_index_dist:
                    free_spaces.append((self.angles[start], self.angles[i]))
                start = i  # Start of next free space
            elif start is None:
                start = i  # Mark the start of a free space

        # Check for free space at the end
        if start is not None and self.grid_size + 1 - start >= min_index_dist:
            free_spaces.append((self.angles[start], self.angles[self.grid_size]))

        return free_spaces

    def get_detections(self):
        if self.scan is None:
            return None
        self.objects_in_distance.fill(0)
        self.angles.fill(0) # Reset angles list
        self.left_min_dist = self.max_range
        self.right_min_dist = self.max_range

        angle = self.scan.angle_min
        for r in self.scan.ranges:
            if not np.isfinite(r) or r < self.scan.range_min or r > self.scan.range_max:
                angle += self.scan.angle_increment
                continue

            # Cartesian coordinates (in meters)
            x = r * math.cos(angle)
            y = r * math.sin(angle)

            ang_deg = (math.degrees(angle) + 360) % 360

            # Convert y to 1D array index (centered)
            j = int(y * self.resolution + self.origin)
            if 0 <= j <= self.grid_size:
                # Only consider obstacles in front (x > 0)
                if 0 < x <= self.obstacle_max_distance:    
                    self.objects_in_distance[j] = 1
                    if ang_deg > 90:
                        self.angles[j] = ang_deg - 360
                    else:
                        self.angles[j] = ang_deg

            # For left/right min distances
            if 45 < ang_deg < 135:
                self.left_min_dist = min(self.left_min_dist, r)
            elif 225 < ang_deg < 315:
                self.right_min_dist = min(self.right_min_dist, r)

            angle += self.scan.angle_increment

        self.objects_in_distance = self.objects_in_distance[::-1]
        self.angles = self.angles[::-1]
        print('left_min: {:.2f}m   right_min: {:.2f}m'.format(self.left_min_dist, self.right_min_dist))
        angles = self._find_free_angles()
        print(angles)# Print the collected angles

        return (angles, self.left_min_dist, self.right_min_dist)
    

    def stop(self):
        self.stop_service()
        self.sub.unregister()
        self.sub = None
        print("Lidar stopped")
    
    def start(self):
        """
        Start the Lidar 
        """
        print("Lidar started")
        self.start_service()
        self.sub = rospy.Subscriber('/scan', LaserScan, self.callback, queue_size=1)
