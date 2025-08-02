# -*- coding: utf-8 -*-
import time
from tramola.task import Task
import numpy as np

class Kamikaze(Task):
    def __init__(self, vehicle, lidar, detection, color_code, default_bearing):
        super(Kamikaze, self).__init__(vehicle, lidar, detection=detection)
        self.color_code = color_code
        self.detection.start()
        self.default_bearing = default_bearing
        self.last_detection = time.time() - 2
        self.state = "SEARCH"  # Initialize state
        self.max_angular_speed = 1.0  # Define max angular speed
        self.vehicle.start_velocity_publisher()


    def mission_callback(self, t):
        detected = None
        for detection in self.detection.detections:
            if detection.class_id == self.color_code:
                if detected is None or detected.confidence < detection.confidence:
                    detected = detection
        
        if detected:
            print("Target detected - Confidence: {:.2f}, Position: {:.2f}".format(detected.confidence, detected.x_center))

        if self.state == "SEARCH":
            if detected:
                print("Target LOCKED - Switching from SEARCH to LOCKED state")
                self.state = "LOCKED"
                self.last_detection = time.time()
                return
            
            print("Searching - Heading: {:.2f}° -> Target: {:.2f}°".format(self.vehicle.heading, self.default_bearing))
            self._move_with_heading(self.default_bearing)

        elif self.state == "LOCKED":
            if detected:
                angular_speed = -(detected.x_center - 0.5)
                print("Tracking target - Angular speed: {:.2f}, Linear speed: 0.4".format(angular_speed))
                self.vehicle.angular_speed = angular_speed
                self.vehicle.linear_speed = 0.4
                self.last_detection = time.time()
            else:
                # Start countdown to stop
                time_since_detection = time.time() - self.last_detection
                print("Target lost - Time since last detection: {:.2f}s".format(time_since_detection))
                
                if time_since_detection > 5:
                    print("Target lost for >3s - STOPPING")
                    self.stop()
                    self.detection.stop()
                

    
    def _move_with_heading(self, target_heading):
        """Move directly towards the target heading."""
        # Calculate the shortest angle to the target, which will be in the range [-180, 180]
        heading_error = (target_heading - self.vehicle.heading + 180) % 360 - 180

        # Normalize the angle error to get the angular speed and assign it to the vehicle
        self.vehicle.angular_speed = self._normalize_angular_speed(-heading_error)

        # Set a constant forward speed
        self.vehicle.linear_speed = 0.4

    def _normalize_angular_speed(self, angle):
        """Normalize angle to the proper angular speed range [-max_angular_speed, max_angular_speed]."""
        # Divide the angle by 180 to scale it to a range of [-1, 1],
        # then clip it to the maximum allowed angular speed.
        normalized_speed = np.clip(angle / 180.0, -self.max_angular_speed, self.max_angular_speed)
        return normalized_speed