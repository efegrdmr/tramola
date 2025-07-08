from tramola.task import Task
import rospy
import numpy as np

class GoTo(Task):
    def __init__(self, vehicle, lidar, point, linear_speed=0.5, max_angular_speed=1.0, completion_threshold=1.0, safety_padding=5.0):
        super(GoTo, self).__init__(vehicle, lidar, detection=None)
        self.point = point
        self.linear_speed = linear_speed
        self.max_angular_speed = max_angular_speed
        self.completion_threshold = completion_threshold
        self.safety_padding = safety_padding  # degrees of padding for free angle edges
        self.max_angular_dist_to_point = 45
        self.lidar.start()
        self.vehicle.start_velocity_publisher()
    


    def mission_callback(self, t):
        print("\033c")  # clear console
        """Main mission control loop."""
        if self._check_completion():
            self.lidar.stop()
            self.vehicle.stop_velocity_publisher()
            return
            
        detections = self.lidar.get_detections()
        if not detections:
            rospy.logwarn("No lidar data available")
            return
            
        free_angles, left_dist, right_dist = detections
        
        # Emergency stop if too close to obstacles
        if left_dist < 0.3 or right_dist < 0.3:
            self.vehicle.linear_speed = 0
            self.vehicle.angular_speed = 0
            rospy.logwarn("Emergency stop: obstacle too close")
            return
            
        target_angle = self.vehicle.angle_between(*self.point)
        
        if self._target_out_of_angular_range(target_angle):
            rospy.logwarn("Target range {} is out of angular range {}".format(target_angle, self.max_angular_dist_to_point))
            self._turn_towards_target(target_angle)
            return
            
        if self._target_in_free_space(target_angle, free_angles):
            rospy.logwarn("Target angle is in free space")
            self._move_to_target(target_angle)
        else:
            rospy.logwarn("Target angle is not in free space avoiding obstacles")
            self._avoid_obstacles(target_angle, free_angles)
        
        self.vehicle.linear_speed = self.linear_speed

    def _check_completion(self):
        """Check if mission is complete."""
        if self.vehicle.reached(*self.point, threshold=self.completion_threshold):
            self.state = "COMPLETED"
            self.vehicle.linear_speed = 0
            self.vehicle.angular_speed = 0
            self.stop()
            rospy.loginfo("GoTo mission completed")
            self.lidar.stop()
            return True
        return False


    def _target_out_of_angular_range(self, target_angle):
        """Check if target is outside lidar's field of view."""
        return abs(target_angle) > self.max_angular_dist_to_point
        

    def _turn_towards_target(self, target_angle):
        """Turn in place towards the target."""
        self.vehicle.turn_inplace(self._normalize_angular_speed(target_angle))
        rospy.logwarn("Turning towards target: %.2f degrees", self._normalize_angular_speed(target_angle))
  

    def _target_in_free_space(self, target_angle, free_angles):
        """Check if target direction is within any free angle range with safety padding."""
        if len(free_angles) == 0:
            return True  # No obstacles detected, go directly
            
        for start_angle, end_angle in free_angles:
            # Apply safety padding to avoid edges of free spaces
            if start_angle > 0:
                padded_start = start_angle - self.safety_padding
            else:
                padded_start = start_angle + self.safety_padding

            if end_angle > 0:
                padded_end = end_angle + self.safety_padding
            else:
                padded_end = end_angle - self.safety_padding
            
            # Only consider this free space if it's wide enough after padding
            if padded_end > padded_start and padded_start <= target_angle <= padded_end:
                return True
        return False

    def _move_to_target(self, target_angle):
        """Move directly towards the target."""
        self.vehicle.angular_speed = self._normalize_angular_speed(target_angle)
        rospy.logwarn("Moving to target: angle=%.2f", target_angle)

    def _avoid_obstacles(self, target_angle, free_angles):
        """Navigate around obstacles towards the target."""
        best_angle = self._find_best_angle(free_angles, target_angle)
        self.vehicle.angular_speed = self._normalize_angular_speed(best_angle)
        rospy.logwarn("Avoiding obstacles: target=%.2f, chosen=%.2f", target_angle, best_angle)

    def _find_best_angle(self, free_angles, target_angle):
        """Find the best angle considering both direction to target and safety."""
        if not free_angles:
            return target_angle
            
        best_angle = None
        best_score = float('inf')
        
        for start_angle, end_angle in free_angles:     
            width = end_angle - start_angle
            
            # Sample multiple points across the padded free space
            for offset in [0.2, 0.5, 0.8]:
                angle = start_angle + offset * width
                angle_diff = abs(angle - target_angle)
                
                # Score combines proximity to target and passage width
                score = angle_diff
                
                if score < best_score:
                    best_score = score
                    best_angle = angle
        
        return best_angle if best_angle is not None else target_angle

    def _normalize_angular_speed(self, angle):
        """Normalize angle to proper angular speed range [-1, 1]."""
        # Clamp to reasonable range and normalize
        normalized = np.clip(angle / 180.0, -self.max_angular_speed, self.max_angular_speed)
        return normalized