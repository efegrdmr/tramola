from tramola.task import Task
import rospy
import numpy as np
class GoTo(Task):
    def __init__(self, vehicle, lidar, move_base, point):
        super(GoTo, self).__init__(vehicle, lidar, detection=None)
        self.point = point
        self.vehicle.set_mode("GUIDED")
        self.move_base = move_base
        self.max_angular_speed =2.0
        angle = self.vehicle.angle_between(*point)
        self.move_base.send_goal(point[0], point[1], angle)
        self.state = "MOVE_BASE_CONTROL"

    
    def mission_callback(self, t):
        if (not self._distance_less_than(2)) and self.state == "MOVE_BASE_CONTROL":
            return
        
        if self.state == "MOVE_BASE_CONTROL":
            rospy.loginfo("Move_base stopped")
            self.move_base.cancel_goal()
            self.state = "STRAIGHT_TO_POINT"
            self.vehicle.start_velocity_publisher()
        
        if self._distance_less_than(1.0):
            self.stop()
            rospy.loginfo("reached to the point")
            return

        target_angle = self.vehicle.angle_between(*self.point)
        if self._target_out_of_angular_range(target_angle, 10):
            self._turn_towards_target(target_angle)
        else:
            self._move_to_target(target_angle)
            

    def _distance_less_than(self, distance):
        if self.vehicle.reached(*self.point, threshold=distance):
            return True
        return False


    def _normalize_angular_speed(self, angle):
        """Normalize angle to proper angular speed range [-1, 1]."""
        # Clamp to reasonable range and normalize
        normalized = np.clip(angle / 90.0, -self.max_angular_speed, self.max_angular_speed)
        return normalized

    def _move_to_target(self, target_angle):
        """Move directly towards the target."""
        self.vehicle.angular_speed = self._normalize_angular_speed(target_angle)
        self.vehicle.linear_speed = 0.4
        rospy.logwarn("Moving to target: angle=%.2f speed=%.2f", target_angle, self.vehicle.linear_speed)
    
    def _turn_towards_target(self, target_angle):
        """Turn in place towards the target."""
        self.vehicle.turn_inplace(self._normalize_angular_speed(target_angle))
        rospy.logwarn("Turning towards target: %.2f degrees", self._normalize_angular_speed(target_angle))
    
    def _target_out_of_angular_range(self, target_angle, max_angular_dist_to_point):
        """Check if target is outside lidar's field of view."""
        rospy.logwarn("target %.2f  max angular dist %.2f  is target out of angular range: %d", target_angle, max_angular_dist_to_point, abs(target_angle) > max_angular_dist_to_point)
        return abs(target_angle) > max_angular_dist_to_point