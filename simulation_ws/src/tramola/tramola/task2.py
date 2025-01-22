import rclpy
from rclpy.node import Node
from tramolaa.msg import DetectionList, Detection
from tramola.thruster import Thrusters
import math

class DetectionListenerNode(Node):
    def __init__(self):
        super().__init__('detection_listener')
        
        self.subscription = self.create_subscription(
            DetectionList,
            'yolo_detections',
            self.detection_callback,
            10
        )
        self.thrusters = Thrusters(self)
        
        # Constants for navigation
        self.CONFIDENCE_THRESHOLD = 0.3
        self.SPEED_BASE = 1000
        self.TURN_FACTOR = 400
        self.SAFETY_DISTANCE = 0.15  # Minimum distance to keep from buoys
        self.LOOK_AHEAD_WEIGHT = 0.7  # Weight for looking ahead vs immediate path
        
        # Class IDs for different buoys
        self.BLUE_BUOY = 0
        self.RED_BUOY = 1
        self.YELLOW_BUOY = 2
        
        # PID controller parameters
        self.P_GAIN = 600
        self.I_GAIN = 0.1
        self.D_GAIN = 100
        self.integral_error = 0.0
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

    def get_buoys_by_type(self, detections):
        """Organize detected buoys by their type"""
        buoys = {
            self.BLUE_BUOY: [],
            self.RED_BUOY: [],
            self.YELLOW_BUOY: []
        }
        
        for detection in detections:
            if detection.confidence >= self.CONFIDENCE_THRESHOLD:
                if detection.class_id in buoys:
                    buoys[detection.class_id].append(detection)
        
        return buoys

    def calculate_safe_path(self, blue_buoys, red_buoys, yellow_buoys):
        """Calculate a safe path considering all buoys"""
        if not blue_buoys and not red_buoys:
            return None, None
        
        # Sort buoys by x_center to get path progression
        blue_buoys.sort(key=lambda x: x.x_center)
        red_buoys.sort(key=lambda x: x.x_center)
        
        # Get immediate path center
        immediate_center = self.calculate_immediate_center(blue_buoys, red_buoys)
        
        # Calculate look-ahead point considering yellow buoys
        look_ahead_center = self.calculate_look_ahead_point(
            blue_buoys, red_buoys, yellow_buoys
        )
        
        if immediate_center is None:
            return None, None
            
        # Weighted average of immediate and look-ahead centers
        target_center = (immediate_center * (1 - self.LOOK_AHEAD_WEIGHT) + 
                        look_ahead_center * self.LOOK_AHEAD_WEIGHT)
        
        # Calculate closest buoy for speed adjustment
        closest_buoy_width = self.get_closest_buoy_width(
            blue_buoys + red_buoys + yellow_buoys
        )
        
        return target_center, closest_buoy_width

    def calculate_immediate_center(self, blue_buoys, red_buoys):
        """Calculate center point between nearest blue and red buoys"""
        if not blue_buoys and not red_buoys:
            return None
            
        # Get nearest buoys
        nearest_blue = min(blue_buoys, key=lambda x: x.width, default=None) if blue_buoys else None
        nearest_red = min(red_buoys, key=lambda x: x.width, default=None) if red_buoys else None
        
        # If missing one type, estimate position
        if nearest_blue and not nearest_red:
            return nearest_blue.x_center - 0.3  # Assume red is to the left
        elif nearest_red and not nearest_blue:
            return nearest_red.x_center + 0.3   # Assume blue is to the right
        
        return (nearest_blue.x_center + nearest_red.x_center) / 2

    def calculate_look_ahead_point(self, blue_buoys, red_buoys, yellow_buoys):
        """Calculate a look-ahead point considering upcoming buoys"""
        # Get all buoy positions
        all_positions = []
        for buoy in blue_buoys + red_buoys:
            all_positions.append(buoy.x_center)
        
        # If no buoys ahead, maintain current direction
        if not all_positions:
            return 0.5
            
        # Calculate average position of upcoming buoys
        look_ahead = sum(all_positions) / len(all_positions)
        
        # Adjust for yellow buoys (avoid them)
        for yellow in yellow_buoys:
            # If yellow buoy is ahead, adjust path away from it
            distance = abs(look_ahead - yellow.x_center)
            if distance < self.SAFETY_DISTANCE:
                # Move away from yellow buoy
                look_ahead += self.SAFETY_DISTANCE if yellow.x_center < look_ahead else -self.SAFETY_DISTANCE
        
        return max(0.1, min(0.9, look_ahead))  # Keep within bounds

    def get_closest_buoy_width(self, buoys):
        """Get the width of the closest (largest) buoy"""
        if not buoys:
            return 0
        return max(buoy.width for buoy in buoys)

    def calculate_speed(self, closest_buoy_width):
        """Calculate speed based on proximity to closest buoy"""
        if closest_buoy_width > self.SAFETY_DISTANCE:
            return int(self.SPEED_BASE * (1 - closest_buoy_width))
        return self.SPEED_BASE

    def detection_callback(self, msg):
        # Organize buoys by type
        buoys = self.get_buoys_by_type(msg.detections)
        
        # Calculate safe path
        target_center, closest_buoy_width = self.calculate_safe_path(
            buoys[self.BLUE_BUOY],
            buoys[self.RED_BUOY],
            buoys[self.YELLOW_BUOY]
        )
        
        if target_center is None:
            # No viable path found, maintain straight course
            self.thrusters.setSpeedLeft(self.SPEED_BASE)
            self.thrusters.setSpeedRight(self.SPEED_BASE)
            return
        
        # Calculate PID control
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        
        if dt > 0:
            error = 0.5 - target_center
            
            # PID terms
            p_term = error * self.P_GAIN
            self.integral_error += error * dt
            i_term = self.integral_error * self.I_GAIN
            d_term = ((error - self.last_error) / dt) * self.D_GAIN
            
            # Combined control signal
            control = p_term + i_term + d_term
            
            # Update last values
            self.last_error = error
            self.last_time = current_time
            
            # Calculate base speed
            base_speed = self.calculate_speed(closest_buoy_width)
            
            # Apply control to thrusters
            left_speed = int(base_speed + control)
            right_speed = int(base_speed - control)
            
            # Clamp speeds
            left_speed = max(0, min(self.SPEED_BASE, left_speed))
            right_speed = max(0, min(self.SPEED_BASE, right_speed))
            
            # Set thruster speeds
            self.thrusters.setSpeedLeft(left_speed)
            self.thrusters.setSpeedRight(right_speed)
            
            self.get_logger().debug(
                f'Target: {target_center:.2f}, Error: {error:.2f}, '
                f'Speeds - Left: {left_speed}, Right: {right_speed}'
            )

def main(args=None):
    rclpy.init(args=args)
    detection_listener = DetectionListenerNode()
    rclpy.spin(detection_listener)
    detection_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()