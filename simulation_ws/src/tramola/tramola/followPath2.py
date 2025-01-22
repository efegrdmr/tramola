import rclpy
from rclpy.node import Node
from tramolaa.msg import DetectionList, Detection
from tramola.thruster import Thrusters

class DetectionListenerNode(Node):
    def __init__(self):
        super().__init__('detection_listener')
        
        # Create a subscription to the detection topic
        self.subscription = self.create_subscription(
            DetectionList,
            'yolo_detections',
            self.detection_callback,
            10
        )
        self.thrusters = Thrusters(self)
        
        # Constants for navigation
        self.CONFIDENCE_THRESHOLD = 0.3
        self.TARGET_CENTER = 0.5  # Desired position in frame
        self.SPEED_BASE = 1000    # Base speed
        self.TURN_FACTOR = 400    # How much to reduce speed for turning
        self.DISTANCE_THRESHOLD = 0.4  # How close buoys should be before slowing down
        
        # PID controller parameters
        self.P_GAIN = 800
        self.last_error = 0.0
        self.last_time = self.get_clock().now()

    def get_nearest_buoy(self, detections, class_id):
        """Find the nearest (largest) buoy of given class"""
        nearest = None
        max_width = 0
        
        for detection in detections:
            if (detection.confidence >= self.CONFIDENCE_THRESHOLD and 
                detection.class_id == class_id and 
                detection.width > max_width):
                nearest = detection
                max_width = detection.width
                
        return nearest

    def calculate_steering(self, middle, current_time):
        """Calculate steering using PID control"""
        error = self.TARGET_CENTER - middle
        
        # Calculate time difference
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt == 0:
            return 0
            
        # Proportional term
        p_term = error * self.P_GAIN
        
        # Update last values
        self.last_error = error
        self.last_time = current_time
        
        return p_term

    def adjust_speed_for_distance(self, buoy_width):
        """Adjust speed based on proximity to buoys"""
        if buoy_width > self.DISTANCE_THRESHOLD:
            return int(self.SPEED_BASE * (1 - buoy_width))
        return self.SPEED_BASE

    def detection_callback(self, msg):
        # Find nearest buoys
        nearest_blue = self.get_nearest_buoy(msg.detections, 0)
        nearest_red = self.get_nearest_buoy(msg.detections, 1)
        
        # If only one buoy is visible, estimate the other's position
        if nearest_blue is None and nearest_red is not None:
            nearest_blue = Detection()
            nearest_blue.x_center = max(0.0, min(1.0, 1 - nearest_red.x_center))
            nearest_blue.width = nearest_red.width
        elif nearest_red is None and nearest_blue is not None:
            nearest_red = Detection()
            nearest_red.x_center = max(0.0, min(1.0, 1 - nearest_blue.x_center))
            nearest_red.width = nearest_blue.width
        elif nearest_red is None and nearest_blue is None:
            # No buoys visible - maintain straight course at full speed
            self.thrusters.setSpeedLeft(self.SPEED_BASE)
            self.thrusters.setSpeedRight(self.SPEED_BASE)
            self.get_logger().warn('No buoys detected')
            return
        
        # Calculate middle point between buoys
        middle = (nearest_red.x_center + nearest_blue.x_center) / 2
        
        # Calculate steering adjustment using PID
        current_time = self.get_clock().now()
        steering = self.calculate_steering(middle, current_time)
        
        # Calculate base speed based on proximity to buoys
        max_buoy_width = max(nearest_red.width, nearest_blue.width)
        base_speed = self.adjust_speed_for_distance(max_buoy_width)
        
        # Apply steering adjustments to thrusters
        left_speed = base_speed + steering
        right_speed = base_speed - steering
        
        # Clamp speeds to valid range
        left_speed = max(0, min(self.SPEED_BASE, int(left_speed)))
        right_speed = max(0, min(self.SPEED_BASE, int(right_speed)))
        
        # Set thruster speeds
        self.thrusters.setSpeedLeft(left_speed)
        self.thrusters.setSpeedRight(right_speed)
        
        # Log current state
        self.get_logger().debug(
            f'Middle: {middle:.2f}, Steering: {steering:.2f}, '
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