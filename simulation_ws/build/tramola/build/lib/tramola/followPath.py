import rclpy
from rclpy.node import Node
from tramolaa.msg import DetectionList  # Adjust with the correct package
from tramolaa.msg import Detection
from tramola.thruster import Thrusters

class DetectionListenerNode(Node):
    def __init__(self):
        super().__init__('detection_listener')

        # Create a subscription to the 'detection_topic' topic
        self.subscription = self.create_subscription(
            DetectionList,
            'yolo_detections',
            self.detection_callback,
            10  # Queue size
        )
        self.thrusters = Thrusters(self)


    def detection_callback(self, msg):
        # 0 blue buoy
        # 1 red buoy
        nearestBlue = None
        nearestRed = None
        for detection in msg.detections:
            if detection.confidence < 0.7:
                self.get_logger().warn(f"Low confidence: {detection.confidence}. Ignoring detection.")
                continue

            if detection.class_id == 0:
                if nearestBlue is None or nearestBlue.width < detection.width:
                    nearestBlue = detection
            
            elif detection.class_id == 1:
                if nearestRed is None or nearestRed.width < detection.width:
                    nearestRed = detection

        if nearestBlue is None and nearestRed is not None:
            nearestBlue = Detection()
            nearestBlue.x_center = 1 - nearestRed.x_center
        elif nearestRed is None and nearestBlue is not None:
            nearestRed = Detection()
            nearestRed.x_center = 1 - nearestBlue.x_center
        elif nearestRed is None and nearestBlue is None:
            self.thrusters.setSpeedLeft(1000)
            self.thrusters.setSpeedRight(1000)
            return
        
        middle = (nearestRed.x_center + nearestBlue.x_center) / 2

        if middle < 0.5:
            self.thrusters.setSpeedLeft(600)
            self.thrusters.setSpeedRight(1000)
        else:
            self.thrusters.setSpeedLeft(1000)
            self.thrusters.setSpeedRight(600)

        for detection in msg.detections:
            # Check if the confidence score is above a threshold
            if detection.confidence > 0.7:
                self.get_logger().info(f"Detected object with class ID {detection.class_id} at center ({detection.x_center}, {detection.y_center}) with confidence {detection.confidence}")
                
                # Take action based on class_id
                if detection.class_id == 1:
                    self.get_logger().info("Object is a car. Triggering car-related action.")
                    # Perform action related to car detection
                
                elif detection.class_id == 2:
                    self.get_logger().info("Object is a pedestrian. Triggering pedestrian-related action.")
                    # Perform action related to pedestrian detection
                
                # Add more actions for different class IDs as needed
            else:
                self.get_logger().warn(f"Low confidence: {detection.confidence}. Ignoring detection.")

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    detection_listener = DetectionListenerNode()

    # Spin the node so it keeps listening for messages
    rclpy.spin(detection_listener)

    # Shutdown the node
    detection_listener.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
