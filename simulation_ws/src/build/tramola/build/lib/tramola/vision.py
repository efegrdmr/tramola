import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tramola.msg import Detection, DetectionList  # Replace with the actual package and message names
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO

class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')

        # Load the YOLO model (You can change the model path to any YOLO model)
        self.model = YOLO('/home/efe/best.pt')  # Replace with your specific YOLO model path

        # Subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_right_camera_sensor/image_raw',
            self.listener_callback,
            1
        )

        # Publisher for detection results
        self.publisher = self.create_publisher(DetectionList, 'yolo_detections', 10)

        # Initialize CvBridge
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLO model on the image
        detections = self.run_yolo_model(cv_image)

        # Create a DetectionList message
        detection_list = DetectionList()
        for det in detections:
            detection = Detection()
            detection.class_id = det['class_id']
            detection.confidence = det['confidence']
            detection.xmin = det['bbox'][0]
            detection.ymin = det['bbox'][1]
            detection.xmax = det['bbox'][2]
            detection.ymax = det['bbox'][3]
            detection_list.detections.append(detection)

        # Publish the detections
        self.publisher.publish(detection_list)

    def run_yolo_model(self, image):
        # Run inference on the image
        results = self.model(image)

        detections = []
        for result in results.xywh[0]:  # Iterate through each detected object
            x_center, y_center, width, height = result[:4]
            confidence = result[4]
            class_id = int(result[5])

            # Create a Detection message for each detected object
            detection_msg = Detection(
                x_center=x_center,
                y_center=y_center,
                width=width,
                height=height,
                confidence=confidence,
                class_id=class_id
            )
            detections.append(detection_msg)

        # Create the DetectionList message containing all detections
        detections_msg = DetectionList(objects=detections)
        
        # Publish the message
        self.publisher.publish(detections_msg)
        self.get_logger().info('Published YOLO detections')

def main(args=None):
    rclpy.init(args=args)
    yolo_node = YoloNode()
    rclpy.spin(yolo_node)

    # Destroy the node explicitly
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
