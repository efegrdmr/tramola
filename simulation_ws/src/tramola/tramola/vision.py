import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from tramolaa.msg import Detection, DetectionList  # Replace with your custom message types
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2


class YoloNode(Node):
    def __init__(self):
        super().__init__('yolo_detection_node')
        self.get_logger().info('Init start')

        # Load the YOLO model (replace with your model path)
        self.model = YOLO('best.pt')  # Ensure this path is correct and accessible

        # Subscription to the image topic
        self.subscription = self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_right_camera_sensor/image_raw',
            self.listener_callback,
            10
        )

        # Publisher for detection results
        self.publisher = self.create_publisher(DetectionList, 'yolo_detections', 10)

        # Initialize CvBridge
        self.bridge = CvBridge()

        self.get_logger().info('YOLO Node initialized and ready.')

    def listener_callback(self, msg):
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Get image dimensions
            height, width, _ = cv_image.shape

            # Run YOLO model on the image
            detections = self.run_yolo_model(cv_image)

            # Create a DetectionList message
            detection_list = DetectionList()
            for det in detections:
                detection = Detection()

                # Normalize the bounding box coordinates
                x_min, y_min, x_max, y_max = det['bbox']
                x_center = (x_min + x_max) / 2 / width
                y_center = (y_min + y_max) / 2 / height
                box_width = (x_max - x_min) / width
                box_height = (y_max - y_min) / height

                detection.x_center = x_center
                detection.y_center = y_center
                detection.width = box_width
                detection.height = box_height
                detection.confidence = det['confidence']
                detection.class_id = det['class_id']

                detection_list.detections.append(detection)

            # Publish the detections
            self.publisher.publish(detection_list)
            self.get_logger().info(f'Published {len(detection_list.detections)} detections.')

        except Exception as e:
            self.get_logger().error(f'Error in listener_callback: {e}')

    def run_yolo_model(self, image):
        # Run inference on the image
        results = self.model(image)

        detections = []
        for result in results:
            for box in result.boxes:  # Access each detected bounding box
                x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
                confidence = box.conf[0].item()
                class_id = int(box.cls[0].item())

                detections.append({
                    'class_id': class_id,
                    'confidence': confidence,
                    'bbox': [x_min, y_min, x_max, y_max]
                })

        return detections


def main(args=None):
    rclpy.init(args=args)
    try:
        yolo_node = YoloNode()
        rclpy.spin(yolo_node)
    except KeyboardInterrupt:
        pass
    finally:
        yolo_node.get_logger().info('Shutting down YOLO Node')
        yolo_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
