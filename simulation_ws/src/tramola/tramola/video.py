import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import time
import os 

from tramola.thruster import Thrusters
from tramolaa.msg import DetectionList, Detection


class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')
        self.subscription = self.create_subscription(
            Image,
            '/wamv/sensors/cameras/front_right_camera_sensor/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.thrusters = Thrusters(self)
        # Create a blank image for initialization
        self.blank_image = np.zeros((720, 1280, 3), dtype=np.uint8)
        cv2.namedWindow("Camera View", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("Camera View", 1280, 720)  # Ensuring the window is a specific size
        cv2.imshow("Camera View", self.blank_image)
        self.recording = False
        self.detecting = False
        self.detections = []
        self.subscription = self.create_subscription(
            DetectionList,
            'yolo_detections',
            self.detection_callback,
            10  # Queue size
        )

    def detection_callback(self, msg):
        self.detections.clear()
        for detection in msg.detections:
            self.detections.append(detection)

    def toggle_recording(self):
        """Toggle video recording state."""
        if self.recording:
            # Stop recording
            self.get_logger().info("Stopping video recording...")
            self.recording = False
            if self.video_writer is not None:
                self.video_writer.release()
                self.video_writer = None
        else:
            # Start recording
            self.get_logger().info("Starting video recording...")
            self.recording = True
            VIDEO_PATH = os.getenv("VIDEO_PATH")
            self.video_writer = cv2.VideoWriter(
                VIDEO_PATH+"/video_" + str(time.time()) + ".avi",
                cv2.VideoWriter_fourcc(*'XVID'),
                20,
                (1280, 720)
)


    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.detecting:
                img_height, img_width, _ = cv_image.shape
                for detection in self.detections:

                    # Normalized values (example)
                    x_center = detection.x_center # Normalized x-center
                    y_center = detection.y_center  # Normalized y-center
                    width = detection.width     # Normalized width
                    height = detection.height   # Normalized height

                    # Denormalize to get pixel coordinates
                    box_width = int(width * img_width)
                    box_height = int(height * img_height)
                    x = int((x_center * img_width) - box_width / 2)  # Top-left x
                    y = int((y_center * img_height) - box_height / 2)  # Top-left y

                    # Draw the bounding box
                    color = (0, 255, 0)  # Green
                    thickness = 2
                    cv2.rectangle(cv_image, (x, y), (x + box_width, y + box_height), color, thickness)
                    text = f"Confidence {detection.confidence:.2f} id {detection.class_id}"
                    cv2.putText(cv_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
           
            if self.recording:
                self.video_writer.write(cv_image)
           
            cv2.imshow("Camera View", cv_image)
        except Exception as e:
            self.get_logger().error(f"Error displaying image: {str(e)}")
        
        


    def handle_keypress(self, key):
        """Handle keypress for thruster control."""
        if key == ord('w'):
            self.thrusters.increaseSpeed()
        elif key == ord('s'):
            self.thrusters.decreaseSpeed()
        elif key == ord('a'):
            self.thrusters.turnLeft()
        elif key == ord('d'):
            self.thrusters.turnRight()
        elif key == ord('c'):
            self.toggle_recording()
        elif key== ord('r'):
            self.detecting = not self.detecting
        elif key == ord('q'):
            self.get_logger().info("Shutting down...")
            return True  # Return True to indicate quit request
        return False


def main(args=None):
    rclpy.init(args=args)
    viewer = ImageViewer()

    try:
        while rclpy.ok():
            key = cv2.waitKey(1) & 0xFF
            if viewer.handle_keypress(key):  # Check if 'q' was pressed
                break
            rclpy.spin_once(viewer)
    except KeyboardInterrupt:
        pass
    finally:
        # Cleanup and shutdown
        cv2.destroyAllWindows()
        viewer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
