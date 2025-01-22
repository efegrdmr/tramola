import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

from tramola.thruster import Thrusters

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

    def image_callback(self, msg):
        try:
            # Convert ROS image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
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
