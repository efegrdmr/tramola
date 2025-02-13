#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np

bridge = CvBridge()

def callback(data):
    try:
        # Convert ROS Image message to OpenCV image
        cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr(e)
        return
    
    # Display image
    cv2.imshow("Camera Feed", cv_image)
    # Shutdown ROS if the ESC key is pressed
    if cv2.waitKey(1) & 0xFF == 27:
        rospy.signal_shutdown("User pressed ESC")

if __name__ == '__main__':
    rospy.init_node('camera_viewer', anonymous=True)
    
    # Create a window and show an empty image (black)
    cv2.namedWindow("Camera Feed", cv2.WINDOW_NORMAL)
    blank_image = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.imshow("Camera Feed", blank_image)
    rospy.loginfo("camera started")
    
    
    # Subscribe to the ROS topic (adjust the topic name if needed)
    rospy.Subscriber('/camera/rgb/image_raw', Image, callback)
    
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("Shutting down")
    cv2.destroyAllWindows()
