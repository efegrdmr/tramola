#!/usr/bin/env python2.7
import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def main():
    rospy.init_node('camera_pub', anonymous=True)
    pub = rospy.Publisher("camera/image_raw", Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        rospy.logerr("Could not access the camera")
        return

    rate = rospy.Rate(10)  # publish at 10 Hz

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if ret:
            try:
                img_msg = bridge.cv2_to_imgmsg(frame, "bgr8")
                pub.publish(img_msg)
            except Exception as e:
                rospy.logerr("Error converting frame: %s", e)
        else:
            rospy.logwarn("Failed to grab frame from camera")
        
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
