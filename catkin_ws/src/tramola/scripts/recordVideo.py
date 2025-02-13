#!/home/tramola/vision/bin/python3.8
import cv2
import time
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

def record_video(output_file, fps=20.0):
    cap = cv2.VideoCapture(0)
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    out = cv2.VideoWriter(output_file, fourcc, fps, (frame_width, frame_height))

    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                break
            out.write(frame)
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        out.release()

if __name__ == "__main__":
    timestamp = time.strftime("%Y%m%d-%H%M%S")
    output_file = f'output_{timestamp}.avi'
    record_video(output_file)
    rospy.init_node('video_publisher', anonymous=True)
    publisher = rospy.Publisher('/mavros/video_stream', Image, queue_size=10)
    bridge = CvBridge()

    cap = cv2.VideoCapture(0)
    rate = rospy.Rate(10)

    try:
        while not rospy.is_shutdown():
            ret, frame = cap.read()
            if ret:
                msg = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                publisher.publish(msg)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    finally:
        cap.release()