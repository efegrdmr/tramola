#!/usr/bin/env python

import rospy
import cv2
import time
import os
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from your_package_name.msg import Detections  # Import your custom message

# Global variables
latest_detections = None
video_writer = None
last_detection_time = 0  # Timestamp of the last detection received

def detections_callback(data):
    """
    Callback function to store the latest received detections.
    """
    global latest_detections, last_detection_time
    latest_detections = data.detections
    last_detection_time = time.time()  # Update the timestamp

def camera_recorder():
    """
    Initializes the ROS node, publisher, subscriber, and camera.
    Continuously captures frames, publishes raw frames, and writes to file.
    Records raw frames by default, but switches to annotated frames when detections are valid.
    """
    global latest_detections, video_writer, last_detection_time

    # Initialize the ROS node
    rospy.init_node('camera_recorder_node', anonymous=True)

    # --- Parameters ---
    # Get parameters from the ROS parameter server or use defaults
    image_topic = rospy.get_param('~image_topic', 'camera/image_raw')
    detections_topic = rospy.get_param('~detections_topic', '/detections')
    output_dir = rospy.get_param('~output_dir', '.')
    output_video_path = os.path.join(output_dir, 'output.avi')
    camera_index = rospy.get_param('~camera_index', 0)
    publish_rate = rospy.get_param('~publish_rate', 30)
    detection_timeout = rospy.get_param('~detection_timeout', 1.0)  # Timeout in seconds

    # --- Publisher and Subscriber ---
    # Create a publisher for the raw image
    image_pub = rospy.Publisher(image_topic, Image, queue_size=10)

    # Create a subscriber for the detections topic
    rospy.Subscriber(detections_topic, Detections, detections_callback)

    # --- OpenCV and CvBridge ---
    bridge = CvBridge()
    cap = cv2.VideoCapture(camera_index)

    if not cap.isOpened():
        rospy.logerr("Could not open video device")
        return

    # Get camera frame dimensions
    frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

    # Ensure output directory exists
    if not os.path.exists(output_dir):
        os.makedirs(output_dir)

    # Initialize VideoWriter
    fourcc = cv2.VideoWriter_fourcc(*'XVID')
    video_writer = cv2.VideoWriter(output_video_path, fourcc, publish_rate, (frame_width, frame_height))

    rate = rospy.Rate(publish_rate)

    rospy.loginfo("Camera recorder node started...")

    while not rospy.is_shutdown():
        ret, frame = cap.read()

        if ret:
            # Always publish the raw frame
            try:
                ros_image = bridge.cv2_to_imgmsg(frame, "bgr8")
                image_pub.publish(ros_image)
            except CvBridgeError as e:
                rospy.logerr(e)
            
            # Check if detections are available and not too old
            current_time = time.time()
            detections_valid = (latest_detections is not None and 
                               (current_time - last_detection_time) <= detection_timeout)
            
            # If there are valid detections, annotate the frame before writing
            if detections_valid:
                frame_to_write = frame.copy()  # Create a copy for annotation
                
                for detection in latest_detections:
                    # Denormalize the bounding box coordinates
                    x_center = int(detection.x_center * frame_width)
                    y_center = int(detection.y_center * frame_height)
                    width = int(detection.width * frame_width)
                    height = int(detection.height * frame_height)

                    # Calculate top-left and bottom-right corners
                    x1 = int(x_center - width / 2)
                    y1 = int(y_center - height / 2)
                    x2 = int(x_center + width / 2)
                    y2 = int(y_center + height / 2)

                    # Draw the bounding box
                    cv2.rectangle(frame_to_write, (x1, y1), (x2, y2), (0, 255, 0), 2)

                    # Put the class ID and confidence as text
                    label = "ID: {} Conf: {:.2f}".format(detection.class_id, detection.confidence)
                    cv2.putText(frame_to_write, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
                
                # Add timestamp to annotated frame
                timestamp = "Time: {}".format(time.strftime("%Y-%m-%d %H:%M:%S", time.localtime()))
                cv2.putText(frame_to_write, timestamp, (10, frame_height - 10), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Write the annotated frame to the video file
                video_writer.write(frame_to_write)
            else:
                # Write the raw frame to the video file (no annotations)
                video_writer.write(frame)

        rate.sleep()

    # Release resources
    cap.release()
    if video_writer:
        video_writer.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        camera_recorder()
    except rospy.ROSInterruptException:
        pass