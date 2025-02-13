#!/usr/bin/env python3
import rospy
from ultralytics import YOLO
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import your ROS messages; for example:
from tramola.msg import Detection, DetectionList

# Initialize ROS node and publishers
rospy.init_node("yolo_v8_ros")
detection_pub = rospy.Publisher("/yolo_detections", DetectionList, queue_size=10)
camera_pub = rospy.Publisher("/camera/image_raw", Image, queue_size=10)

# Create a CV bridge instance
bridge = CvBridge()

# Initialize YOLOv8 model (adjust the path to your custom model)
model = YOLO("/home/tramola/yolov8n.pt")

# Use OpenCV to capture video (adjust device index as needed)
cap = cv2.VideoCapture(0)
rospy.sleep(2)  # Give time for the camera to warm up

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Failed to capture image")
        continue

    # Publish the camera image
    ros_image = bridge.cv2_to_imgmsg(frame, encoding="bgr8")
    camera_pub.publish(ros_image)

    # YOLOv8 expects a BGR image (as provided by cv2.VideoCapture)
    results = model(frame)

    msg = DetectionList()
    for result in results:
        # Iterate over detected bounding boxes
        for box in result.boxes:
            # Get box coordinates ([x1, y1, x2, y2]) as a list of floats
            coords = box.xyxy[0].cpu().numpy().tolist()
            x1, y1, x2, y2 = coords

            # Get image dimensions
            height, width, _ = frame.shape

            # Calculate normalized center, width, and height
            x_center = ((x1 + x2) / 2) / width
            y_center = ((y1 + y2) / 2) / height
            box_width = (x2 - x1) / width
            box_height = (y2 - y1) / height

            # Get confidence and class id
            confidence = box.conf.cpu().numpy()[0]
            class_id = int(box.cls.cpu().numpy()[0])
            
            det_msg = Detection()
            det_msg.x_center = x_center
            det_msg.y_center = y_center
            det_msg.width = box_width
            det_msg.height = box_height
            det_msg.confidence = confidence
            det_msg.class_id = class_id

            msg.detections.append(det_msg)

    rospy.loginfo(f"Detected {len(msg.detections)} objects")
    for det in msg.detections:
        rospy.loginfo(f"Class ID: {det.class_id}, Confidence: {det.confidence:.2f}, "
                      f"Center: ({det.x_center:.2f}, {det.y_center:.2f}), "
                      f"Size: ({det.width:.2f}, {det.height:.2f})")

    detection_pub.publish(msg)

cap.release()
