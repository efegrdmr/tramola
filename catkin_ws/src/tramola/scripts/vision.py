#!/usr/bin/env python3

import os
import cv2
import rospy
from tramola.msg import Detection, DetectionList
from ultralytics import YOLO

def main():
    rospy.init_node('yolo_detection_node', anonymous=True)
    publisher = rospy.Publisher('yolo_detections', DetectionList, queue_size=10)
    rate = rospy.Rate(10)

    # Replace with your YOLO model path
    MODEL_PATH = os.getenv('MODEL_PATH', 'path_to_your_model.pt')
    model = YOLO(MODEL_PATH)

    # Use default USB camera (or replace with another index if needed)
    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        # Run YOLO
        results = model(frame)

        # Build detection list
        height, width, _ = frame.shape
        detection_list = DetectionList()
        for result in results:
            for box in result.boxes:
                x_min, y_min, x_max, y_max = box.xyxy[0].tolist()
                confidence = box.conf[0].item()
                class_id = int(box.cls[0].item())

                detection = Detection()
                x_center = (x_min + x_max) / 2.0 / width
                y_center = (y_min + y_max) / 2.0 / height
                box_width = (x_max - x_min) / width
                box_height = (y_max - y_min) / height

                detection.x_center = x_center
                detection.y_center = y_center
                detection.width = box_width
                detection.height = box_height
                detection.confidence = confidence
                detection.class_id = class_id
                detection_list.detections.append(detection)

        publisher.publish(detection_list)
        rospy.loginfo("Published {} detections.".format(len(detection_list.detections)))
        rate.sleep()

    cap.release()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass