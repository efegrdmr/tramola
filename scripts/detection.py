#!/usr/bin/env python
# -*- coding: utf-8 -*-

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tramola.msg import Detection, Detections
from tramola.srv import ImageDetection, ImageDetectionResponse
import threading

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector', anonymous=True)
        
        # Initialize detection parameters
        self.detection_enabled = True
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        # Class mappings
        self.class_mapping = {
            "Red": 0,
            "Green": 1,
            "Black": 2
        }
        
        # Set up color ranges
        # Red ranges (split into two parts due to HSV wrapping)
        self.lower_red1 = np.array([0, 150, 100])
        self.upper_red1 = np.array([15, 255, 255])
        self.lower_red2 = np.array([170, 150, 100])
        self.upper_red2 = np.array([179, 255, 255])
        
        # Skin range in YCbCr for filtering
        self.lower_skin = np.array([0, 135, 85])
        self.upper_skin = np.array([255, 173, 127])
        
        # Green range
        self.lower_green = np.array([47, 100, 50])
        self.upper_green = np.array([79, 255, 255])
        
        # Black range
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 97, 27])
        
        # Morphological kernel for noise removal
        self.kernel = np.ones((5, 5), np.uint8)
        
        # Set up ROS publishers, subscribers, and services
        self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
        self.detection_pub = rospy.Publisher('/color_detections', Detections, queue_size=10)
        self.toggle_service = rospy.Service('/toggle_detection', ImageDetection, self.toggle_detection_callback)
        
        # Optionally publish processed image
        self.processed_pub = rospy.Publisher('/color_detector/processed_image', Image, queue_size=10)
        
        rospy.loginfo("Color detector initialized and ready")
    
    def toggle_detection_callback(self, req):
        with self.lock:
            self.detection_enabled = req.on
            success = True
        
        rospy.loginfo("Detection %s" % ("enabled" if req.on else "disabled"))
        return ImageDetectionResponse(success=success)
    
    def image_callback(self, data):
        if not self.detection_enabled:
            return
        
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Get image dimensions for normalization
            height, width, _ = frame.shape
            
            # Create detections message
            detections_msg = Detections()
            detections_msg.header = data.header
            
            # Process image to detect colors
            # Convert to HSV and YCbCr color spaces
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            ycbcr_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2YCR_CB)
            
            # Detect red (combines two HSV ranges)
            mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            # Filter out skin tones
            skin_mask = cv2.inRange(ycbcr_frame, self.lower_skin, self.upper_skin)
            true_red_mask = cv2.bitwise_and(mask_red, mask_red, mask=cv2.bitwise_not(skin_mask))
            
            # Noise removal
            true_red_mask = cv2.erode(true_red_mask, self.kernel, iterations=1)
            true_red_mask = cv2.dilate(true_red_mask, self.kernel, iterations=2)
            
            # Detect green
            mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
            
            # Detect black
            mask_black = cv2.inRange(hsv, self.lower_black, self.upper_black)
            
            # Process each color and add detections
            detections_msg.detections.extend(self.process_detections(true_red_mask, "Red", width, height))
            detections_msg.detections.extend(self.process_detections(mask_green, "Green", width, height))
            detections_msg.detections.extend(self.process_detections(mask_black, "Black", width, height))
            
            # Publish detections
            self.detection_pub.publish(detections_msg)
            
            # Optionally draw bounding boxes and publish the processed image
            self.draw_and_publish_frame(frame, detections_msg.detections, width, height)
            
        except CvBridgeError as e:
            rospy.logerr("CV Bridge error: %s" % e)
    
    def process_detections(self, mask, color_name, width, height):
        detections = []
        
        # Find contours in the mask - Python 2 version returns 3 values
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in contours:
            area = cv2.contourArea(c)
            if area > 400:  # Filter small contours
                x, y, w, h = cv2.boundingRect(c)
                
                # Create a detection message
                detection = Detection()
                # Ensure floating point division in Python 2
                detection.x_center = float(x + w/2.0) / float(width)  # Normalize to 0-1
                detection.y_center = float(y + h/2.0) / float(height)  # Normalize to 0-1
                detection.width = float(w) / float(width)  # Normalize to 0-1
                detection.height = float(h) / float(height)  # Normalize to 0-1
                detection.confidence = min(1.0, float(area) / 10000.0)  # Simple confidence metric
                detection.class_id = self.class_mapping[color_name]
                detection.class_name = color_name
                
                detections.append(detection)
        
        return detections
    
    def draw_and_publish_frame(self, frame, detections, width, height):
        # Create a copy of the frame to draw on
        draw_frame = frame.copy()
        
        # Draw bounding boxes for all detections
        for detection in detections:
            # Convert normalized coordinates back to pixel values
            x = int(detection.x_center * width - (detection.width * width / 2.0))
            y = int(detection.y_center * height - (detection.height * height / 2.0))
            w = int(detection.width * width)
            h = int(detection.height * height)
            
            # Set color based on class
            if detection.class_name == "Red":
                color = (0, 0, 255)
            elif detection.class_name == "Green":
                color = (0, 255, 0)
            elif detection.class_name == "Black":
                color = (0, 0, 0)
            else:
                color = (255, 255, 255)
            
            # Draw rectangle and label
            cv2.rectangle(draw_frame, (x, y), (x + w, y + h), color, 2)
            label = "%s: %.2f" % (detection.class_name, detection.confidence)
            cv2.putText(draw_frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Publish the processed image
        try:
            self.processed_pub.publish(self.bridge.cv2_to_imgmsg(draw_frame, "bgr8"))
        except CvBridgeError as e:
            rospy.logerr("Error publishing processed image: %s" % e)

if __name__ == '__main__':
    try:
        detector = ColorDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass