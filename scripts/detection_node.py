#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Updated by: efegrdmr
# Last Update: 2025-08-01 18:19:10 UTC

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from tramola.msg import Detection, DetectionList
from tramola.srv import ImageDetection, ImageDetectionResponse
import threading

class ColorDetector:
    def __init__(self):
        rospy.init_node('color_detector', anonymous=True)
        
        # Initialize detection parameters
        self.detection_enabled = False  # Start with detection disabled
        self.bridge = CvBridge()
        self.lock = threading.Lock()
        
        # Region ignore configuration
        self.top_ignore_percent = 0.25   # Ignore top 20% of image
        self.bottom_ignore_percent = 0.25 # Ignore bottom 10% of image
        
        # Class mappings - maps class IDs to display names
        self.class_names = {
            0: "Red",
            1: "Green",
            2: "Black"
        }
        
        # Set up color ranges for maritime environment
        # Red ranges (split into two parts due to HSV wrapping)
        self.lower_red1 = np.array([0, 120, 70])     # Adjusted for maritime conditions
        self.upper_red1 = np.array([25, 255, 255])   # Expanded range to account for sunlight
        self.lower_red2 = np.array([160, 120, 70])   # Adjusted for maritime conditions
        self.upper_red2 = np.array([179, 255, 255])  
        
        # Green range - adjusted for maritime environment
        self.lower_green = np.array([40, 90, 40])    # Expanded to catch more greens
        self.upper_green = np.array([85, 255, 255])  # Expanded upper hue to include more variants
        
        # Black range - adjusted for maritime conditions (shadows might appear darker)
        self.lower_black = np.array([0, 0, 0])
        self.upper_black = np.array([180, 140, 40])  # Increased upper threshold for varying light
        
        # Blue water range - for filtering out the sea
        self.lower_blue = np.array([90, 40, 40])
        self.upper_blue = np.array([130, 255, 255])
        
        # Bright spots range (for sun glare detection)
        self.lower_glare = np.array([0, 0, 200])
        self.upper_glare = np.array([180, 30, 255])
        
        # Morphological kernels
        self.kernel_small = np.ones((3, 3), np.uint8)
        self.kernel_med = np.ones((5, 5), np.uint8)
        
        # Adaptive parameters
        self.min_contour_area = {
            0: 150,    # Lower threshold for red objects
            1: 200,    # Medium threshold for green
            2: 250     # Higher threshold for black to avoid false positives
        }
        
        # Set up ROS publishers and services
        self.image_sub = None  # Will be initialized when detection is enabled
        self.detection_pub = rospy.Publisher('/color_detections', DetectionList, queue_size=10)
        self.toggle_service = rospy.Service('/toggle_detection', ImageDetection, self.toggle_detection_callback)
        
        # Debug image publishers
        self.processed_pub = rospy.Publisher('/color_detector/processed_image', Image, queue_size=10)
        self.mask_pub = rospy.Publisher('/color_detector/debug_mask', Image, queue_size=10)
        
        rospy.loginfo("Color detector initialized and ready for maritime environment")
    
    def toggle_detection_callback(self, req):
        with self.lock:
            # Only make changes if the state is actually changing
            if self.detection_enabled != req.on:
                self.detection_enabled = req.on
                
                if req.on:
                    # Enable subscription to camera topic
                    rospy.loginfo("Enabling image subscription")
                    self.image_sub = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)
                else:
                    # Disable subscription to camera topic
                    rospy.loginfo("Disabling image subscription")
                    if self.image_sub is not None:
                        self.image_sub.unregister()
                        self.image_sub = None
            
            success = True
        
        rospy.loginfo("Detection %s" % ("enabled" if req.on else "disabled"))
        return ImageDetectionResponse(success=success)
    
    def create_ignore_mask(self, height, width):
        """Create a mask that ignores the top and bottom portions of the image"""
        mask = np.ones((height, width), dtype=np.uint8) * 255
        
        # Calculate the cutoff lines for the top and bottom regions
        top_cutoff_y = int(height * self.top_ignore_percent)
        bottom_cutoff_y = int(height * (1.0 - self.bottom_ignore_percent))
        
        # Set the top and bottom portions to zero (ignored)
        mask[0:top_cutoff_y, :] = 0
        mask[bottom_cutoff_y:height, :] = 0
        
        return mask
    
    def image_callback(self, data):
        try:
            # Convert ROS Image message to OpenCV image
            frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
            
            # Get image dimensions for normalization
            height, width, _ = frame.shape
            
            # Create detections message
            detections_msg = DetectionList()
            
            # Create mask to ignore top and bottom portions of image
            ignore_mask = self.create_ignore_mask(height, width)
            
            # Apply preprocessing for maritime conditions
            processed_frame = self.preprocess_maritime_image(frame)
            
            # Convert to HSV color space
            hsv = cv2.cvtColor(processed_frame, cv2.COLOR_BGR2HSV)
            
            # Detect blue (sea/water) to exclude from other detections
            mask_blue = cv2.inRange(hsv, self.lower_blue, self.upper_blue)
            
            # Detect sun glare to exclude from detections
            mask_glare = cv2.inRange(hsv, self.lower_glare, self.upper_glare)
            
            # Combine blue and glare masks to create an exclusion mask
            exclusion_mask = cv2.bitwise_or(mask_blue, mask_glare)
            inv_exclusion_mask = cv2.bitwise_not(exclusion_mask)
            
            # Apply ignore mask to exclusion mask
            inv_exclusion_mask = cv2.bitwise_and(inv_exclusion_mask, ignore_mask)
            
            # IMPROVED RED DETECTION
            # Detect red (combines two HSV ranges)
            mask_red1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask_red2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask_red = cv2.bitwise_or(mask_red1, mask_red2)
            
            # Apply exclusion mask to remove blue water and glare
            mask_red = cv2.bitwise_and(mask_red, inv_exclusion_mask)
            
            # Apply noise removal
            mask_red = self.clean_mask(mask_red)
            
            # Detect green
            mask_green = cv2.inRange(hsv, self.lower_green, self.upper_green)
            mask_green = cv2.bitwise_and(mask_green, inv_exclusion_mask)
            mask_green = self.clean_mask(mask_green)
            
            # Detect black
            mask_black = cv2.inRange(hsv, self.lower_black, self.upper_black)
            mask_black = cv2.bitwise_and(mask_black, inv_exclusion_mask)
            mask_black = self.clean_mask(mask_black)
            
            # Process each color and add detections
            detections_msg.detections.extend(self.process_detections(mask_red, 0, width, height))
            detections_msg.detections.extend(self.process_detections(mask_green, 1, width, height))
            detections_msg.detections.extend(self.process_detections(mask_black, 2, width, height))
            
            # Publish detections
            self.detection_pub.publish(detections_msg)
            
            # Draw and publish processed image
            self.draw_and_publish_frame(frame, detections_msg.detections, width, height)
            
            # Publish debug masks
            combined_mask = cv2.bitwise_or(cv2.bitwise_or(mask_red, mask_green), mask_black)
            try:
                self.mask_pub.publish(self.bridge.cv2_to_imgmsg(combined_mask, "mono8"))
            except CvBridgeError as e:
                rospy.logerr("Error publishing mask: %s" % e)
                
        except CvBridgeError as e:
            rospy.logerr("CV Bridge error: %s" % e)
            
    def preprocess_maritime_image(self, frame):
        """Apply preprocessing specific to maritime environments"""
        # Create a copy to avoid modifying the original
        processed = frame.copy()
        
        # Apply CLAHE (Contrast Limited Adaptive Histogram Equalization) for better contrast
        # Convert to LAB color space for better color processing
        lab = cv2.cvtColor(processed, cv2.COLOR_BGR2LAB)
        l_channel, a_channel, b_channel = cv2.split(lab)
        
        # Apply CLAHE to the L channel
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
        cl = clahe.apply(l_channel)
        
        # Merge the CLAHE enhanced L-channel with the original a and b channels
        merged_lab = cv2.merge((cl, a_channel, b_channel))
        
        # Convert back to BGR color space
        processed = cv2.cvtColor(merged_lab, cv2.COLOR_LAB2BGR)
        
        # Apply slight Gaussian blur to reduce noise and reflections
        processed = cv2.GaussianBlur(processed, (3, 3), 0)
        
        return processed
    
    def clean_mask(self, mask):
        """Apply morphological operations to clean the mask"""
        # First remove small noise with opening
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, self.kernel_small)
        
        # Then close small holes inside objects
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, self.kernel_med)
        
        return mask
    
    def process_detections(self, mask, class_id, width, height):
        detections = []
        
        # Find contours in the mask - Python 2 version returns 3 values
        _, contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Use dynamic area threshold based on class
        min_area = self.min_contour_area[class_id]
        
        for c in contours:
            area = cv2.contourArea(c)
            if area > min_area:
                x, y, w, h = cv2.boundingRect(c)
                
                # Calculate solidity (area / convex hull area) for filtering irregular shapes
                hull = cv2.convexHull(c)
                hull_area = cv2.contourArea(hull)
                solidity = float(area) / hull_area if hull_area > 0 else 0
                
                # Filter out very irregular shapes (likely waves or reflections)
                if solidity < 0.3 and class_id != 2:  # Not for black
                    continue
                
                # Create a detection message
                detection = Detection()
                # Ensure floating point division in Python 2
                detection.x_center = float(x + w/2.0) / float(width)  # Normalize to 0-1
                detection.y_center = float(y + h/2.0) / float(height)  # Normalize to 0-1
                detection.width = float(w) / float(width)  # Normalize to 0-1
                detection.height = float(h) / float(height)  # Normalize to 0-1
                
                # More sophisticated confidence calculation
                confidence_area = min(1.0, float(area) / 10000.0)
                confidence_solidity = min(1.0, solidity)
                detection.confidence = (confidence_area * 0.7) + (confidence_solidity * 0.3)
                
                detection.class_id = class_id
                
                detections.append(detection)
        
        return detections
    
    def draw_and_publish_frame(self, frame, detections, width, height):
        # Create a copy of the frame to draw on
        draw_frame = frame.copy()
        
        # Calculate cutoff lines for the top and bottom ignored regions
        top_cutoff_y = int(height * self.top_ignore_percent)
        bottom_cutoff_y = int(height * (1.0 - self.bottom_ignore_percent))
        
        # Draw horizontal lines at the cutoff points
        cv2.line(draw_frame, (0, top_cutoff_y), (width, top_cutoff_y), (0, 255, 255), 2)
        cv2.line(draw_frame, (0, bottom_cutoff_y), (width, bottom_cutoff_y), (0, 255, 255), 2)
        
        # Add labels for the ignored regions
        cv2.putText(draw_frame, "Ignored Region", (width//2 - 80, top_cutoff_y - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        cv2.putText(draw_frame, "Ignored Region", (width//2 - 80, bottom_cutoff_y + 25), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
        
        # Draw bounding boxes for all detections
        for detection in detections:
            # Convert normalized coordinates back to pixel values
            x = int(detection.x_center * width - (detection.width * width / 2.0))
            y = int(detection.y_center * height - (detection.height * height / 2.0))
            w = int(detection.width * width)
            h = int(detection.height * height)
            
            # Set color based on class
            if detection.class_id == 0:  # Red
                color = (0, 0, 255)
                class_name = "Red"
            elif detection.class_id == 1:  # Green
                color = (0, 255, 0)
                class_name = "Green"
            elif detection.class_id == 2:  # Black
                color = (0, 0, 0)
                class_name = "Black"
            else:
                color = (255, 255, 255)
                class_name = "Unknown"
            
            # Draw rectangle and label
            cv2.rectangle(draw_frame, (x, y), (x + w, y + h), color, 2)
            label = "%s: %.2f" % (class_name, detection.confidence)
            cv2.putText(draw_frame, label, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        
        # Add timestamp and info
        timestamp = "Marine Detection - Active | " + rospy.get_time().__str__()
        cv2.putText(draw_frame, timestamp, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
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