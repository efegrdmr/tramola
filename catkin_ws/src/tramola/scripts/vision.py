#!/home/tramola/vision/bin/python3.8
import rospy
from ultralytics import YOLO
import cv2

from std_msgs.msg import Bool, Int32

from tramola.srv import greenLightDetectionState, greenLightDetectionStateResponse
from tramola.srv import inferenceState, inferenceStateResponse
from tramola.srv import selectModel, selectModelResponse
from tramola.srv import yellowDetectionState, yellowDetectionStateResponse

from tramola.msg import Detection, DetectionList
from tramola.balloonTracker import BalloonTracker

detect_green_light = False
detect_yellow = False
detect_objects = False
model = None
tracker = None

def green_light_detection_state_handler(req):
    global detect_green_light
    # Example callback for green light detection service
    if req.on:
        response = greenLightDetectionStateResponse()
        detect_green_light = True
        response.success = True
        return response
    else:
        response = greenLightDetectionStateResponse()
        detect_green_light = False
        response.success = True
        return response

def inference_state_handler(req):
    global detect_objects
    # Example callback for inference state service
    if req.on:
        response = inferenceStateResponse()
        detect_objects = True
        response.success = True
        return response
    else:
        response = inferenceStateResponse()
        detect_objects = False
        response.success = True
        return response

def select_model_handler(req):
    global model
    if model is not None:
        del model
    model = YOLO(req.model_path)
    response = selectModelResponse()
    response.success = True
    return response

def yellow_detection_state_handler(req):
    global detect_yellow, tracker
    if req.on:
        response = yellowDetectionStateResponse()
        detect_yellow = True
        if tracker is not None:
            del tracker
        tracker = BalloonTracker()
        response.success = True
        return response
    else:
        response = yellowDetectionStateResponse()
        detect_yellow = False
        del tracker
        tracker = None
        response.success = True
        return response


rospy.init_node("vision")

# Services
rospy.Service("/green_light_detection_state", greenLightDetectionState, green_light_detection_state_handler)
rospy.Service("/inference_state", inferenceState, inference_state_handler)
rospy.Service("/select_model", selectModel, select_model_handler)
rospy.Service("/yellow_detection_state", yellowDetectionState, yellow_detection_state_handler)

# Publishers
green_light_detection_pub = rospy.Publisher("/green_light_detection", Bool, queue_size=1)
detection_pub = rospy.Publisher("/yolo_detections", DetectionList, queue_size=1)
yellow_counter_pub = rospy.Publisher("/yellow_counter", Int32, queue_size=1)

# Create a CV bridge instance
bridge = CvBridge()

# Use OpenCV to capture video (adjust device index as needed)
cap = cv2.VideoCapture(0)
rospy.sleep(2)  # Give time for the camera to warm up
names = ['blue circle', 'blue plus', 'blue square', 'blue triangle', 
        'green circle', 'green plus', 'green square', 'green triangle', 
        'red circle', 'red plus', 'red square', 'red triangle',
        'black balloon', 'green balloon', 'red balloon', 'yellow balloon',
        'black plus', 'black triangle']

while not rospy.is_shutdown():
    ret, frame = cap.read()
    if not ret:
        rospy.logwarn("Failed to capture image")
        continue

    if detect_green_light:
        # TODO: Detect green light and publish to /green_light_detection
        res = Bool(data=False)
        green_light_detection_pub.publish(res)

    

    if detect_objects and model is not None: 
        # YOLOv8 expects a BGR image (as provided by cv2.VideoCapture)
        results = model(frame)
        current_detections = []
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

                # Yellow detection
                if detect_yellow:
                    count = Int32(data=0)
                    cls = int(box.cls[0])
                    conf = float(box.conf[0])
                    
                    if cls == 15:
                        bbox = box.xyxy[0].cpu().numpy()
                        current_detections.append((bbox, conf))

                
                det_msg = Detection()
                det_msg.x_center = x_center
                det_msg.y_center = y_center
                det_msg.width = box_width
                det_msg.height = box_height
                det_msg.confidence = confidence
                det_msg.class_id = class_id

                msg.detections.append(det_msg)

        if detect_yellow:
            processed_detections = tracker.process_detections(current_detections)
            yellow_count = Int32(tracker.balloon_count)
            yellow_counter_pub.publish(count)

        rospy.loginfo(f"Detected {len(msg.detections)} objects")
        for det in msg.detections:
            rospy.loginfo(f"Class: {names[det.class_id]}, Confidence: {det.confidence:.2f}, "
                        f"Center: ({det.x_center:.2f}, {det.y_center:.2f}), "
                        f"Size: ({det.width:.2f}, {det.height:.2f})")

        detection_pub.publish(msg)

cap.release()
