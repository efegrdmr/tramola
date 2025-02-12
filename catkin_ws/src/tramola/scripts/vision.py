#!/usr/bin/env python3
import rospy
import jetson_inference
import jetson_utils
from tramola.msg import Detection, DetectionList

# Initialize ROS node
rospy.init_node("yolo_trt_ros")
pub = rospy.Publisher("/yolo_detections", DetectionList, queue_size=10)

# Load custom TensorRT model
net = jetson_inference.detectNet(argv=['--model=/home/tramola/yolov4-custom.trt', '--labels=/home/tramola/labels.txt', ' --input-blob=input',  '--output-cvg=confs',  '--output-bbox=boxes'])

camera = jetson_utils.gstCamera(1280, 720, "/dev/video0")  
rospy.sleep(2)  # Give time for camera to initialize


while not rospy.is_shutdown():
    img, width, height = camera.CaptureRGBA()
    detections = net.Detect(img, width, height)
    
    msg = DetectionList()
    for d in detections:
        det_msg = Detection()
        det_msg.x_center = d.Center[0] / width
        det_msg.y_center = d.Center[1] / height
        det_msg.width = d.Width / width
        det_msg.height = d.Height / height
        det_msg.confidence = d.Confidence
        det_msg.class_id = d.ClassID
        msg.detections.append(det_msg)

    rospy.loginfo(f"Detected {len(detections)} objects")
    for det in msg.detections:
        rospy.loginfo(f"Class ID: {det.class_id}, Confidence: {det.confidence:.2f}, "
                      f"Center: ({det.x_center:.2f}, {det.y_center:.2f}), "
                      f"Size: ({det.width:.2f}, {det.height:.2f})")

    pub.publish(msg)
