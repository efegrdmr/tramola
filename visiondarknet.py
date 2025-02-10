import os
import cv2
import rospy
import darknet
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
from tramola.msg import Detection, DetectionList

# Load YOLOv4 model
CONFIG_FILE = os.getenv('MODEL_CFG', 'yolov4.cfg')
WEIGHTS_FILE = os.getenv('MODEL_WEIGHTS', 'yolov4.weights')
DATA_FILE = os.getenv('DATA_FILE', 'coco.data')

network, class_names, class_colors = darknet.load_network(CONFIG_FILE, DATA_FILE, WEIGHTS_FILE, batch_size=1)
width = darknet.network_width(network)
height = darknet.network_height(network)

def convert_bbox(x, y, w, h, img_w, img_h):
    x = int((x - w / 2) * img_w)
    y = int((y - h / 2) * img_h)
    w = int(w * img_w)
    h = int(h * img_h)
    return x, y, w, h

def main():
    rospy.init_node('yolo_detection_node', anonymous=True)
    detection_pub = rospy.Publisher('yolo_detections', DetectionList, queue_size=10)
    image_pub = rospy.Publisher('/camera/image_raw', Image, queue_size=10)
    bridge = CvBridge()
    rate = rospy.Rate(10)

    cap = cv2.VideoCapture(0)

    while not rospy.is_shutdown():
        ret, frame = cap.read()
        if not ret:
            continue

        # Publish raw camera frame
        image_pub.publish(bridge.cv2_to_imgmsg(frame, "bgr8"))

        # Convert frame to darknet format
        darknet_image = darknet.make_image(width, height, 3)
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        frame_resized = cv2.resize(frame_rgb, (width, height))
        darknet.copy_image_from_bytes(darknet_image, frame_resized.tobytes())

        # Run YOLO detection
        detections = darknet.detect_image(network, class_names, darknet_image, thresh=0.5)
        darknet.free_image(darknet_image)

        height, width, _ = frame.shape
        detection_list = DetectionList()

        for label, confidence, bbox in detections:
            x, y, w, h = convert_bbox(*bbox, width, height)

            det_msg = Detection()
            det_msg.class_id = class_names.index(label)
            det_msg.class_name = label
            det_msg.confidence = float(confidence)
            det_msg.x = x
            det_msg.y = y
            det_msg.width = w
            det_msg.height = h
            detection_list.detections.append(det_msg)
        
        detection_pub.publish(detection_list)
        rate.sleep()

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
