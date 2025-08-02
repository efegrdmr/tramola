import rospy
from tramola.msg import DetectionList, Detection
from tramola.srv import imageDetection

class Detection:
    def __init__(self):
        assert rospy.core.is_initialized(), "ROS node is not initialized"
        

        self.sub = rospy.Subscriber("/color_detections", DetectionList, self.detection_callback, queue_size=1)
        # Add service proxies corresponding to vision.py services
        # rospy.wait_for_service("/imageDetection")
        self.inference_state_srv = rospy.ServiceProxy(
            "/toggle_detection", imageDetection
        )
        self.detections = []

    def detection_callback(self, msg):
        self.detections = msg.detections
        
    def get_detections(self):
        return self.detections

    def start(self):
        self.inference_state_srv(True)  # Start the inference service
        print("Detection started")
    
    def stop(self):
        self.inference_state_srv(False)