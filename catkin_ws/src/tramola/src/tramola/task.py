import rospy
from tramola.vehicle import Vehicle
from tramola.msg import DetectionList, Detection
from tramola.srv import imageDetection

class Task:
    def __init__(self, vehicle):
        assert rospy.core.is_initialized(), "ROS node is not initialized"
        
        self.subscriptions = []
        self.publications = []
        self.services = []
        self.timers = []
        self.vehicle = vehicle
        self.add_subscription("/detections", DetectionList, self.detection_callback)
        self.status = "STARTED"

        # Add service proxies corresponding to vision.py services
        rospy.wait_for_service("/imageDetection")
        self.inference_state_srv = rospy.ServiceProxy(
            "/imageDetection", imageDetection
        )

        # Subscribe to Lidar 
    
        self.start()
    
    def add_timer(self, period, callback):
        self.timers.append(rospy.Timer(rospy.Duration(period), callback))
    
    def add_subscription(self, topic, msg_type, callback):
        self.subscriptions.append(rospy.Subscriber(topic, msg_type, callback))

    def add_publication(self, topic, msg_type):
        publisher = rospy.Publisher(topic, msg_type, queue_size=10)
        self.publications.append(publisher)
        return publisher
    
    def add_service(self, srv_name, srv_type, handler):
        self.services.append(rospy.Service(srv_name, srv_type, handler))

    def start(self):
        raise NotImplementedError("The start method must be overridden in a subclass")

    def detection_callback(self, msg):
        raise NotImplementedError("The detection_callback method must be overridden in a subclass")

    def stop(self):
        if self.status == "COMPLETED":
            return
        for sub in self.subscriptions:
            sub.unregister()
        for pub in self.publications:
            pub.unregister()
        for srv in self.services:
            srv.shutdown()
        for timer in self.timers:
            timer.shutdown()
        self.status = "COMPLETED"

    # Functions wrapping the service proxies

    def set_green_light_detection(self, on):
        """
        Enable or disable green light detection.
        """
        try:
            response = self.green_light_detection_srv(on=on)
            rospy.loginfo(f"Green light detection state set to {on}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set green light detection state: {e}")
            return False

    def set_inference_state(self, on):
        """
        Enable or disable object inference.
        """
        try:
            response = self.inference_state_srv(on=on)
            rospy.loginfo(f"Inference state set to {on}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set inference state: {e}")
            return False

    def set_yellow_detection_state(self, on):
        """
        Enable or disable yellow object detection.
        """
        try:
            response = self.yellow_detection_state_srv(on=on)
            rospy.loginfo(f"Yellow detection state set to {on}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to set yellow detection state: {e}")
            return False


