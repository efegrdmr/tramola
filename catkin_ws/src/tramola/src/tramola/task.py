import rospy
from tramola.vehicle import Vehicle
from tramola.msg import DetectionList, Detection
from tramola.srv import greenLightDetectionState, inferenceState, selectModel, yellowDetectionState

class Task:
    def __init__(self, vehicle):
        assert rospy.core.is_initialized(), "ROS node is not initialized"
        self.objects = {
            "green_buoy": 0, "red_buoy": 1, "yellow_buoy": 2, "blue_buoy": 3, "black_buoy": 4,
            "red_gate_buoy": 5, "green_gate_buoy": 6,
            "orange_vessel": 7, "black_vessel": 8, 
            "blue_triangle": 9, "red_triangle": 10, "green_triangle": 11, 
            "blue_circle": 12, "red_circle": 13, "green_circle": 14,
            "blue_square": 15, "red_square": 16, "green_square": 17,
            "blue_plus": 18, "red_plus": 19, "green_plus": 20
        }

        self.subscriptions = []
        self.publications = []
        self.services = []
        self.timers = []
        self.vehicle = vehicle
        self.add_subscription("yolo_detections", DetectionList, self.detection_callback)
        self.status = "STARTED"

        # Add service proxies corresponding to vision.py services
        rospy.wait_for_service("/inference_state")
        self.inference_state_srv = rospy.ServiceProxy(
            "/inference_state", inferenceState
        )
        rospy.wait_for_service("/select_model")
        self.select_model_srv = rospy.ServiceProxy(
            "/select_model", selectModel
        )

        self.select_model("/home/tramola/catkin_ws/src/tramola/models/balon.pt")
        self.set_inference_state(True)

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
        self.vehicle.set_mode("LOITER")
        self.status = "COMPLETED"

        self.set_inference_state(False)

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

    def select_model(self, model_path):
        """
        Select a YOLO model by path.
        """
        try:
            response = self.select_model_srv(model_path=model_path)
            rospy.loginfo(f"Model selected from: {model_path}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Failed to select model: {e}")
            return False
