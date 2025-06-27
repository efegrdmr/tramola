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

    