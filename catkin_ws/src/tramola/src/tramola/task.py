import rospy
from tramola.vehicle import Vehicle
from tramola.detection import Detection
from tramola.lidar import Lidar


class Task:
    def __init__(self, vehicle, lidar, detection=None):
        assert rospy.core.is_initialized(), "ROS node is not initialized"
        
        self.vehicle = vehicle
        self.status = "STARTED"
        self.lidar = lidar
        self.detection = detection

        self.mission_loop = rospy.Timer(rospy.Duration(0.1), self.mission_callback)  # Call mission_callback every 100ms
    

    def mission_callback(self, t):
        raise NotImplementedError("The mission_callback method must be overridden in a subclass")

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
        self.lidar.stop()
        self.mission_loop.shutdown()

    