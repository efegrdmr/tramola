import rospy
from tramola.task import Task
from tramola.msg import DetectionList, Detection


class SpeedChallenge(Task):
    def start(self):
        self.vehicle.set_mode("GUIDED")
        self.objects = {"green": 0, "red": 1, "yellow": 2}


    def detection_callback(self, msg):
        pass
