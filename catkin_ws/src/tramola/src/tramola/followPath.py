from tramola.task import Task
import rospy


class FollowPath(Task):
    def __init__(self):
        super().__init__()

    def start(self):
        pass

    def detection_callback(self, msg):
        pass