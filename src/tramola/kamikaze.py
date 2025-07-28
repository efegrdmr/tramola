import time
from tramola.task import Task


class Kamikaze(Task):
    def __init__(self, vehicle, lidar, detection, max_linear_speed=0.5, max_angular_speed=1.0, completion_threshold=1.0, safety_padding=5.0):
        super(Kamikaze, self).__init__(vehicle, lidar, detection=None)
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed
        self.completion_threshold = completion_threshold
        self.safety_padding = safety_padding  # degrees of padding for free angle edges
        self.vehicle.linear_speed = self.max_linear_speed

    def mission_callback(self, t):
        self.state = "COMPLETED"