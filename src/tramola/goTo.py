from tramola.task import Task
import rospy
class GoTo(Task):
    def __init__(self, vehicle, lidar, move_base, point):
        super(GoTo, self).__init__(vehicle, lidar, detection=None)
        self.point = point
        self.lidar.start()
        self.vehicle.set_mode("GUIDED")
        self.move_base = move_base
        angle = self.vehicle.angle_between(*point)
        self.move_base.send_goal(*point, angle)

    
    def mission_callback(self, t):
        if self.move_base.succeeded():
            self.stop()


