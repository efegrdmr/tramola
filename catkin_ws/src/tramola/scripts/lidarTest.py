from tramola.lidar import Lidar 
from tramola.vehicle import Vehicle

import rospy

class LidarTest:
    def __init__(self):
        rospy.init_node("lidarTest", anonymous=True)
        self.lidar = Lidar()
        self.lidar.scan_func = self.scan
        self.lidar.start_lidar()
        self.vehicle = Vehicle()

    def scan(self, clusters):
        for c in clusters:
            rospy.loginfo(c)
        

        # YDYO
        rospy.loginfo("distance between point: " + self.vehicle.angle_between(39.86518558806454, 32.73531196027693))







LidarTest()
