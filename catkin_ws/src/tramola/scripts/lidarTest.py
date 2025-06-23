#!/usr/bin/env python

from tramola.lidar import Lidar 
from tramola.vehicle import Vehicle
from tramola.rviz_drawer import RvizDrawer

import rospy

class LidarTest:
    def __init__(self):
        rospy.init_node("lidarTest", anonymous=True)
        self.lidar = Lidar()
        self.lidar.scan_func = self.scan
        self.lidar.start_lidar()
        self.vehicle = Vehicle()
        self.drawer = RvizDrawer()

    def scan(self, clusters):
        print("\033c")
        for c in clusters:
            self.drawer.draw_line(
                c.start_angle, c.start_dist,
                c.end_angle, c.end_dist,
                0.1, (0.0, 0.0, 1.0)
            )
        

        # YDYO
        # rospy.loginfo("distance between point: " + self.vehicle.angle_between(39.86518558806454, 32.73531196027693))







LidarTest()
rospy.spin()
