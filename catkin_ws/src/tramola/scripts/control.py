#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
from tramola.loralib import Lora
from tramola.vehicle import Vehicle
from tramola.lidar import Lidar
from tramola.detection import Detection
from tramola.goTo import GoTo
from tramola.kamikaze import Kamikaze
import rospy
import time

class Control:
    def __init__(self):
        rospy.init_node("control", anonymous=True)
        self.vehicle = Vehicle()
        self.lidar = Lidar()
        self.detection = Detection()
        self.lora = None
        self.init_lora()
        self.task = None
        self.vehicle.set_mode("HOLD")
        self.vehicle.arming(False)
        self.points = []
        self.state = "IDLE"
        self.last_gcs_message_time = time.time()
        
        rospy.Timer(rospy.Duration(0.1), self.mission_callback)  # Call mission_callback every 100ms
        self.init_lora()
    
    def init_lora(self):
        if self.lora:
            self.lora.close()
            time.sleep(3)
        self.lora = Lora(message_callback=self.lora_callback, port="/dev/ttyACM0")
        self.lora.start_receiver()
        self.last_gcs_message_time = time.time()
        

    # checks the status of a given task and get to the next one
    def mission_callback(self, t):
        # Change mode to the guided if there is a task
        if self.state != "IDLE":
            self.vehicle.set_mode("GUIDED")


        if self.state == "GOTO":
            if not self.task:
                rospy.loginfo("GoTo mission Started")
                self.vehicle.arming(True)
                self.task = GoTo(self.vehicle, self.lidar, self.points.pop(0))
                self.lidar.start()
                self.vehicle.set_mode("GUIDED")
                self.vehicle.start_velocity_publisher()

            if self.task.state == "COMPLETED":
                rospy.loginfo("GoTo mission completed")
                if len(self.points) == 0:
                    rospy.loginfo("KAMIKAZE mission Started")
                    self.state = "KAMIKAZE"
                    self.task = Kamikaze(self.vehicle, self.lidar, self.detection)
                else:
                    rospy.loginfo("GoTo mission Started")
                    self.task = GoTo(self.vehicle, self.lidar, self.points.pop(0))
        elif self.state == "KAMIKAZE":
            if self.task.state == "COMPLETED":
                self.task.stop()
                self.vehicle.set_mode("HOLD")
                self.vehicle.stop_velocity_publisher()
                rospy.loginfo("KAMIKAZE mission completed")
                self.state = "IDLE"
                self.task = None


            

    def lora_callback(self, data):
        self.last_gcs_message_time = time.time()
        data = data.split(",")
        command = data[0]
        if command == "speed_real":
            return str(self.vehicle.speed)
        elif command == "heading":
            return str(self.vehicle.heading)
        elif command == "yaw_real":
            return str(self.vehicle.yaw)
        elif command == "thruster_requested":
            return "%f,%f" % (self.vehicle.thrust_left,self.vehicle.thrust_right)
        elif command == "speed_requested":
            return str(self.vehicle.last_sent_linear_speed)
        elif command == "yaw_requested":
            return str(self.vehicle.last_sent_angular_speed)
        elif command == "location":
            return "%f,%f" % (self.vehicle.location[0], self.vehicle.location[1])
        elif command == "start_mission":
            if self.state != "IDLE":
                return "ERR"
            if len(self.points) == 0:
                return "ERR"
            self.state = "GOTO"
            return "OK"

        elif command == "emergency_shutdown":
            rospy.logwarn("Emergency shutdown")
            self.vehicle.set_mode("HOLD")
            self.vehicle.arming(False)
            return "OK"
        elif command == "add_waypoint":
            if len(self.points) == 0 or self.points[len(self.points) - 1] != (data[1], data[2]):
                self.points.append((float(data[1]), float(data[2])))
                rospy.logwarn("waypoint added")
                return "OK"
            else:
                return "ERR" 
                
        else:
            rospy.logwarn("Unknown command received: %s" % command)
            return "ERR"


if __name__ == "__main__":
    control = Control()
    rospy.spin()
