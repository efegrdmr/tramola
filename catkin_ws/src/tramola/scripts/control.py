#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
from tramola.loralib import Lora
from tramola.vehicle import Vehicle
from tramola.goTo import GoTo
from tramola.task3 import Task3
import rospy


class Control:
    def __init__(self):
        rospy.init_node("control", anonymous=True)
        self.vehicle = Vehicle()
        self.lora = Lora(message_callback=self.lora_callback)
        self.task = None
        self.vehicle.set_mode("HOLD")
        self.vehicle.arming(False)
        self.points = []
        self.state = "IDLE"
        self.lora.start_receiver()

      

    # checks the status of a given task and get to the next one
    def mission_callback(self):
        if self.state == "GOTO":
            if not self.task:
                self.vehicle.arming(True)
                self.vehicle.set_mode("AUTO")
                self.task = GoTo(self.points.pop(0))
            if self.task.status == "COMPLETED":
                if len(self.points) == 0:
                    self.state = "KAMIKAZE"
                    self.task.stop()
                    self.task = Task3()
                else:
                    self.task.stop()
                    self.task = self.points.pop(0)
            
        elif self.state == "KAMIKAZE":
            if self.task.status == "COMPLETED":
                self.task.stop()
                self.vehicle.set_mode = "HOLD"
            

    def lora_callback(self, data):
        data = data.split(",")
        command = data[0]
        print("message came: " + str(data)) 
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
            self.vehicle.set_mode("HOLD")
            self.vehicle.arming(False)
            return "OK"
        elif command == "add_waypoint":
            last_added = self.points[len(self.points) - 1]
            if last_added != (data[1], data[2]):
                self.points.append((data[1], data[2]))

            return "OK"
            
        else:
            rospy.logwarn("Unknown command received: %s" % command)
            return "ERR"


if __name__ == "__main__":
    control = Control()
    rospy.spin()
