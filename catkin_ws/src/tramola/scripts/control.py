#!/usr/bin/env python
import math
from tramola.loralib import LoRa
from tramola.vehicle import Vehicle
from tramola.task2 import Task2
from tramola.task3 import Task3
import rospy


class Control:
    def __init__(self):
        self.vehicle = Vehicle()
        self.lora = LoRa(self.lora_callback)
        rospy.init_node("control", anonymous=True)
        self.task = None
        self.set_mode("HOLD")
        self.vehicle.arming(False)
        self.points = []
        self.state = "idle"
      

    # checks the status of a given task and get to the next one
    def mission_callback(self):
        if self.state == "Following waypoints":
            if self.vehicle.waypoints_reached:
                self.state = "Task 2"
                self.task = Task2()
        if self.state == "Task 2":
            if self.task.status == "COMPLETED":
                self.state = "Task 3"
                self.task = Task3()
        if self.state == "Task 3":
            if self.task.status == "COMPLETED":
                self.state = "IDLE"
                self.vehicle.set_mode("HOLD")
                self.vehicle.arming(False)
            

    def lora_callback(self, data):
        data = data.split(",")
        command = data[0]

        if command == "speed_real":
            return self.vehicle.speed
        elif command == "heading":
            return self.vehicle.heading
        elif command == "yaw_real":
            return self.vehicle.yaw
        elif command == "thruster_requested":
            return "%f,%f" % (self.vehicle.thrust_left,self.vehicle.thrust_right)
        elif command == "speed_requested":
            return self.vehicle.last_sent_linear_speed
        elif command == "yaw_requested":
            return self.vehicle.last_sent_angular_speed
        elif command == "location":
            return "%f,%f" % (self.vehicle.location.latitude, self.vehicle.location.longitude)
        elif command == "start_mission":
            if self.state != "idle":
                return "ERR"
            self.state = "Following waypoints"
            if len(self.vehicle.waypoints) != 5: # video için değişecek
                return "ERR"
            self.vehicle.set_mode("AUTO")

            self.vehicle.arming(True)
            self.vehicle.follow_waypoints()
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
