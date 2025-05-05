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
        self.state = "Idle"
        self.speed_real = 0
        self.heading_real = 0
        self.yaw_real = 0
        self.thruster_requested = 0
        self.speed_requested = 0
        self.heading_requested = 0
        self.yaw_requested = 0

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
            return self.yaw_real
        elif command == "thruster_requested":
            return self.thruster_requested
        elif command == "speed_requested":
            return self.speed_requested
        elif command == "heading":
            return self.heading_requested
        elif command == "yaw_requested":
            return self.yaw_requested
        elif command == "add_waypoint":
            pass
        elif command == "location":
            return "%f,%f" % (self.vehicle.location.latitude, self.vehicle.location.longitude)
        elif command == "start_mission":
            self.state = "Following waypoints"
            self.vehicle.set_mode("AUTO")

            self.vehicle.arming(True)

            
        elif command == "emergency_shutdown":
            pass
        elif command == "start_manual_control":
            pass
        elif command == "manual_control":
            pass
        elif command == "add_waypoint":
            pass
        else:
            rospy.logwarn("Unknown command received: %s" % command)


if __name__ == "__main__":
    control = Control()
