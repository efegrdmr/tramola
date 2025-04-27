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

        if command == "state":
            self.lora.send_messaage(self.state)
        elif command == "latitude":
            self.lora.send_message(self.vehicle.location.latitude)     
        elif command == "longitude":
            self.lora.send_message(self.vehicle.location.longitude)
        elif command == "degree_from_north":
            self.lora.send_message(self.vehicle.orientation)
        elif command == "speed":
            self.lora.send_message(self.vehicle.linear_speed)
        elif command == "start_mission":
            self.vehicle.arming(True)
            self.vehicle.follow_waypoints()
            self.state = "Task 1"
        elif command == "start_manual_control":
            self.state = "Manual control"
            self.vehicle.start_manual_control()
        elif command == "emergency_shutdown":
            self.state = "Emergency shutdown"
            self.vehicle.emergency_stop()
        elif command == "add_waypoint" and len(data) == 3:
            lat, long = data[1], data[2]
            self.vehicle.add_waypoint(lat, long)
        elif command == "manual_control" and len(data) == 3:
            speed, yaw = data[1], data[2]
            self.vehicle.set_velocity(speed, yaw)
        else:
            rospy.logwarn("Unknown command received: %s" % command)


if __name__ == "__main__":
    control = Control()
