#!/usr/bin/env python
# -*- coding: utf-8 -*-
import math
from tramola.loralib import Lora
from tramola.vehicle import Vehicle
from tramola.lidar import Lidar
from tramola.detection import Detection
from tramola.goTo import GoTo
from tramola.kamikaze import Kamikaze
from tramola.move_base_client import MoveBaseClient
import rospy
import time

class Control:
    def __init__(self):
        rospy.init_node("control", anonymous=True)
        self.vehicle = Vehicle()
        self.lidar = Lidar()
        self.detection = Detection()
        self.move_base = MoveBaseClient()
        self.lora = None
        self.task = None
        self.vehicle.set_mode("HOLD")
        self.vehicle.arming(True)
        self.points = []
        self.state = "IDLE"
        self.objective_color = None  # Initialize with None instead of -1 for clarity
        
        # Call mission_callback every 100ms
        rospy.Timer(rospy.Duration(0.1), self.mission_callback)
        self.init_lora()
    
    def init_lora(self):
        if self.lora:
            rospy.loginfo("Closing existing LoRa connection")
            self.lora.close()
            time.sleep(1)  # Reduced wait time from 3 seconds
        
        try:
            self.lora = Lora(message_callback=self.lora_callback, port="/dev/ttyACM0")
            self.lora.start_receiver()
            rospy.loginfo("LoRa initialized successfully")
        except Exception as e:
            rospy.logerr(f"Failed to initialize LoRa: {e}")

    # Checks the status of a given task and moves to the next one
    def mission_callback(self, t):
        if self.state == "GOTO":
            if not self.task:
                rospy.loginfo("GoTo mission started")
                if self.points:
                    self.task = GoTo(self.vehicle, self.lidar, self.move_base, self.points.pop(0))
                else:
                    rospy.logwarn("No waypoints available for GoTo mission")
                    self.state = "IDLE"
                    return

            if self.task.state == "COMPLETED":
                rospy.loginfo("GoTo mission completed")
                if len(self.points) == 0:
                    rospy.loginfo("KAMIKAZE mission started")
                    self.state = "KAMIKAZE"
                    self.task = Kamikaze(self.vehicle, self.lidar, self.detection)
                else:
                    self.task = None
                    self.state = "GOTO"
        
        elif self.state == "KAMIKAZE":
            if self.task and self.task.state == "COMPLETED":
                rospy.loginfo("KAMIKAZE mission completed")
                self.state = "IDLE"
                self.task = None

    def lora_callback(self, data):
        try:
            data = data.split(",")
            command = data[0]
            
            # Query commands
            if command == "speed_real":
                return str(self.vehicle.speed)
            elif command == "heading":
                return str(self.vehicle.heading)
            elif command == "yaw_real":
                return str(self.vehicle.yaw)
            elif command == "thruster_requested":
                return f"{self.vehicle.thrust_left:.6f},{self.vehicle.thrust_right:.6f}"
            elif command == "speed_requested":
                return str(self.vehicle.last_sent_linear_speed)
            elif command == "yaw_requested":
                return str(self.vehicle.last_sent_angular_speed)
            elif command == "location":
                return f"{self.vehicle.location[0]:.6f},{self.vehicle.location[1]:.6f}"
            elif command == "state":
                if self.state == "GOTO":
                    return f"GOTO,{self.move_base.get_state_num()}"
                return self.state
            
            # Control commands
            elif command == "start_mission":
                if self.state != "IDLE":
                    return "ERR"
                if len(self.points) == 0:
                    return "ERR"
                self.state = "GOTO"
                return "OK"
            
            elif command == "emergency_shutdown":
                rospy.logwarn("Emergency shutdown activated")
                if self.task:
                    self.task.stop()
                    self.task = None
                self.move_base.cancel_goal()
                self.vehicle.set_mode("HOLD")
                self.vehicle.arming(False)
                self.state = "IDLE"
                self.points = []
                return "OK"
            
            # Waypoint management
            elif command == "add_waypoint":
                try:
                    x, y = float(data[1]), float(data[2])
                    new_point = (x, y)
                    
                    # Only add if different from the last point
                    if not self.points or self.points[-1] != new_point:
                        self.points.append(new_point)
                        rospy.loginfo(f"Waypoint added: {new_point}")
                        return "OK"
                    else:
                        return "ERR"
                except ValueError:
                    return "ERR"
            
            # Manual control
            elif command == "start_manual_mode":
                if self.task:
                    self.task.stop()
                    self.task = None
                    self.points = []
                self.state = "MANUAL"
                self.move_base.cancel_goal()
                self.vehicle.start_rc_override()
                rospy.loginfo("Manual mode activated")
                return "OK"
            
            elif command == "stop_manual_mode":
                if self.state == "MANUAL":
                    self.vehicle.stop_rc_override()
                    self.state = "IDLE"
                    rospy.loginfo("Manual mode deactivated")
                return "OK"
            
            elif command == "manual":
                if self.state != "MANUAL":
                    return "ERR"

                try:
                    # Use first parameter for speed, second for yaw instead of left/right thrusters
                    speed = float(data[1])
                    yaw = float(data[2])
                    self.vehicle.set_rc_speed(speed)
                    self.vehicle.set_rc_yaw(yaw)
                    return "OK"
                except ValueError:
                    return "ERR"
            
            # Color objective setting
            elif command == "set_color":
                try:
                    color_code = int(data[1])
                    if color_code == 0:
                        self.objective_color = "RED"
                    elif color_code == 1:
                        self.objective_color = "GREEN"
                    elif color_code == 2:
                        self.objective_color = "BLACK"
                    else:
                        return "ERR"
                    
                    rospy.loginfo(f"Target color set to {self.objective_color}")
                    return "OK"
                except ValueError:
                    return "ERR"
            
            else:
                rospy.logwarn(f"Unknown command received: {command}")
                return "ERR"
                
        except Exception as e:
            rospy.logerr(f"Error in lora_callback: {e}")
            return "ERR"


if __name__ == "__main__":
    try:
        control = Control()
        rospy.loginfo("Control system initialized")
        rospy.spin()
    except rospy.ROSInterruptException:
        rospy.loginfo("Control system terminated")