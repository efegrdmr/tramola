#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
import csv
import os
import math
from datetime import datetime
from sensor_msgs.msg import NavSatFix, Imu
from geometry_msgs.msg import TwistStamped, Twist
from std_msgs.msg import Float64

class FlightDataLogger:
    def __init__(self):
        rospy.init_node('flight_data_logger', anonymous=True)
        
        # Set the logging rate (Hz)
        self.log_rate = rospy.get_param('~log_rate', 1.0)
        
        # Initialize data variables
        self.position = [0.0, 0.0, 0.0]  # lat, lon, alt
        self.ground_speed = [0.0, 0.0, 0.0]  # x, y, z
        self.attitude = [0.0, 0.0]  # roll, pitch
        self.heading = 0.0  # heading (degrees)
        self.speed_setpoint = [0.0, 0.0, 0.0]  # x, y, z
        self.attitude_setpoint = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        self.heading_setpoint = 0.0  # Calculated heading setpoint
        self.last_update_time = rospy.Time.now()
        
        # Parameters for heading prediction
        self.prediction_time = rospy.get_param('~prediction_time', 3.0)  # seconds
        self.min_yaw_rate = rospy.get_param('~min_yaw_rate', 0.01)  # deg/s
        
        # Create data file with timestamp in filename
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_dir = os.path.expanduser("~") + "/flight_logs/"
        
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        self.log_file = self.log_dir + timestamp + "_flight_data.csv"
        
        # Initialize CSV file with headers
        with open(self.log_file, 'w') as f:
            writer = csv.writer(f)
            writer.writerow([
                'timestamp',
                'latitude', 'longitude', 'altitude',
                'ground_speed_x', 'ground_speed_y', 'ground_speed_z',
                'roll', 'pitch', 'heading',
                'speed_setpoint_x', 'speed_setpoint_y', 'speed_setpoint_z',
                'roll_setpoint', 'pitch_setpoint', 'yaw_setpoint',
                'heading_setpoint'
            ])
        
        # Set up subscribers
        rospy.Subscriber('/mavros/mavros/global_position/global', NavSatFix, self.position_callback)
        rospy.Subscriber('/mavros/mavros/global_position/raw/gps_vel', TwistStamped, self.ground_speed_callback)
        rospy.Subscriber('/mavros/mavros/imu/data', Imu, self.attitude_callback)
        rospy.Subscriber('/mavros/mavros/global_position/compass_hdg', Float64, self.heading_callback)
        rospy.Subscriber('/mavros/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, self.velocity_setpoint_callback)
        
        # Set up timer for logging
        rospy.Timer(rospy.Duration(1.0/self.log_rate), self.log_data)
        
        rospy.loginfo("Flight data logger initialized. Logging to: " + self.log_file)
    
    def position_callback(self, msg):
        self.position = [msg.latitude, msg.longitude, msg.altitude]
    
    def ground_speed_callback(self, msg):
        self.ground_speed = [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z]
    
    def attitude_callback(self, msg):
        # Extract roll and pitch from quaternion
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        roll, pitch, _ = self.euler_from_quaternion(quaternion)
        self.attitude = [math.degrees(roll), math.degrees(pitch)]
    
    def heading_callback(self, msg):
        self.heading = msg.data
    
    def velocity_setpoint_callback(self, msg):
        # Store speed setpoints
        self.speed_setpoint = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z
        ]
        
        # Store yaw rate setpoint (angular.z) in attitude_setpoint[2]
        yaw_setpoint_rad = msg.angular.z
        self.attitude_setpoint[2] = yaw_setpoint_rad
        
        # Convert yaw rate from rad/s to deg/s for heading calculations
        yaw_rate_deg = math.degrees(yaw_setpoint_rad)
        
        # CASE 1: If there's a meaningful velocity vector, use its direction for heading
        if abs(msg.linear.x) > 0.01 or abs(msg.linear.y) > 0.01:
            heading_rad = math.atan2(msg.linear.y, msg.linear.x)
            heading_deg = (math.degrees(heading_rad) + 90) % 360
            
            self.heading_setpoint = heading_deg
            rospy.loginfo("Velocity-based heading setpoint: %.2f degrees" % heading_deg)
            
        # CASE 2: If no velocity but angular.z command exists, predict heading based on yaw rate
        elif abs(yaw_rate_deg) > self.min_yaw_rate:
            # Calculate target heading by adding the projected yaw change to current heading
            heading_change = yaw_rate_deg * self.prediction_time
            target_heading = (self.heading + heading_change) % 360
            
            self.heading_setpoint = target_heading
            rospy.loginfo("ZERO VELOCITY - Using angular.z for heading prediction")
            rospy.loginfo("Current heading: %.2f, Yaw rate: %.2f deg/s, Target heading: %.2f" % 
                         (self.heading, yaw_rate_deg, target_heading))
            
        # CASE 3: No meaningful velocity or yaw commands, maintain current heading
        else:
            self.heading_setpoint = self.heading
            rospy.loginfo("No movement commands - Maintaining current heading: %.2f" % self.heading)

    def euler_from_quaternion(self, quaternion):
        x, y, z, w = quaternion
        
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def log_data(self, event):
        current_time = rospy.Time.now().to_sec()
        
        with open(self.log_file, 'a') as f:
            writer = csv.writer(f)
            writer.writerow([
                current_time,
                self.position[0], self.position[1], self.position[2],
                self.ground_speed[0], self.ground_speed[1], self.ground_speed[2],
                self.attitude[0], self.attitude[1], self.heading,
                self.speed_setpoint[0], self.speed_setpoint[1], self.speed_setpoint[2],
                self.attitude_setpoint[0], self.attitude_setpoint[1], self.attitude_setpoint[2],
                self.heading_setpoint
            ])
    
    def run(self):
        rospy.spin()

if __name__ == '__main__':
    try:
        logger = FlightDataLogger()
        logger.run()
    except rospy.ROSInterruptException:
        pass