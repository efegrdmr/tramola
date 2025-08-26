#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped, Twist
from mavros_msgs.msg import AttitudeTarget  # Add this import for the likely message type
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class MavrosTelemetryLogger(object):
    def __init__(self):
        rospy.init_node('mavros_telemetry_logger', anonymous=True)

        # Parameters
        self.log_rate = rospy.get_param('~log_rate', 1.0)  # Hz
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/telemetry_logs'))

        # Ensure output directory exists
        if not os.path.exists(self.output_dir):
            try:
                os.makedirs(self.output_dir)
            except Exception as e:
                rospy.logerr("Cannot create output directory {}: {}".format(self.output_dir, e))
                rospy.signal_shutdown("Cannot create output directory")
                return

        # Create CSV file with timestamped name
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file_path = os.path.join(self.output_dir, 'telemetry_log_{}.csv'.format(timestamp))
        try:
            # Python2: open in binary mode for csv.writer
            self.csv_file = open(self.csv_file_path, 'wb')
        except Exception as e:
            rospy.logerr("Failed to open CSV file {}: {}".format(self.csv_file_path, e))
            rospy.signal_shutdown("Failed to open CSV file")
            return

        self.csv_writer = csv.writer(self.csv_file)

        # Write CSV header
        self.csv_writer.writerow([
            'timestamp',
            'latitude', 'longitude', 'altitude',
            'ground_speed',
            'roll', 'pitch', 'heading',
            'speed_setpoint_x', 'speed_setpoint_y', 'speed_setpoint_z',
            'attitude_setpoint_roll', 'attitude_setpoint_pitch', 'attitude_setpoint_yaw'
        ])
        self.csv_file.flush()

        # Data variables (defaults)
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.ground_speed = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.speed_setpoint = [0.0, 0.0, 0.0]
        self.attitude_setpoint = [0.0, 0.0, 0.0]

        # Subscribers
        rospy.Subscriber('/mavros/mavros/global_position/global', NavSatFix, self.global_position_callback)
        rospy.Subscriber('/mavros/mavros/global_position/raw/gps_vel', TwistStamped, self.velocity_callback)
        rospy.Subscriber('/mavros/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/mavros/mavros/global_position/compass_hdg', Float64, self.heading_callback)
        rospy.Subscriber('/mavros/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, self.velocity_setpoint_callback)
        
        # Updated to use one of your available attitude setpoint topics
        # Note: Adjust the message type according to what 'rostopic info' shows
        rospy.Subscriber('/mavros/mavros/setpoint_raw/target_attitude', AttitudeTarget, self.attitude_setpoint_callback)

        # Periodic logger timer
        self.timer = rospy.Timer(rospy.Duration(1.0 / float(self.log_rate)), self.log_data)

        rospy.loginfo("Telemetry logging to: {}".format(self.csv_file_path))
        rospy.on_shutdown(self.shutdown_handler)

    def global_position_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude

    def velocity_callback(self, msg):
        # Compute ground speed from vx, vy
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.ground_speed = (vx**2 + vy**2)**0.5

    def imu_callback(self, msg):
        # Convert quaternion to Euler angles
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        euler = euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        # Yaw is available here as euler[2], but we use compass heading topic

    def heading_callback(self, msg):
        self.heading = msg.data

    def velocity_setpoint_callback(self, msg):
        self.speed_setpoint = [
            msg.linear.x,
            msg.linear.y,
            msg.linear.z
        ]

    def attitude_setpoint_callback(self, msg):
        # Updated for AttitudeTarget message type
        # If this is not the correct message type, adjust accordingly
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        euler = euler_from_quaternion(quaternion)
        self.attitude_setpoint = [
            euler[0],  # roll
            euler[1],  # pitch
            euler[2]   # yaw
        ]

    def log_data(self, event):
        # Current time in seconds
        timestamp = rospy.Time.now().to_sec()

        # Write row to CSV
        self.csv_writer.writerow([
            timestamp,
            self.latitude, self.longitude, self.altitude,
            self.ground_speed,
            self.roll, self.pitch, self.heading,
            self.speed_setpoint[0], self.speed_setpoint[1], self.speed_setpoint[2],
            self.attitude_setpoint[0], self.attitude_setpoint[1], self.attitude_setpoint[2]
        ])

        # Flush buffer to disk
        try:
            self.csv_file.flush()
            # os.fsync(self.csv_file.fileno())  # optional stronger durability
        except Exception:
            pass

    def shutdown_handler(self):
        rospy.loginfo("Shutting down telemetry logger...")
        try:
            if getattr(self, 'csv_file', None) is not None:
                self.csv_file.close()
                rospy.loginfo("Telemetry saved to: {}".format(self.csv_file_path))
        except Exception:
            pass

if __name__ == '__main__':
    try:
        logger = MavrosTelemetryLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass