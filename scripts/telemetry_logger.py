#!/usr/bin/env python

import rospy
import csv
import os
from datetime import datetime
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, NavSatFix
from geometry_msgs.msg import TwistStamped, PoseStamped
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion

class MavrosTelemetryLogger:
    def __init__(self):
        rospy.init_node('mavros_telemetry_logger', anonymous=True)
        
        # Parametre okumaları
        self.log_rate = rospy.get_param('~log_rate', 1.0)  # Default 1 Hz
        self.output_dir = rospy.get_param('~output_dir', os.path.expanduser('~/telemetry_logs'))
        
        # Dizin oluşturma
        if not os.path.exists(self.output_dir):
            os.makedirs(self.output_dir)
            
        # CSV dosyasını oluştur
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.csv_file_path = os.path.join(self.output_dir, f'telemetry_log_{timestamp}.csv')
        self.csv_file = open(self.csv_file_path, 'w')
        self.csv_writer = csv.writer(self.csv_file)
        
        # CSV başlık satırını yaz
        self.csv_writer.writerow([
            'timestamp',
            'latitude', 'longitude', 'altitude',
            'ground_speed',
            'roll', 'pitch', 'heading',
            'speed_setpoint_x', 'speed_setpoint_y', 'speed_setpoint_z',
            'attitude_setpoint_roll', 'attitude_setpoint_pitch', 'attitude_setpoint_yaw'
        ])
        
        # Veri değişkenleri
        self.latitude = 0.0
        self.longitude = 0.0
        self.altitude = 0.0
        self.ground_speed = 0.0
        self.roll = 0.0
        self.pitch = 0.0
        self.heading = 0.0
        self.speed_setpoint = [0.0, 0.0, 0.0]  # x, y, z
        self.attitude_setpoint = [0.0, 0.0, 0.0]  # roll, pitch, yaw
        
        # Topic'lere abone ol
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.global_position_callback)
        rospy.Subscriber('/mavros/global_position/raw/gps_vel', TwistStamped, self.velocity_callback)
        rospy.Subscriber('/mavros/imu/data', Imu, self.imu_callback)
        rospy.Subscriber('/mavros/global_position/compass_hdg', Float64, self.heading_callback)
        rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel', TwistStamped, self.velocity_setpoint_callback)
        rospy.Subscriber('/mavros/setpoint_attitude/attitude', PoseStamped, self.attitude_setpoint_callback)
        
        # Timer ile periyodik kayıt
        self.timer = rospy.Timer(rospy.Duration(1.0 / self.log_rate), self.log_data)
        
        rospy.loginfo(f"Telemetri verisi kaydediliyor: {self.csv_file_path}")
        rospy.on_shutdown(self.shutdown_handler)
    
    def global_position_callback(self, msg):
        self.latitude = msg.latitude
        self.longitude = msg.longitude
        self.altitude = msg.altitude
    
    def velocity_callback(self, msg):
        # Yer hızını hesapla (x, y bileşenlerinden)
        vx = msg.twist.linear.x
        vy = msg.twist.linear.y
        self.ground_speed = (vx**2 + vy**2)**0.5
    
    def imu_callback(self, msg):
        # Quaternion'ı Euler açılarına dönüştür
        quaternion = [
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        ]
        euler = euler_from_quaternion(quaternion)
        self.roll = euler[0]
        self.pitch = euler[1]
        # Yaw açısı buradan alınabilir, ancak heading için compass kullanıyoruz
    
    def heading_callback(self, msg):
        self.heading = msg.data
    
    def velocity_setpoint_callback(self, msg):
        self.speed_setpoint = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ]
    
    def attitude_setpoint_callback(self, msg):
        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w
        ]
        euler = euler_from_quaternion(quaternion)
        self.attitude_setpoint = [
            euler[0],  # roll
            euler[1],  # pitch
            euler[2]   # yaw
        ]
    
    def log_data(self, event):
        # Geçerli zamanı al
        timestamp = rospy.Time.now().to_sec()
        
        # CSV'ye veri satırını yaz
        self.csv_writer.writerow([
            timestamp,
            self.latitude, self.longitude, self.altitude,
            self.ground_speed,
            self.roll, self.pitch, self.heading,
            self.speed_setpoint[0], self.speed_setpoint[1], self.speed_setpoint[2],
            self.attitude_setpoint[0], self.attitude_setpoint[1], self.attitude_setpoint[2]
        ])
        
        # Dosyayı kaydet (bellek tamponundaki verileri diske yaz)
        self.csv_file.flush()
    
    def shutdown_handler(self):
        rospy.loginfo("Telemetri kaydı sonlandırılıyor...")
        if self.csv_file is not None:
            self.csv_file.close()
            rospy.loginfo(f"Telemetri kaydı tamamlandı: {self.csv_file_path}")

if __name__ == '__main__':
    try:
        logger = MavrosTelemetryLogger()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass