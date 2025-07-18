# -*- coding: utf-8 -*-
import rospy
import math

# ROS messages
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg   import RCOut
from std_msgs.msg      import Float64
from sensor_msgs.msg   import NavSatFix
from geographic_msgs.msg import GeoPoseStamped

# MAVROS services
from mavros_msgs.srv   import SetMode, SetModeRequest, CommandBool, CommandBoolRequest

class Vehicle:
    def __init__(self):
        assert rospy.core.is_initialized(), "ROS node is not initialized"  

        # Velocity publisher 
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        self.linear_speed = 0.0 # max 1.0
        self.angular_speed = 0.0 # max 1.0
        self.last_sent_linear_speed = 0
        self.last_sent_angular_speed = 0
        
        # Set mode service
        rospy.wait_for_service("/mavros/set_mode") 
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)

        # Real heading
        self.compass_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.compass_callback)
        self.heading = None

        # GPS
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.location = None

        # Real speed
        self.speed_sub = rospy.Subscriber("/mavros/global_position/raw/gps_vel", TwistStamped, self.speed_callback)
        self.speed = 0.0
        self.yaw = 0.0


        # Arming
        self.arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arming_srv.wait_for_service()

        # Get thrust
        rospy.Subscriber("/mavros/rc/out", RCOut, self.rc_out_cb)
        self.thrust_left = 0
        self.thrust_right = 0


                                        
        
    def rc_out_cb(self, msg):
        self.thrust_left =  self.pwm_to_percentage(msg.channels[0])# ?????????
        self.thrust_right = self.pwm_to_percentage(msg.channels[2])

    def pwm_to_percentage(self, pwm_value):
        """
        Convert PWM value to percentage.
        Assuming PWM range is 1000-2000, where 1000 is -100% and 2000 is 100%.
        """
        # Map 1000-2000 to -100 to 100
        return (pwm_value - 1500) / 5
        

    def start_velocity_publisher(self):
        self.velocity_publisher_timer = rospy.Timer(rospy.Duration(0.1), self.publish_speed)  # 10 Hz

    def stop_velocity_publisher(self):
        if self.velocity_publisher_timer is not None:
            self.velocity_publisher_timer.shutdown()
            self.velocity_publisher_timer = None

    def emergency_stop(self):
        self.stop_rc_ovveride()
        self.set_mode("HOLD")
        self.arming(False)
        self.linear_speed = 0.0
        self.angular_speed = 0.0
        self.publish_speed()

    def arming(self, arm):
        req = CommandBoolRequest()
        req.value = arm
        self.arming_srv(req)

    def speed_callback(self, data):
        self.speed = math.sqrt(data.twist.linear.x**2 + data.twist.linear.y**2)


    def send_location(self, latitude, longitude):
        msg = GeoPoseStamped()
        msg.pose.position.latitude = latitude
        msg.pose.position.longitude = longitude
        self.location_pub.publish(msg)

    def set_mode(self, mode):
        req = SetModeRequest()
        req.custom_mode = mode
        resp = self.mode_srv(req)
        if resp.mode_sent:
            rospy.loginfo("Mode changed to %s successfully", mode)
        else:
            rospy.logwarn("Failed to change mode to %s", mode)
        
    def turn_inplace(self, angle):
        """
        Turn the vehicle in place by setting the angular speed.
        Positive angle turns left, negative angle turns right.
        """
        self.angular_speed = angle
        self.linear_speed = 0.1

    def publish_speed(self, t):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.angular_speed
        self.velocity_pub.publish(cmd)
        self.last_sent_linear_speed = self.linear_speed
        self.last_sent_angular_speed = self.angular_speed


    def compass_callback(self, msg):
        # 0 and 360 North 90 East 180 South 270 West
        self.heading = float(msg.data)

    def gps_callback(self, msg):
        self.location = (msg.latitude, msg.longitude)

    def angle_between(self, lat, lon):
        """
        Return the relative bearing from the vehicle's heading to the point.
        Negative → turn right; Positive → turn left.
        """
        # Convert degrees to radians
        a1 = math.radians(self.location[0])
        a2 = math.radians(lat)
        l = math.radians(lon - self.location[1])

        # compute true bearing 0–360°
        x = math.sin(l) * math.cos(a2)
        y = math.cos(a1) * math.sin(a2) - math.sin(a1) * math.cos(a2) * math.cos(l)
        bearing = (math.degrees(math.atan2(x, y)) + 360) % 360

        # compute relative: negative = turn right, positive = turn left
        rel = ((self.heading - bearing + 540) % 360) - 180
        return rel

    def distance(self, lat, lon):
        """
        Return the great-circle distance (in meters) between the vehicle and (lat, lon).
        radius is the Earth radius in meters (default 6371000m).
        """
        a1 = math.radians(self.location[0])
        a2 = math.radians(lat)
        d = a2 - a1
        l = math.radians(lon - self.location[1])

        a = math.sin(d/2)**2 + math.cos(a1) * math.cos(a2) * math.sin(l/2)**2
        c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))

        return 6371000 * c
    
    def reached(self, lat, lon, threshold=1.0):
        """
        Check if the vehicle is within a certain distance (threshold) from the target location.
        """
        dist = self.distance(lat, lon)
        return dist <= threshold

        