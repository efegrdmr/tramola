
import rospy
import math

# ROS messages
from geometry_msgs.msg import Twist, TwistStamped
from mavros_msgs.msg   import Waypoint, OverrideRCIn, WaypointReached, ActuatorControl
from std_msgs.msg      import Float64
from sensor_msgs.msg   import NavSatFix
from geographic_msgs.msg import GeoPoseStamped

# MAVROS services
from mavros_msgs.srv   import SetMode, SetModeRequest, WaypointPush, CommandBool, CommandBoolRequest


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
        self.current_mode = None

        # Real heading
        self.compass_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.compass_callback)
        self.heading = None

        # GPS
        self.gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.location = None

        # Real speed
        self.speed_sub = rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.speed_callback)
        self.speed = 0.0
        self.yaw = 0.0


        # Arming
        self.arming_srv = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool)
        self.arming_srv.wait_for_service()

        # Get thrust
        rospy.Subscriber("/mavros/actuator_output", ActuatorControl, self.actuator_output_cb)
        self.thrust_left = 0
        self.thrust_right = 0
                                        
        
    def actuator_output_cb(self, msg):
        self.thrust_left =  msg.controls[1] # ?????????
        self.thrust_right = msg.controls[3]


    def start_velocity_publisher(self):
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_speed)  # 10 Hz

    def stop_velocity_publisher(self):
        if self.timer is not None:
            self.timer.shutdown()
            self.timer = None

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
        if self.current_mode == mode:
            return

        req = SetModeRequest()
        req.custom_mode = mode
        resp = self.mode_srv(req)
        if resp.mode_sent:
            self.current_mode = mode
            rospy.loginfo("Mode changed to %s successfully", mode)
        else:
            rospy.logwarn("Failed to change mode to %s", mode)
        
    def set_velocity(self, linear_speed, angular_speed):
        self.linear_speed = linear_speed
        self.angular_speed = angular_speed

    def publish_speed(self):
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
        Return angle between the vehicle and the point
        """
        
        # Convert degrees to radians
        lat1_rad = math.radians(self.location[0])
        lat2_rad = math.radians(lat)
        delta_lon_rad = math.radians(lon - self.location[1])

        x = math.sin(delta_lon_rad) * math.cos(lat2_rad)
        y = math.cos(lat1_rad) * math.sin(lat2_rad) - \
            math.sin(lat1_rad) * math.cos(lat2_rad) * math.cos(delta_lon_rad)

        angle_rad = math.atan2(x, y)
        bearing_deg = (math.degrees(angle_rad) + 360) % 360  # Normalize to 0-360 degrees

        return bearing_deg - self.heading

    def __del__(self):
        self.timer.shutdown()
        self.stop()
        self.velocity_pub.unregister()
        self.mode_srv.shutdown()
        self.compass_sub.unregister()
        self.gps_sub.unregister()
        self.speed_sub.unregister()
        self.rc_override_pub.unregister()
        self.arming_srv.shutdown()
        self.location_pub.unregister()
        