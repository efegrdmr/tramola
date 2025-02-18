
import rospy

from geometry_msgs.msg import Twist
from mavros_msgs.srv import SetMode, SetModeRequest
from std_msgs.msg import Float64
from sensor_msgs.msg import NavSatFix, Range
from geographic_msgs.msg import GeoPoseStamped



class Vehicle:
    def __init__(self):
        assert rospy.core.is_initialized(), "ROS node is not initialized"  

        # Velocity publisher 
        self.velocity_pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=1)
        self.linear_speed = 0.0 # max 1.0
        self.angular_speed = 0.0 # max 1.0
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_speed)  # 10 Hz

        # Set mode service
        rospy.wait_for_service("/mavros/set_mode") 
        self.mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
        self.current_mode = None

        # Compass
        self.compass_sub = rospy.Subscriber("/mavros/global_position/compass_hdg", Float64, self.compass_callback)
        self.orientation = None

        # GPS
        gps_sub = rospy.Subscriber("/mavros/global_position/global", NavSatFix, self.gps_callback)
        self.location = None

        # location sending
        self.location_pub = rospy.Publisher("/mavros/global_position/global", GeoPoseStamped, queue_size=1)


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

    def publish_speed(self, event):
        cmd = Twist()
        cmd.linear.x = self.linear_speed
        cmd.angular.z = self.angular_speed
        self.velocity_pub.publish(cmd)


    def compass_callback(self, msg):
        # 0 and 360 North 90 East 180 South 270 West
        self.orientation = msg.data

    def gps_callback(self, msg):
        self.location = (msg.latitude, msg.longitude)

    def turn_degrees(self, degrees, angular_speed=0.5):
        #TODO 
        # clockwise is positive
        target_orientation = self.orientation + degrees
        if degrees < 0:
            self.turn_left(angular_speed)
        else:
            self.turn_right(angular_speed)

        while abs(self.orientation - target_orientation) > 10:
            rospy.sleep(0.1)
        
        self.stop()

    def turn_left(self, angular_speed=0.5):
        self.linear_speed = 0.0
        self.angular_speed = angular_speed

    def turn_right(self, angular_speed=0.5):
        self.linear_speed = 0.0
        self.angular_speed = -angular_speed

    def go_straight(self, speed=0.4):
        self.linear_speed = speed

    def go_left(self, speed=0.3, angular_speed=0.3):
        self.linear_speed = speed
        self.angular_speed = angular_speed

    def go_right(self, speed=0.3, angular_speed=0.3):
        self.linear_speed = speed
        self.angular_speed = -angular_speed
        

    def stop(self):
        self.linear_speed = 0.0
        self.angular_speed = 0.0

    def __del__(self):
        self.timer.shutdown()
        self.stop()
        self.velocity_pub.unregister()
        self.mode_srv.shutdown()
        self.compass_sub.unregister()
        self.gps_sub.unregister()
        