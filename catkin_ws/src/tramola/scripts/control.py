import math
from tramola.loralib import LoRa
from tramola.vehicle import Vehicle
import rospy

from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float64
from geometry_msgs.msg import TwistStamped

class Control:
    def __init__(self):
        self.vehicle = Vehicle()
        self.manual_speed = 0
        self.manual_yaw = 0
        self.speed = 0
        self.state = ""
        self.lora = LoRa(self.lora_callback)

        rospy.init_node("control", anonymous=True)
        # Speed
        self.speed_sub = rospy.Subscriber("/mavros/global_position/gp_vel", TwistStamped, self.speed_callback)

    def gps_callback(self, data):
        self.latitude = data.latitude
        self.longitude = data.longitude

    def compass_callback(self, data):
        self.degree_from_north = data.data
    
    

    def lora_callback(self, data):
        pass

if __name__ == "__main__":
    
    