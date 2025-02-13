
import rospy
from mavros_msgs.msg import OverrideRCIn

class Vehicle:
    def __init__(self):
        rospy.init_node('usv_controller', anonymous=True)
        self.rc_override_pub = rospy.Publisher('/mavros/rc/override', OverrideRCIn, queue_size=10)
        self.rc_msg = OverrideRCIn()
        self.steering_channel = 0  # Adjust based on your RC configuration
        self.throttle_channel = 2  # Adjust based on your RC configuration
        self.timer = rospy.Timer(rospy.Duration(0.1), self.publish_rc_override)  # 10 Hz
        self.MAX_PWM = 2000
        self.MIN_PWM = 1000

    def publish_rc_override(self, event):
        self.rc_override_pub.publish(self.rc_msg)

    def scale_pwm(self, value):
        return int(self.MIN_PWM + value * (self.MAX_PWM - self.MIN_PWM))

    def go_straight(self, throttle=0.2):
        self.rc_msg.channels[self.steering_channel] = self.scale_pwm(0.5)  # Neutral steering
        self.rc_msg.channels[self.throttle_channel] = self.scale_pwm(throttle)

    def go_left(self, throttle=0.2, steering=0.3):
        self.rc_msg.channels[self.steering_channel] = self.scale_pwm(steering)  # Left steering
        self.rc_msg.channels[self.throttle_channel] = self.scale_pwm(throttle)

    def go_right(self, throttle=0.2, steering=0.7):
        self.rc_msg.channels[self.steering_channel] = self.scale_pwm(steering)  # Right steering
        self.rc_msg.channels[self.throttle_channel] = self.scale_pwm(throttle)

    def stop(self):
        self.rc_msg.channels[self.steering_channel] = self.scale_pwm(0.5)  # Neutral steering
        self.rc_msg.channels[self.throttle_channel] = self.scale_pwm(0.5)  # Neutral throttle

    def __del__(self):
        self.timer.shutdown()
        self.stop()
        self.rc_override_pub.unregister()