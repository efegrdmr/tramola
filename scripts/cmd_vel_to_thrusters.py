#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32
from mavros_msgs.msg import OverrideRCIn

# --- Global Variables ---
pub_l = None
pub_r = None
last_rc_time = None
last_cmd_vel_time = None
CONTROL_TIMEOUT = rospy.Duration(0.5) # Seconds before stopping if no command is received

# --- Configuration Parameters ---
# You can adjust these values using rosparam or by changing the defaults here
MAX_LINEAR_VELOCITY = 1.5  # m/s
MAX_ANGULAR_VELOCITY = 1.0 # rad/s
PWM_MIN = 1000.0
PWM_MAX = 2000.0
PWM_NEUTRAL = 1500.0
PWM_DEADBAND = 20

def map_pwm_to_velocity(pwm_val, is_angular):
    """ Maps a PWM value to a corresponding velocity. """
    # Apply deadband - return zero if within deadband range
    if abs(pwm_val - PWM_NEUTRAL) < PWM_DEADBAND:
        return 0.0

    # Determine the maximum velocity for this control type
    max_vel = MAX_ANGULAR_VELOCITY if is_angular else MAX_LINEAR_VELOCITY
    
    # Normalize PWM value to [-1, 1] range
    if pwm_val > PWM_NEUTRAL:
        # Map [PWM_NEUTRAL+DEADBAND, PWM_MAX] to [0, 1]
        normalized = (pwm_val - PWM_NEUTRAL) / (PWM_MAX - PWM_NEUTRAL)
    else:
        # Map [PWM_MIN, PWM_NEUTRAL-DEADBAND] to [-1, 0]
        normalized = (pwm_val - PWM_NEUTRAL) / (PWM_NEUTRAL - PWM_MIN)
    
    # Apply velocity scaling
    velocity = normalized * max_vel
    
    # For angular velocity (yaw), invert the value to match expected RC behavior:
    # - PWM < 1500: Turn left (negative angular velocity in ROS)
    # - PWM > 1500: Turn right (positive angular velocity in ROS)
    if is_angular:
        velocity = -velocity
        
    return velocity

def rc_override_callback(msg):
    """
    Callback for /mavros/rc/override.
    This has HIGHER priority. It converts RC PWM to thruster commands.
    """
    global last_rc_time
    last_rc_time = rospy.get_rostime()

    # We only care about channels 0 (speed) and 2 (yaw)
    if len(msg.channels) < 3 or msg.channels[0] == 0 or msg.channels[2] == 0:
        rospy.logwarn_throttle(5, "RC Override message received, but channels 0 or 2 are not active.")
        return

    rospy.loginfo_once("RC Override is ACTIVE. Taking control of thrusters.")

    L = rospy.get_param('~wheel_separation', 1.2)
    speed_pwm = msg.channels[0]
    yaw_pwm = msg.channels[2]

    v = map_pwm_to_velocity(speed_pwm, is_angular=False)
    omega = map_pwm_to_velocity(yaw_pwm, is_angular=True)

    left = v - omega * L / 2.0
    right = v + omega * L / 2.0

    pub_l.publish(Float32(left))
    pub_r.publish(Float32(right))

def cmd_vel_callback(msg):
    """
    Callback for /mavros/setpoint_velocity/cmd_vel_unstamped.
    This has LOWER priority. It is ignored if RC override is active.
    """
    global last_cmd_vel_time

    # Check if RC is active by looking at the timestamp
    rc_is_active = (last_rc_time is not None) and (rospy.get_rostime() - last_rc_time < CONTROL_TIMEOUT)

    if rc_is_active:
        rospy.loginfo_throttle(5, "Ignoring cmd_vel because RC override is active.")
        return

    # If we get here, RC is not active, so we can process this command
    rospy.loginfo_once("cmd_vel is ACTIVE. Taking control of thrusters.")
    last_cmd_vel_time = rospy.get_rostime()
    
    L = rospy.get_param('~wheel_separation', 1.2)
    v = msg.linear.x
    omega = msg.angular.z
    
    left = v - omega * L / 2.0
    right = v + omega * L / 2.0

    pub_l.publish(Float32(left))
    pub_r.publish(Float32(right))

def check_timeout(event):
    """
    Periodically checks if the active control source has timed out.
    If so, it stops the thrusters.
    """
    # Determine if any source is active
    rc_is_active = (last_rc_time is not None) and (rospy.get_rostime() - last_rc_time < CONTROL_TIMEOUT)
    cmd_vel_is_active = (last_cmd_vel_time is not None) and (rospy.get_rostime() - last_cmd_vel_time < CONTROL_TIMEOUT)
    
    # If neither is active, stop the thrusters
    if not rc_is_active and not cmd_vel_is_active:
        # Only publish stop command if the last known command has actually timed out
        # This prevents constantly publishing zeros
        last_known_time = last_rc_time if last_rc_time is not None else rospy.Time(0)
        if last_cmd_vel_time is not None and last_cmd_vel_time > last_known_time:
            last_known_time = last_cmd_vel_time
        
        if rospy.get_rostime() - last_known_time > CONTROL_TIMEOUT:
            pub_l.publish(Float32(0.0))
            pub_r.publish(Float32(0.0))


if __name__ == '__main__':
    rospy.init_node('thruster_controller_node')

    # Initialize last message times to prevent immediate timeout on startup
    now = rospy.get_rostime()
    last_rc_time = now
    last_cmd_vel_time = now

    # Publishers for the left and right thrusters
    pub_l = rospy.Publisher('/wamv/thrusters/left_thrust_cmd', Float32, queue_size=1)
    pub_r = rospy.Publisher('/wamv/thrusters/right_thrust_cmd', Float32, queue_size=1)

    # Subscribers
    rospy.Subscriber('/mavros/rc/override', OverrideRCIn, rc_override_callback, queue_size=1)
    rospy.Subscriber('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, cmd_vel_callback, queue_size=1)

    # Timer for the safety timeout check (runs at 10 Hz)
    rospy.Timer(rospy.Duration(0.1), check_timeout)

    rospy.loginfo("Thruster controller node started.")
    rospy.loginfo("Listening for RC Override (Priority) and cmd_vel (Fallback).")

    # Keep the node running
    rospy.spin()