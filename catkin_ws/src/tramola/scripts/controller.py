#!/usr/bin/env python
import rospy
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import SetMode
import subprocess

script_process = None
current_mode = None

def state_callback(msg):
    global current_mode
    current_mode = msg.mode

def set_mode(mode):
    if current_mode == mode:
        return
    set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    try:
        resp = set_mode_srv(base_mode=0, custom_mode=mode)
        if resp.mode_sent:
            rospy.loginfo("Mode changed successfully")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def rc_callback(msg):
    global script_process
    ch6 = msg.channels[5] if len(msg.channels) > 5 else 0
    ch7 = msg.channels[6] if len(msg.channels) > 6 else 0

    if ch7 < 1300:
        set_mode("HOLD")
        if script_process is not None:
            script_process.kill()
            script_process = None
        return
    elif ch7 < 1700:
        set_mode("MANUAL")
        if script_process is not None:
            script_process.kill()
            script_process = None
        return
    
    # start the desired script
    set_mode("MANUAL")

    if ch6 < 1300:
        if script_process is None:
            script_process = subprocess.Popen(["rosrun", "your_package", "your_script.py"])
    elif ch6 < 1700:
        if script_process is None:
            script_process = subprocess.Popen(["rosrun", "your_package", "your_script.py"])

        
    

def main():
    rospy.init_node("mode_switcher", anonymous=True)
    rospy.Subscriber("/mavros/rc/in", RCIn, rc_callback)
    rospy.Subscriber("/mavros/state", State, state_callback)
    rospy.spin()

if __name__ == "__main__":
    main()
