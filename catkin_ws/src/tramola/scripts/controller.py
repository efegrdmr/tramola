#!/home/tramola/vision/bin/python3.8
import rospy
from mavros_msgs.msg import RCIn, State
from mavros_msgs.srv import SetMode
from tramola.followPathNew import FollowPath  
from tramola.dock import Dock
from tramola.speedChallenge import SpeedChallenge


current_mode = None
task = None
set_mode_srv = None

def state_callback(msg):
    global current_mode
    current_mode = msg.mode

def set_mode(mode):
    global set_mode_srv, current_mode
    if current_mode == mode:
        return
    if set_mode_srv is None:
        set_mode_srv = rospy.ServiceProxy("/mavros/set_mode", SetMode)
    try:
        resp = set_mode_srv(base_mode=0, custom_mode=mode)
        if resp.mode_sent:
            rospy.loginfo("Mode changed successfully")
    except rospy.ServiceException as e:
        rospy.logerr("Service call failed: %s", e)

def rc_callback(msg):
    global task
    ch5 = msg.channels[4]
    ch6 = msg.channels[5]
    ch7 = msg.channels[6]


    if ch6 < 1500:
        set_mode("MANUAL")
        if task is not None:
            task.stop()
            task = None
        return
    
    if task is not None:
        if task.status == "FOLLOWPATH":
            task = FollowPath()
            task.start()
        elif task.status == "DOCK":
            task = Dock()
            task.start()
        
        return
    
    # RC7 < 1500 ve rc5 < 1333 ise followpath
    # RC7 < 1500 ve rc5 > 1333 > 1666 ise dock
    # RC7 < 1500 ve rc5 > 1666 ise speed test

    # RC7 > 1500 ve rc5 < 1333 ise su atma top atma
    # RC7 > 1500 ve rc5 > 1333 > 1666 ise ???
    # RC7 > 1500 ve rc5 > 1666 ise ???


    if ch7 < 1500 and ch5 < 1333:
        task = FollowPath()
        task.start()
    elif ch7 < 1500 and ch5 > 1333 and ch5 < 1666:
        task = Dock()
        task.start()
    elif ch7 < 1500 and ch5 > 1666:
        task = SpeedChallenge()
        task.start()
        
    

def main():
    rospy.init_node("controller", anonymous=True)
    rospy.Subscriber("/mavros/rc/in", RCIn, rc_callback)
    rospy.Subscriber("/mavros/state", State, state_callback)

    rospy.spin()

if __name__ == "__main__":
    main()
