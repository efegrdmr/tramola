import rospy
from std_srvs.srv import Empty

class Lidar:
    def __init__(self):    
        assert rospy.core.is_initialized(), "ROS node is not initialized"  
        #rospy.wait_for_service("/start_motor")
        self.start = rospy.ServiceProxy("/start_motor", Empty)

        #rospy.wait_for_service("/stop_motor")
        self.stop = rospy.ServiceProxy("/stop_motor", Empty)


