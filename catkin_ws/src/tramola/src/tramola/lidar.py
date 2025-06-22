import math
import rospy
from sensor_msgs.msg import LaserScan
from std_srvs.srv import Empty


SCAN_DEGREE = math.radians(45) 
ANGLE_GAP = math.radians(5) 
DISTANCE_GAP  = 0.2 

class Cluster:
    """
    Represents a sequence of points forming a cluster.
    Tracks start/end beams, closest point, and horizontal distance.
    """
    def __init__(self, angle, distance):
        self.start_angle = angle
        self.start_dist = distance
        self.end_angle = angle
        self.end_dist = distance
        # horizontal component for min distance
        self.min_horiz_dist = distance * math.cos(angle)

    def add(self, angle, distance):
        # update end
        self.end_angle = angle
        self.end_dist = distance
        # update horizontal
        horizontal_dist = distance * math.cos(angle)
        if horizontal_dist < self.min_horiz_dist:
            self.min_horiz_dist = horizontal_dist

    def __str__(self):
        return (
            "Cluster(start: {0:.3f} m @ {1:.3f} rad, "
            "end: {2:.3f} m @ {3:.3f} rad, "
            "horiz: {4:.3f} m)".format(
                self.start_dist,
                self.start_angle,
                self.end_dist,
                self.end_angle,
                self.min_horiz_dist,
            )
        )

class Lidar:
    def __init__(self):    
        assert rospy.core.is_initialized(), "ROS node is not initialized"  
        self.scan_func = None
        rospy.wait_for_service("stop_motor")
        rospy.wait_for_service("start_motor")
        self.stop_lidar = rospy.ServiceProxy("stop_motor", Empty)
        self.stop_lidar()
        self.start_lidar = rospy.ServiceProxy("start_motor", Empty)
        rospy.Subscriber("/scan", LaserScan, self.scan_callback, queue_size=1)
        self.half_cone = SCAN_DEGREE / 2.0

    def scan_callback(self, scan_msg):
        ranges = scan_msg.ranges
        angle = scan_msg.angle_min
        prev_dist  = None
        first_valid = True
        clusters = []
        current = None
        prev_angle = None

        # Single-pass scan through beams
        for r in ranges:
            # compute angle
            # note: angle starts at angle_min and increments each loop
            if not (r > 0.0 and not math.isinf(r) and abs(angle) <= self.half_cone):
                angle += scan_msg.angle_increment
                continue

            if first_valid:
                # start first cluster
                current = Cluster(angle, r)
                first_valid = False
            else:
                # compute gaps
                ang_gap  = angle - prev_angle
                dist_gap = abs(r - prev_dist)

                # same cluster only if both gaps are small
                if ang_gap <= ANGLE_GAP and dist_gap <= DISTANCE_GAP:
                    current.add(angle, r)
                else:
                    clusters.append(current)
                    current = Cluster(angle, r)

            prev_angle = angle
            prev_dist = r
            angle += scan_msg.angle_increment

        # finalize last cluster
        if not first_valid and current is not None:
            clusters.append(current)
        elif first_valid:
            self.scan_func([])
            return

        self.scan_func(clusters)


