#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import PoseStamped, Quaternion
from tf.transformations import quaternion_from_euler
from pyproj import Proj, transform
import tf2_ros
import tf2_geometry_msgs
from actionlib_msgs.msg import GoalStatusArray, GoalID, GoalStatus

class MoveBaseClient(object):
    """
    A client to send GPS goals to the move_base node using ROS topics.
    This class converts latitude and longitude coordinates into a goal
    in the 'odom' frame and sends it to move_base via the /move_base_simple/goal topic.
    """

    def __init__(self, utm_zone=56, is_southern_hemisphere=True):
        """
        Initializes the client and all necessary tools for coordinate conversion.
        """
        # --- Publisher and Subscriber Setup ---
        self.goal_pub = rospy.Publisher('/move_base_simple/goal', PoseStamped, queue_size=1)
        self.cancel_pub = rospy.Publisher('/move_base/cancel', GoalID, queue_size=1)
        self.status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)
        rospy.loginfo("move_base topic publishers and subscribers initialized.")

        # --- UTM CONFIGURATION ---
        self.utm_zone = utm_zone
        self.is_southern_hemisphere = is_southern_hemisphere

        # --- pyproj and TF2 Setup ---
        self.proj_latlon = Proj(proj='latlong', datum='WGS84')
        self.proj_utm = Proj(
            proj='utm',
            zone=self.utm_zone,
            ellps='WGS84',
            south=self.is_southern_hemisphere
        )
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # --- Goal Status Tracking ---
        self.latest_status = None

    def status_callback(self, msg):
        """
        Callback function for the /move_base/status subscriber.
        """
        if msg.status_list:
            self.latest_status = msg.status_list[-1]
            # You can add more detailed status logging here if needed
            # rospy.loginfo("Received status: %s", self.latest_status.text)

    def _convert_latlon_to_odom_pose(self, latitude, longitude, yaw_rad):
        """
        Converts a lat/lon coordinate to a PoseStamped in the 'odom' frame.
        Returns None on failure.
        """
        # 1) Convert Lat/Lon to UTM
        try:
            easting, northing = transform(
                self.proj_latlon,
                self.proj_utm,
                longitude,
                latitude
            )
        except Exception as e:
            rospy.logerr("pyproj.transform failed: %s", e)
            return None
        rospy.loginfo("Converted GPS goal to UTM: E=%.3f, N=%.3f", easting, northing)

        # 2) Build a PoseStamped in the 'utm' frame
        utm_pose = PoseStamped()
        utm_pose.header.frame_id = 'utm'
        utm_pose.header.stamp = rospy.Time.now()
        utm_pose.pose.position.x = easting
        utm_pose.pose.position.y = northing
        q = quaternion_from_euler(0, 0, yaw_rad)
        utm_pose.pose.orientation = Quaternion(*q)

        # 3) Transform the pose from 'utm' to 'odom' frame
        target_frame = 'odom'
        try:
            odom_pose = self.tf_buffer.transform(
                utm_pose,
                target_frame,
                rospy.Duration(1.0)
            )
            rospy.loginfo("Transformed goal to 'odom' frame: x=%.3f y=%.3f",
                          odom_pose.pose.position.x,
                          odom_pose.pose.position.y)
            return odom_pose

        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to transform goal from 'utm' to 'odom': %s", e)
            rospy.logerr("Make sure a transform from 'utm' to 'odom' is being published.")
            return None

    def send_goal(self, latitude, longitude, yaw_degrees):
        """
        Converts a GPS coordinate and sends it as a goal to move_base.
        This is now an asynchronous call.

        Args:
            latitude (float): The latitude of the goal.
            longitude (float): The longitude of the goal.
            yaw_degrees (float): The final orientation of the robot in degrees.
        """
        yaw_rad = yaw_degrees * 3.14159 / 180.0
        odom_pose = self._convert_latlon_to_odom_pose(latitude, longitude, yaw_rad)

        if odom_pose is None:
            rospy.logerr("Could not convert GPS goal to odom pose. Aborting.")
            return

        rospy.loginfo("Sending goal to /move_base_simple/goal topic.")
        self.goal_pub.publish(odom_pose)

    def cancel_goal(self):
        """Cancels the current goal by publishing an empty GoalID."""
        rospy.loginfo("Cancelling all active goals.")
        # Publishing an empty GoalID message to /move_base/cancel cancels all current goals.
        cancel_msg = GoalID()
        self.cancel_pub.publish(cancel_msg)

    def get_state_text(self):
        """Returns a human-readable string of the current goal status."""
        if self.latest_status:
            return "Goal status: {} - {}".format(self.latest_status.status, self.latest_status.text)
        return "No status received yet."
        
    def get_state_num(self):
        if self.latest_status:
            return self.latest_status.status
        
        return -1

    def succeeded(self):
        """
        Checks if the last goal sent has succeeded.

        Returns:
            bool: True if the latest goal status is SUCCEEDED, False otherwise.
        """
        # GoalStatus.SUCCEEDED has an integer value of 3
        if self.latest_status and self.latest_status.status == GoalStatus.SUCCEEDED:
            return True
        return False

if __name__ == '__main__':
    """
    Main function to run the script as an interactive console application.
    It runs in a loop, allowing the user to repeatedly enter GPS goals.
    """
    try:
        rospy.init_node('move_base_gps_client_topics_py')

        # Instantiate the client once
        move_base_client = MoveBaseClient(utm_zone=56, is_southern_hemisphere=True)

        # Loop to continuously ask for new goals
        while not rospy.is_shutdown():
            try:
                # Get user input for the next goal
                print("\n" + "="*20 + " Enter New Goal " + "="*20)
                lat_str = raw_input("Enter the latitude of the goal (or 's' for status, 'c' to cancel, 'q' to quit): ")

                if lat_str.lower() == 'q':
                    break
                elif lat_str.lower() == 'c':
                    move_base_client.cancel_goal()
                    continue
                elif lat_str.lower() == 's':
                    rospy.loginfo(move_base_client.get_state_text())
                    continue

                lon_str = raw_input("Enter the longitude of the goal: ")
                yaw_str = raw_input("Enter the final yaw angle (in degrees): ")

                # Convert input to float
                lat = float(lat_str)
                lon = float(lon_str)
                yaw = float(yaw_str)

                rospy.loginfo("Attempting to send goal: Lat=%.6f, Lon=%.6f, Yaw=%.2f", lat, lon, yaw)
                move_base_client.send_goal(lat, lon, yaw)
                rospy.loginfo("Goal has been sent. You can enter a new goal, check status, or cancel.")

            except ValueError:
                rospy.logerr("Invalid input. Please ensure you enter numeric values for latitude, longitude, and yaw.")
            except (KeyboardInterrupt, SystemExit):
                rospy.loginfo("User requested shutdown. Cancelling any active goal.")
                move_base_client.cancel_goal()
                break

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation client shut down.")
    except Exception as e:
        rospy.logerr("A critical error occurred: %s", e)