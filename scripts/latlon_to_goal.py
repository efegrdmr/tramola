#!/usr/bin/env python2
# -*- coding: utf-8 -*-

import rospy
import tf2_ros
import tf2_geometry_msgs  # provides Python2 bindings for transforming PoseStamped
from pyproj import Proj, transform
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

class LatLonToGoal(object):
    """
    Converts NavSatFix (lat/lon) to a PoseStamped in the 'map' frame
    and publishes it to /move_base_simple/goal.
    """
    def __init__(self):
        rospy.init_node('latlon_to_goal_node')

        # --- UTM CONFIGURATION (keep at Zone 56 South) ---
        self.utm_zone = 56
        self.is_southern_hemisphere = True

        # pyproj projections
        self.proj_latlon = Proj(proj='latlong', datum='WGS84')
        self.proj_utm    = Proj(
            proj='utm',
            zone=self.utm_zone,
            ellps='WGS84',
            south=self.is_southern_hemisphere
        )

        # TF2: buffer & listener
        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        # Publisher & Subscriber
        self.goal_pub = rospy.Publisher(
            '/move_base_simple/goal',
            PoseStamped,
            queue_size=1
        )
        rospy.Subscriber(
            '/lat_lon_goal',
            NavSatFix,
            self.goal_callback,
            queue_size=1
        )

        rospy.loginfo("latlon_to_goal_node initialized (UTM Zone %d %s)",
                      self.utm_zone,
                      "South" if self.is_southern_hemisphere else "North")

    def goal_callback(self, msg):
        rospy.loginfo("Received GPS goal: lat=%.6f lon=%.6f",
                      msg.latitude, msg.longitude)

        # 1) Lat/Lon -> UTM
        try:
            easting, northing = transform(
                self.proj_latlon,
                self.proj_utm,
                msg.longitude,
                msg.latitude
            )
        except Exception as e:
            rospy.logerr("pyproj.transform failed: %s", e)
            return

        rospy.loginfo("Converted to UTM: E=%.3f, N=%.3f", easting, northing)

        # 2) Build a PoseStamped in 'utm' frame
        utm_pose = PoseStamped()
        utm_pose.header.frame_id = 'utm'
        utm_pose.header.stamp = rospy.Time.now()
        utm_pose.pose.position.x = easting
        utm_pose.pose.position.y = northing
        utm_pose.pose.orientation.w = 1.0

        # 3) Wait for the utm->map transform
        target_frame = 'map'
        source_frame = utm_pose.header.frame_id
        if not self.tf_buffer.can_transform(
                source_frame,
                target_frame,
                rospy.Time(0),
                rospy.Duration(5.0)
        ):
            rospy.logerr(
                "Transform %s→%s not available after 5s; "
                "check navsat_transform_node and TF tree",
                source_frame, target_frame
            )
            return

        # 4) Actually transform into 'map'
        try:
            map_pose = self.tf_buffer.transform(
                utm_pose,
                target_frame,
                rospy.Duration(1.0)
            )
        except (tf2_ros.LookupException,
                tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            rospy.logerr("Failed to transform goal: %s", e)
            return

        # 5) Publish to move_base
        self.goal_pub.publish(map_pose)
        rospy.loginfo("Published goal in 'map' frame: x=%.3f y=%.3f",
                      map_pose.pose.position.x,
                      map_pose.pose.position.y)

if __name__ == '__main__':
    try:
        node = LatLonToGoal()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
