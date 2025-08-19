#!/usr/bin/env python

import rospy
import rosgraph_msgs.msg
from visualization_msgs.msg import Marker, MarkerArray

class TimestampVisualizer:
    def __init__(self):
        rospy.init_node('timestamp_visualizer', anonymous=True)
        
        # Time callback için subscribe ol
        rospy.Subscriber('/clock', rosgraph_msgs.msg.Clock, self.time_callback)
        
        # Marker publisher
        self.marker_pub = rospy.Publisher('/time_display', MarkerArray, queue_size=1)
        
        self.latest_time = None
    
    def time_callback(self, clock_msg):
        self.latest_time = clock_msg.clock
        self.publish_time_marker()
    
    def publish_time_marker(self):
        if not self.latest_time:
            return
        
        # Zaman damgası metni oluştur - ROS tarzı (saniye.nanosaniye)
        secs = self.latest_time.secs
        nsecs = self.latest_time.nsecs
        
        # ROS tarzı zaman damgası formatı: saniye.nanosaniye
        formatted_time = "%d.%09d" % (secs, nsecs)
        
        # Metin marker'ı oluştur
        marker = Marker()
        marker.header.frame_id = "velodyne"  # LIDAR frame'inize göre ayarlayın
        marker.header.stamp = rospy.Time.now()
        marker.ns = "timestamp"
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        
        # Pozisyon - görüş alanında üstte görünecek şekilde ayarlayın
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = 3  # LIDAR'ın üzerinde göster
        
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0
        
        marker.scale.x = 0.5
        marker.scale.y = 0.5
        marker.scale.z = 0.5
        
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0
        
        marker.text = "ROS Time: " + formatted_time
        marker.lifetime = rospy.Duration(0.2)  # Sadece kısa bir süre göster
        
        # MarkerArray oluştur ve yayınla
        marker_array = MarkerArray()
        marker_array.markers.append(marker)
        self.marker_pub.publish(marker_array)

if __name__ == '__main__':
    try:
        visualizer = TimestampVisualizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass