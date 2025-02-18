import numpy as np
from tramola.task import Task
import rospy
from tramola.sort import convert_normalized_to_bbox, Sort


class FollowPath(Task):
    def start(self):
        self.vehicle.set_mode("GUIDED")
        self.ANGULAR_LOSS_RATE = 0.1 # açısal hızın sıfıra yaklaşma hızı
        self.ANGULAR_GAIN_RATE = 0.4 # açısal hızın artma oranı
        self.add_timer(0.3,self.loss_callback)
        self.last_detection_time = rospy.get_time()
        self.vehicle.linear_speed = 0.5
        self.mot_tracker = Sort() 
        self.track_bbs_ids = np.array([])
        
    def loss_callback(self, event):
        if self.vehicle.angular_speed > 0:
            self.vehicle.angular_speed = max(self.vehicle.angular_speed - self.ANGULAR_LOSS_RATE, 0)
        else:
            self.vehicle.angular_speed = min(self.vehicle.angular_speed + self.ANGULAR_LOSS_RATE, 0)

    def calculate_steering_angle(self, desired_x):
        rospy.loginfo("desired " + str(desired_x))
        
        diff = -(desired_x - 0.5)
        self.vehicle.angular_speed +=  self.ANGULAR_GAIN_RATE * diff
        if self.vehicle.angular_speed < 0:
            rospy.loginfo("going right")
        else:
            rospy.loginfo("going left")
        
    

    def detection_callback(self, msg):
        nearestGreen = None
        nearestRed = None
        nearestYellow = None

        if len(msg.detections) == 0:
            if rospy.get_time() - self.last_detection_time > 2:
                rospy.logwarn("No detection for 2 seconds. Stopping.")
                self.stop()
                rospy.loginfo("total yellow buoys detected: " + str(self.track_bbs_ids.shape[0]))
                
                return

        yellow_buoys = []
        for detection in msg.detections:
            if detection.class_id == self.objects["yellow_buoy"]:
                yellow_buoys.append(convert_normalized_to_bbox(detection.x_center, detection.y_center, detection.width, detection.height, detection.confidence))

            if detection.confidence < 0.3:
                rospy.logwarn(f"Low confidence: {detection.confidence}. Ignoring detection.")
                continue
            

            # do not recognize if the object is too far away
            # if (detection.class_id == self.objects["green_gate_buoy"] or detection.class_id == self.objects["red_gate_buoy"]):
            #    continue

            elif (detection.class_id == self.objects["green_buoy"] or detection.class_id ==self.objects["red_buoy"]) and detection.width < 0.05:
                continue
            elif detection.class_id == self.objects["yellow_buoy"] and detection.width < 0.0025:
                continue

            # find the closest object
            if detection.class_id == self.objects["green_gate_buoy"]:
                if nearestGreen is None or nearestGreen.class_id == self.objects["green_buoy"] or nearestGreen.width < detection.width:
                    nearestGreen = detection

            elif detection.class_id == self.objects["green_buoy"]:
                if nearestGreen is None or (nearestGreen.class_id != self.objects["green_gate_buoy"] and nearestGreen.width < detection.width):
                    nearestGreen = detection

            elif detection.class_id == self.objects["red_gate_buoy"]:
                if nearestRed is None or nearestRed.class_id == self.objects["red_buoy"] or nearestRed.width < detection.width:
                    nearestRed = detection

            elif detection.class_id == self.objects["red_buoy"]:
                if nearestRed is None or (nearestRed.class_id != self.objects["red_gate_buoy"] and nearestRed.width < detection.width):
                    nearestRed = detection

            elif detection.class_id == self.objects["yellow_buoy"]:
                if nearestYellow is None or nearestYellow.width < detection.width:
                    nearestYellow = detection

        if len(yellow_buoys) > 0:
            self.track_bbs_ids = self.mot_tracker.update(np.array(yellow_buoys))
        else:
            self.track_bbs_ids = self.mot_tracker.update()
        if nearestGreen is not None or nearestRed is not None or nearestYellow is not None:
            self.last_detection_time = rospy.get_time()

        if nearestYellow is None or \
        (nearestYellow.x_center < 0.3 or nearestYellow.x_center > 0.7 or nearestYellow.y_center > 0.6):
            if nearestGreen is None and nearestRed is None:
                rospy.logwarn("No green or red buoy or yellow buoy detected. Going straight.")
                desired_x = 0.5
            elif nearestGreen is None and nearestRed is not None:      
                rospy.logwarn("Red buoy detected. avoiding the red buoy. x " + str(nearestRed.x_center))
                if nearestRed.x_center < 0.5:
                    desired_x = (0.5 + nearestRed.x_center) % 1
                else:
                    desired_x = 0.9 # manually turn right to get back on track
            elif nearestGreen is not None and nearestRed is None:
                rospy.logwarn("Green buoy detected. avoiding the green buoy.")
                if nearestGreen.x_center > 0.5:
                    desired_x = (0.5 + nearestGreen.x_center) % 1
                else:
                    desired_x = 0.1 # manually turn left to get back on track
            else:
                rospy.logwarn("Both green and red buoy detected. Going between them.")
                desired_x = (nearestGreen.x_center + nearestRed.x_center) / 2
            
        else:
            if nearestGreen is None and nearestRed is None:
                rospy.logwarn("No green or red buoy detected. avoiding the yellow buoy.")
                desired_x = (0.5 + nearestYellow.x_center) % 1
            elif nearestGreen is None and nearestRed is not None:                
                if nearestYellow.x_center > 0.8:
                    rospy.logwarn("Red buoy and yellow buoy detected yellow is far from red. Going between them.")
                    desired_x = (nearestYellow.x_center + nearestRed.x_center) / 2
                else:
                    desired_x =  (0.5 + max(nearestYellow.x_center, nearestRed.x_center)) % 1
                    rospy.logwarn("Red buoy and yellow buoy detected yellow is close to red. avoiding them.")
            elif nearestGreen is not None and nearestRed is None:
                if nearestYellow.x_center < 0.3:
                    rospy.logwarn("Green buoy and yellow buoy detected yellow is far from green. Going between them.")
                    desired_x = (nearestGreen.x_center + nearestYellow.x_center) / 2
                else:
                    rospy.logwarn("Green buoy and yellow buoy detected yellow is close to green. avoiding them.")
                    desired_x =  (0.5 + min(nearestYellow.x_center, nearestGreen.x_center)) % 1
            else:
                rospy.logwarn("All buoys detected. Avoiding the closest one.")
                closest = min(nearestGreen.x_center, nearestRed.x_center, nearestYellow.x_center)
                desired_x = (0.5 + closest) % 1
        
        self.calculate_steering_angle(desired_x)
       

                
        