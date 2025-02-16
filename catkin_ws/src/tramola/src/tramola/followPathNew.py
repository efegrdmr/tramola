from tramola.task import Task
import rospy


class followPath(Task):
    def start(self):
        self.vehicle.set_mode("GUIDED")
        self.ANGULAR_LOSS_RATE = 0.1 # açısal hızın sıfıra yaklaşma oranı
        self.ANGULAR_GAIN_RATE = 0.1 # açısal hızın artma oranı
        self.add_timer(0.3,self.loss_callback)
        self.last_detection_time = rospy.get_time()
        self.vehicle.linear_speed = 0.3
        
    def loss_callback(self):
        if self.vehicle.angular_speed > 0:
            self.vehicle.angular_speed = max(self.vehicle.angular_speed - self.ANGULAR_LOSS_RATE, 0)
        else:
            self.vehicle.angular_speed = min(self.vehicle.angular_speed + self.ANGULAR_LOSS_RATE, 0)

    def calculate_steering_angle(self, desired_x):
        diff = desired_x - 0.5
        self.vehicle.angular_speed += self.ANGULAR_GAIN_RATE * diff
    
    def detection_callback(self, msg):
        nearestGreen = None
        nearestRed = None
        nearestYellow = None

        if len(msg.detections) == 0:
            if rospy.get_time() - self.last_detection_time > 2:
                self.stop()
                return
    
        for detection in msg.detections:
            if detection.confidence < 0.3:
                rospy.logwarn(f"Low confidence: {detection.confidence}. Ignoring detection.")
                continue
            

            # do not recognize if the object is too far away
            if (detection.class_id == self.objects["green_gate_buoy"] or detection.class_id == self.objects["red_gate_buoy"]):
                rospy.logwarn("Object is too far away")
                continue

            elif (detection.class_id == self.objects["green_buoy"] or detection.class_id ==self.objects["red_buoy"]) and detection.width < 0.05:
                rospy.logwarn("Object is too far away")
                continue
            elif detection.class_id == self.objects["yellow_buoy"] and detection.width < 0.0025:
                rospy.logwarn("Object is too far away")
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
                
            
        if nearestGreen is not None or nearestRed is not None or nearestYellow is not None:
            self.last_detection_time = rospy.get_time()

        if nearestYellow is None:
            if nearestGreen is None and nearestRed is None:
                desired_x = 0.5
            elif nearestGreen is None and nearestRed is not None:                
                desired_x = (0.5 + nearestRed.x_center) % 1
            elif nearestGreen is not None and nearestRed is None:
                desired_x = (0.5 + nearestGreen.x_center) % 1
            else:
                desired_x = (nearestGreen.x_center + nearestRed.x_center) / 2
            
        else:
            if nearestGreen is None and nearestRed is None:
                desired_x = (0.5 + nearestYellow.x_center) % 1
            elif nearestGreen is None and nearestRed is not None:                
                if nearestYellow.x_center > 0.5:
                    desired_x = (nearestYellow.x_center + nearestRed.x_center) / 2
                else:
                    desired_x =  (0.5 + max(nearestYellow.x_center, nearestRed.x_center)) % 1
            elif nearestGreen is not None and nearestRed is None:
                if nearestYellow.x_center < 0.5:
                    desired_x = (nearestGreen.x_center + nearestYellow.x_center) / 2
                else:
                    desired_x =  (0.5 + min(nearestYellow.x_center, nearestGreen.x_center)) % 1
            else:
                red_yellow_diff = nearestRed.x_center - nearestYellow.x_center
                green_yellow_diff = nearestGreen.x_center - nearestYellow.x_center
                if red_yellow_diff < green_yellow_diff:
                    desired_x = (nearestGreen.x_center + nearestYellow.x_center) / 2
                else:
                    desired_x = (nearestYellow.x_center + nearestRed.x_center) / 2
        
        self.calculate_steering_angle(desired_x)
       

                
        