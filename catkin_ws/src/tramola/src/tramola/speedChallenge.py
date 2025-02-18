import rospy
from tramola.task import Task
from geopy.distance import geodesic

class SpeedChallenge(Task):
    def start(self):
        self.vehicle.set_mode("GUIDED")
        self.status = "WAITING_FOR_GREEN_LIGHT"
        self.blueBuoyLastSeen = rospy.get_time()
        self.previousStatus = None
        self.start_location = self.vehicle.location
        self.start_orientation = self.vehicle.orientation

        self.target_last_x = 0.5

        self.ANGULAR_LOSS_RATE = 0.1 # açısal hızın sıfıra yaklaşma hızı
        self.ANGULAR_GAIN_RATE = 0.4 # açısal hızın artma oranı
        self.add_timer(0.3,self.loss_callback)
        self.vehicle.linear_speed = 0.5
        
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

    def set_status(self, status):
        self.previousStatus = self.status
        self.status = status

    def detection_callback(self, msg):
        nearestObstacle = None
        blueBuoy = None
        greenBuoy = None
        redBuoy = None

        if self.status == "WAITING_FOR_GREEN_LIGHT":
            # TODO
            self.set_status("GOING_TO_BLUE_BUOY")
            return

        for detection in msg.detections:
            if detection.confidence < 0.3:
                rospy.logwarn(f"Low confidence: {detection.confidence}. Ignoring detection.")
                continue

            if detection.class_id == self.objects["blue_buoy"]:
                blueBuoy = detection
                self.blueBuoyLastSeen = rospy.get_time()
            
            elif 0.4 < detection.x_center < 0.6 and detection.width > 0.2:
                if nearestObstacle is None:
                    nearestObstacle = detection
                elif abs(0.5 - detection.x_center) < abs(0.5 - nearestObstacle.x_center):
                    nearestObstacle = detection

            if detection.class_id == self.objects["green_gate_buoy"]:
                greenBuoy = detection
            elif detection.class_id == self.objects["red_gate_buoy"]:
                redBuoy = detection

        
        if self.status == "GOING_TO_BLUE_BUOY":            
            if nearestObstacle is not None:
                self.set_status("AVOIDING_OBSTACLE")
                return
            if blueBuoy is not None:
                self.target_last_x = blueBuoy.x_center

            if self.target_last_x > 0.3:
                # make sure the vehicle turns right
                diff = (self.target_last_x - 0.3) / 7 * 5
                desired_x = 0.5 + diff
            elif self.target_last_x < 0.2:
                diff = (0.2 - self.target_last_x) / 2 * 5
                desired_x = 0.5 - diff
            else:
                desired_x = 0.5

            if blueBuoy is None:
                if rospy.get_time() - self.blueBuoyLastSeen > 2:
                    if self.previousStatus == "PAST_BLUE_BUOY":
                        self.set_status("RETURNING")
                    else:
                        self.set_status("PAST_BLUE_BUOY")

        elif self.status == "AVOIDING_OBSTACLE":
            if nearestObstacle is None:
                self.set_status(self.previousStatus)
                return
            
            if 0.5 < nearestObstacle.x_center < 0.7:
                desired_x = (0.5 + nearestObstacle.x_center) % 1
            else:
                self.set_status(self.previousStatus)
                return
            
        elif self.status == "PAST_BLUE_BUOY":
            if nearestObstacle is not None:
                self.set_status("AVOIDING_OBSTACLE")
                return
            if blueBuoy is not None:
                self.set_status("GOING_TO_BLUE_BUOY")
                return
            else:
                self.vehicle.turn_left(0.3)
                return
        elif self.status == "RETURNING":
            if nearestObstacle is not None:
                self.set_status("AVOIDING_OBSTACLE")
                return
            if greenBuoy is None and redBuoy is None:
                if geodesic(self.start_location, self.vehicle.location).meters < 3:
                    self.stop()
                    return
                else:
                    self.vehicle.send_location(*self.start_location)
                    rospy.sleep(0.2)
                    return
            elif greenBuoy is not None and redBuoy is not None:
                desired_x = (greenBuoy.x_center + redBuoy.x_center) / 2
            elif greenBuoy is not None:
                if greenBuoy.x_center > 0.3:
                    diff = (greenBuoy.x_center - 0.3) / 7 * 5
                    desired_x = 0.5 + diff
                elif greenBuoy.x_center < 0.2:
                    diff = (0.2 - greenBuoy.x_center) / 2 * 5
                    desired_x = 0.5 - diff
                else:
                    desired_x = 0.5
            elif redBuoy is not None:
                if redBuoy.x_center < 0.7:
                    diff = (0.7 - redBuoy.x_center) / 7 * 5
                    desired_x = 0.5 - diff
                elif redBuoy.x_center > 0.8:
                    diff = (redBuoy.x_center - 0.8) / 2 * 5
                    desired_x = 0.5 + diff
                else:
                    desired_x = 0.5
        
        self.calculate_steering_angle(desired_x)

            
