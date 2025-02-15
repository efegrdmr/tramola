import rospy
from tramola.task import Task
import time
from geopy.distance import geodesic

class SpeedChallenge(Task):
    def start(self):
        self.vehicle.set_mode("GUIDED")
        self.status = "WAITING_FOR_GREEN_LIGHT"
        self.blueBuoyLastSeen = time.time()
        self.previousStatus = None
        self.start_location = self.vehicle.location
        self.start_orientation = self.vehicle.orientation
        # TESTING
        self.objects["blue_buoy"] = self.objects["yellow_buoy"]
        # TESTING

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
                self.blueBuoyLastSeen = time.time()
            
            elif 0.4 < detection.x_center < 0.6 and detection.width > 0.2:
                if nearestObstacle is None or nearestObstacle.x_center > detection.x_center:
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
                if blueBuoy.x_center > 0.4:
                    self.vehicle.go_right()
                elif blueBuoy.x_center < 0.1:
                    self.vehicle.go_left()
                else:
                    self.vehicle.go_straight()
            else:
                self.vehicle.go_straight(0.2)
                if time.time() - self.blueBuoyLastSeen > 2:
                    if self.previousStatus == "PAST_BLUE_BUOY":
                        self.set_status("RETURNING")
                    else:
                        self.set_status("PAST_BLUE_BUOY")

        elif self.status == "AVOIDING_OBSTACLE":
            if nearestObstacle is None:
                self.set_status(self.previousStatus)
                return
            
            if 0.5 > nearestObstacle.x_center > 0.3:
                self.vehicle.go_right()
            elif 0.5 < nearestObstacle.x_center < 0.7:
                self.vehicle.go_left()
            else:
                self.set_status(self.previousStatus)
            

        elif self.status == "PAST_BLUE_BUOY":
            if nearestObstacle is not None:
                self.set_status("AVOIDING_OBSTACLE")
                return
            if blueBuoy is not None:
                self.set_status("GOING_TO_BLUE_BUOY")
            else:
                self.vehicle.turn_left(0.7)
        elif self.status == "RETURNING":
            if nearestObstacle is not None:
                self.set_status("AVOIDING_OBSTACLE")
                return
            
            # TESTING
            if geodesic(self.start_location, self.vehicle.location).meters < 3:
                    self.stop()
            else:
                self.vehicle.send_location(*self.start_location)
                rospy.sleep(0.5)
            return
            # TESTING
            if greenBuoy is None and redBuoy is None:
                if geodesic(self.start_location, self.vehicle.location).meters < 3:
                    self.stop()
                else:
                    self.vehicle.send_location(*self.start_location)
                    rospy.sleep(0.5)
            elif greenBuoy is not None:
                if greenBuoy.x_center > 0.3:
                    self.vehicle.go_right()
                elif greenBuoy.x_center < 0.1:
                    self.vehicle.go_left()
                else:
                    self.vehicle.go_straight()
            elif redBuoy is not None:
                if redBuoy.x_center > 0.9:
                    self.vehicle.go_right()
                elif redBuoy.x_center < 0.6:
                    self.vehicle.go_left()
                else:
                    self.vehicle.go_straight()
            


            
