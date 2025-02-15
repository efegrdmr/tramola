import rospy
from tramola.task import Task
from tramola.msg import DetectionList, Detection
import time


class SpeedChallenge(Task):
    def start(self):
        self.vehicle.set_mode("GUIDED")
        self.status = "WAITING_FOR_GREEN_LIGHT"
        self.blueBuoyLastSeen = time.time()
        self.previousStatus = None

    def set_status(self, status):
        self.previousStatus = self.status
        self.status = status

    def detection_callback(self, msg):
        nearestObstacle = None
        blueBuoy = None

        if self.status == "WAITING_FOR_GREEN_LIGHT":
            pass
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

        
        if self.status == "GOING_TO_BLUE_BUOY":
            if nearestObstacle is not None:
                self.set_status("AVOIDING_OBSTACLE")
                return
            if blueBuoy is not None:
                if blueBuoy.x_center > 0.4:
                    self.vehicle.go_right()
                if blueBuoy.x_center > 0.8:
                    self.vehicle.go_left()
                else:
                    self.vehicle.go_straight()
            else:
                if time.time() - self.blueBuoyLastSeen > 2:
                    self.set_status("PAST_BLUE_BUOY")

        elif self.status == "AVOIDING_OBSTACLE":
            if nearestObstacle is None:
                self.set_status(self.previousStatus)
                return
            
            if nearestObstacle.x_center > 0.3:
                self.vehicle.go_right()
            elif nearestObstacle.x_center < 0.7:
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
            
