from tramola.task import Task
import rospy


class FollowPath(Task):
    def __init__(self):
        super().__init__()

    def start(self):
        pass

    def detection_callback(self, msg):
        # 0 green buoy
        # 1 red buoy
        # 2 yellow buoy
        nearestGreen = None
        nearestRed = None
        nearestYellow = None
        for detection in msg.detections:
            if detection.confidence < 0.3:
                rospy.rospy.loginfo(f"Low confidence: {detection.confidence}. Ignoring detection.")
                continue
            
            # do not recognize if the object is too far away
            if (detection.class_id == 0 or detection.class_id ==1) and detection.width < 0.05:
                rospy.rospy.loginfo("Object is too far away")
                continue
            elif detection.class_id == 2 and detection.width < 0.0025:
                rospy.rospy.loginfo("Object is too far away")
                continue

            # find the closest object
            if detection.class_id == 0:
                if nearestGreen is None or nearestGreen.width < detection.width:
                    nearestGreen = detection

            elif detection.class_id == 1:
                if nearestRed is None or nearestRed.width < detection.width:
                    nearestRed = detection

            elif detection.class_id == 2:
                if nearestYellow is None or nearestYellow.width < detection.width:
                    nearestYellow = detection
            

        if nearestYellow is None:
            if nearestGreen is None and nearestRed is not None:
                if nearestRed.x_center > 0.3:
                    self.vehicle.go_right()
                else:
                    self.vehicle.go_straight()

            elif nearestRed is None and nearestGreen is not None:
                if nearestGreen.x_center < 0.7:
                    self.vehicle.go_left()
                else:
                    self.vehicle.go_straight()
            elif nearestRed is None and nearestGreen is None:
                self.vehicle.go_straight()
            else:
                middle = (nearestRed.x_center + nearestGreen.x_center) / 2

                if middle < 0.5:
                    self.vehicle.go_left()
                else:
                    self.vehicle.go_right()
        else:
            if nearestGreen is None and nearestRed is not None:
                self.vehicle.go_right()
            elif nearestRed is None and nearestGreen is not None:
                self.vehicle.go_left()
            elif nearestRed is None and nearestGreen is None:
                self.vehicle.go_straight()
            else:
                if abs(nearestGreen.x_center - nearestYellow.x_center) < abs(nearestRed.x_center - nearestYellow.x_center):
                    middle = (nearestRed.x_center + nearestYellow.x_center) / 2
                else:
                    middle = (nearestGreen.x_center + nearestYellow.x_center) / 2
                
                if middle < 0.5:
                    self.vehicle.go_left()
                else:
                    self.vehicle.go_right()




    def _stop(self):
        pass