from tramola.task import Task
import rospy
import time


class FollowPath(Task):
    def start(self):
        # PID constants (tweak these as needed)
        self.kp = 1.0
        self.ki = 0.0
        self.kd = 0.1
        self.integral = 0.0
        self.last_error = 0.0
        self.last_pid_time = time.time()
        self.max_steer_angle = 0.3

        self.vehicle.set_mode("GUIDED")
        self.last_detection_time = time.time()
        self.last_pid_time = time.time()

    def detection_callback(self, msg):
        nearestGreen = None
        nearestRed = None
        nearestYellow = None

        if len(msg.detections) == 0:
            if time.time() - self.last_detection_time > 2:
                self.vehicle.set_mode("MANUAL")
                self.stop()
                return
        

        for detection in msg.detections:
            if detection.confidence < 0.3:
                rospy.logwarn(f"Low confidence: {detection.confidence}. Ignoring detection.")
                continue
            

            # do not recognize if the object is too far away
            if (detection.class_id == self.objects["green_gate_buoy"] or detection.class_id == self.objects["red_gate_buoy"]) and detection.width < 0.09:
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
            self.last_detection_time = time.time()


        # Compute a measured x_center based on available detections
        measured_center = None
        desired_center = 0.5  # centered value

        if nearestYellow is None:  # no yellow buoy detected
            if nearestGreen is not None and nearestRed is not None:
                measured_center = (nearestGreen.x_center + nearestRed.x_center) / 2
            elif nearestRed is not None:
                if nearestRed.x_center < 0.5:
                    measured_center = 0.5 + nearestRed.x_center
                else:
                    measured_center = 0.5 - nearestRed.x_center
            elif nearestGreen is not None:
                if nearestGreen.x_center < 0.5:
                    measured_center = 0.5 + nearestGreen.x_center
                else:
                    measured_center = 0.5 - nearestGreen.x_center
        else:
            # If yellow is present, combine it with whichever buoy is present.
            if nearestGreen is not None and nearestRed is not None:
                # Choose the pair that is furthest together
                if abs(nearestGreen.x_center - nearestYellow.x_center) > abs(nearestRed.x_center - nearestYellow.x_center):
                    measured_center = (nearestGreen.x_center + nearestYellow.x_center) / 2
                else:
                    measured_center = (nearestRed.x_center + nearestYellow.x_center) / 2
            elif nearestRed is not None:
                measured_center = (nearestRed.x_center + nearestYellow.x_center) / 2
            elif nearestGreen is not None:
                measured_center = (nearestGreen.x_center + nearestYellow.x_center) / 2

        # If no detection could provide a measured center, keep heading straight.
        if measured_center is None:
            self.vehicle.go_straight()
            return

        # PID CONTROL
        now = time.time()
        dt = now - self.last_pid_time
        if dt == 0:
            dt = 0.01

        error = desired_center - measured_center
        self.integral += error * dt
        derivative = (error - self.last_error) / dt

        # Compute the control signal
        control = self.kp * error + self.ki * self.integral + self.kd * derivative

        # Limit the control signal to the maximum steering angle
        control = max(-self.max_steer_angle, min(self.max_steer_angle, control))

        self.last_pid_time = now
        self.last_error = error

        # Decide vehicle command based on PID output:
        # Adjust these thresholds and functions to suit your vehicle's API.
        base_speed = 0.5
        if abs(control) < 0.05:
            # Go straight
            self.vehicle.set_velocity(base_speed, 0.0)
        else:
            # Turn: positive control means error is positive so steer right (negative angular speed)
            angular_speed = -abs(control) if control > 0 else abs(control)
            self.vehicle.set_velocity(base_speed, angular_speed)
