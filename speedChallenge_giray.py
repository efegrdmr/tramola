from tramola.task import Task
import rospy


class FollowPath(Task):
    def start(self):
 # State machine variables


        self.vehicle.set_mode("GUIDED")
        self.ANGULAR_LOSS_RATE = 0.1 # açısal hızın sıfıra yaklaşma hızı
        self.ANGULAR_GAIN_RATE = 0.4 # açısal hızın artma oranı
        self.add_timer(0.3,self.loss_callback)
        self.last_detection_time = rospy.get_time()
        self.vehicle.linear_speed = 0.5


        self.state = "FIND_GATE"
        self.blue_buoy_passes = 0
        self.gate_passing_initiated = False
        self.gate_pass_start_time = None
        
        # Blue buoy navigation states
        self.blue_passing_state = "APPROACH"  # States: APPROACH, PASSING, FORWARD, ROTATE
        self.blue_pass_start_time = None
        self.passing_phase_duration = 2.0
        self.forward_duration = 4.0
        self.rotation_duration = 3.0

        # Thresholds
        self.focal_length = 7
        self.real_object_width = 0.3
        self.gate_commit_distance = 50.0  # Distance to commit to gate passing
        self.blue_start_passing_distance = 40.0
        self.gate_pass_timeout = 1.0  # Time to count after losing sight of gate
        self.red_avoid_distance = 25.0  # Distance to start avoiding red buoys
        
        # Black ball detection counter
        self.black_ball_count = 0


        self.bl_last = None
        self.yl_last1 = None
        self.yl_last2 = None

        self.yl_dist1 = None
        self.yl_dist2 = None

  



    def forseconds(self, duration ,action):
        start = rospy.get_time()
        while rospy.get_time()- start< duration:
            action()
 
        return    
    

    def calculate_distance(self, bounding_box_width):
        return (self.vehicle.focal_length * self.vehicle.real_object_width) / bounding_box_width if bounding_box_width > 0 else None



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
    
    def avoid_red_buoy(self, red_buoy):
        detection, distance = red_buoy
        self.get_logger().info(f"Red buoy detected at {distance:.2f} meters. Avoiding...")
        if 0.2<detection.x_center < 0.4:  # Red buoy is on the left
            self.forseconds(0.5,self.vehicle.go_right)
        elif 0.6<detection.x_center < 0.8 :  # Red buoy is on the right
            self.forseconds(0.5,self.vehicle.go_left)
        else:
            self.vehicle.go_straight()

    def avoid_green_buoy(self, green_buoy):
        detection, distance = green_buoy
        self.get_logger().info(f"Red buoy detected at {distance:.2f} meters. Avoiding...")
        if 0.2<detection.x_center < 0.4:  # Green buoy is on the left
            self.forseconds(0.5,self.vehicle.go_right)
        elif 0.6<detection.x_center < 0.8 :  # Green buoy is on the right
            self.forseconds(0.5,self.vehicle.go_left)
        else:
            self.vehicle.go_straight()   

    def handle_gate_navigation(self, red, green):
    
        current_time = rospy.get_time()
        
        if self.gate_passing_initiated:
            self.vehicle.go_straight()
            if not red and not green:
                if self.gate_pass_start_time is None:
                    self.gate_pass_start_time = current_time
                    rospy.logwarn("Lost sight of gate, starting timer...")
                elif current_time - self.gate_pass_start_time > self.gate_pass_timeout:
                    self.state = "FIND_BLUE"
                    self.gate_passing_initiated = False
                    self.gate_pass_start_time = None
                    rospy.logwarn("Gate passed, transitioning to FIND_BLUE state...")
            return
        if red and green:
            detection1, distance1 = red
            detection2, distance2 = green
            
            center_x = (detection1.x_center + detection2.x_center) / 2
            avg_distance = (distance1 + distance2) / 2

            if avg_distance < self.gate_commit_distance:
                self.gate_passing_initiated = True
                self.vehicle.go_straight()
                rospy.logwarn("Committed to passing gate...")
                return

            if center_x < 0.45:
                self.go_left()
            elif center_x > 0.55:
                self.go_right()
            else:
                self.go_straight()
        else:


            if red is None and green is None:
                self.vehicle.go_left
                rospy.logwarn("Start gate is not found rotating..." )


    def handle_blue_buoy_navigation(self, blue_buoy):
            current_time = rospy.get_time()
            if self.blue_passing_state == "APPROACH":
                

                if blue_buoy:
                    detection, distance = blue_buoy
                    self.bl_last = detection.x_center
            
                    
                    if detection.x_center < 0.60:
                        self.vehicle.go_left()
                        rospy.logwarn("going left for blue")
                    elif detection.x_center > 0.70:
                        self.vehicle.go_right()
                        rospy.logwarn("going right for blue")
                    else:
                        self.vehicle.go_straight()

                    if distance < self.blue_start_passing_distance:
                        self.blue_passing_state = "PASSING"
                        rospy.logwarn("PASSİNG THE BLUE BUOY.........")
                        self.blue_pass_start_time = current_time
                else:
                    if self.bl_last:

                        if self.bl_last<0.5:
                            self.forseconds( 0.1,self.vehicle.go_left)
                            rospy.logwarn("FORSECONDS FOR BLUEBUOY TRYİNG")
                        elif self.bl_last>0.5:
                            self.forseconds( 0.1,self.vehicle.go_right)
                            rospy.logwarn("forseconds bluebuoy trying")


            elif self.blue_passing_state == "PASSING":
                if current_time - self.blue_pass_start_time < self.passing_phase_duration:
                    self.vehicle.go_straight()
                else:
                    self.blue_passing_state = "FORWARD"
                    self.blue_pass_start_time = current_time

            elif self.blue_passing_state == "FORWARD":
                if current_time - self.blue_pass_start_time < self.forward_duration:
                    self.vehicle.go_straight()
                else:
                    self.blue_passing_state = "ROTATE"
                    self.blue_pass_start_time = current_time

            elif self.blue_passing_state == "ROTATE":
                if current_time - self.blue_pass_start_time < self.rotation_duration:
                    self.forseconds(5,self.vehicle.go_right)
                    rospy.logwarn( "ROTATE TO 2ND BLUE PASS ")
                else:
                    if self.blue_buoy_passes >= 1:
                        self.state = "RETURN_TO_GATE"
                        rospy.logwarn("Blue buoy circumnavigation complete, returning to gate...")
                    else:
                        self.blue_passing_state = "APPROACH"
                        self.blue_buoy_passes += 1
                        rospy.logwarn("Starting pass of blue buoy...")
    def handle_final_gate_pass(self, red, green):
            current_time = rospy.get_time()

            if self.gate_passing_initiated:
                self.vehicle.go_straight()
                if not red and not green:
                    if self.gate_pass_start_time is None:
                        self.gate_pass_start_time = current_time
                        rospy.logwarn("Lost sight of final gate, starting timer...")
                    elif current_time - self.gate_pass_start_time > self.gate_pass_timeout:
                        rospy.logwarn(" final gate lost stopping....")
                        self.stop()
                        self.vehicle.linear_speed = 0
                        
                return

            if red and green:
                detection1, distance1 = red
                detection2, distance2 = green
                
                center_x = (detection1.x_center + detection2.x_center) / 2
                avg_distance = (distance1 + distance2) / 2

            
                center_x = (detection1.x_center + detection2.x_center) / 2
                avg_distance = (distance1 + distance2) / 2
                
                self.yl_last1 =detection1.x_center
                self.yl_last2 =detection2.x_center
                self.yl_dist1 = distance1
                self.yl_dist2 = distance2

                if avg_distance < self.gate_commit_distance:
                    self.gate_passing_initiated = True
                    self.vehicle.go_straight()
                    rospy.logwarn("Committed to passing final gate...")
                    return

                if center_x < 0.45:
                    self.vehicle.go_left()
                elif center_x > 0.55:
                    self.vehicle.go_right()
        

    def detection_callback(self, msg):
        nearestGreen = None
        nearestRed = None
        nearestYellow = None

        self.vehicle.linear_speed= 0.5

        


        if len(msg.detections) == 0:
            if rospy.get_time() - self.last_detection_time > 2:
                rospy.logwarn("No detection for 2 seconds. Stopping.")
                self.stop()
                return
    
        for detection in msg.detections:

            distance = self.calculate_distance(detection.width)
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
                




            if nearestRed is not None and nearestGreen is not None and -20<(nearestRed.distance - nearestGreen.distance)<20:
                desired_x = (nearestRed.x_center + nearestGreen.x_center)/2
                if nearestGreen.distance < self.gate_commit_distance or nearestRed< self.gate_commit_distance:
                    self.gate_passing_initiated= True
            elif nearestRed is not None and nearestGreen is None and nearestRed.distance <self.red_avoid_distance:
                self.avoid_red_buoy(nearestRed)
                return
            elif nearestRed is  None and nearestGreen is not None and nearestGreen.distance <self.red_avoid_distance:
                self.avoid_red_buoy(nearestGreen)
                return

            if self.state == "FIND_GATE":
                self.handle_gate_navigation(nearestRed, nearestGreen)
            elif self.state == "FIND_BLUE":
                self.handle_blue_buoy_navigation(nearestYellow)
            elif self.state == "RETURN_TO_GATE":
                self.handle_final_gate_pass(nearestRed,nearestGreen)






        self.calculate_steering_angle(desired_x)
       

