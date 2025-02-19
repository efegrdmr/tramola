from tramola.task import Task
import rospy
import time
import math
#import utm
from tramola.sensor import get_sensor_distance

# açıları sabit kullandım onları dinamik hale getirmem lazım. kod çalıştığı durumda.
class Docking(Task):
    def __init__(self):
        super().__init__()

    def start(self):
        self.vehicle.set_mode("GUIDED")
        self.last_detection_time = time.time()

    
    def find_dock(self, msg):
        for detection in msg.detections:
            self.sensor_distance = get_sensor_distance()
            self.screen_center = 0.5

            dets = [d for d in self.detections if d.class_id < 12]
            
            
            if detection.class_id == self.objects["red_circle"]:
                self.chosen_shape_id = detection
            
           
            try:
                self.sensor_distance = get_sensor_distance()
                if self.sensor_distance is None or self.sensor_distance == 0:
                    rospy.logwarn("Sensor distance is None or 0. Skipping this detection.")
                    continue  # Mesafe geçerli değilse, bu tespiti atla
            except Exception as e:
                rospy.logerr(f"Error getting sensor distance: {e}")
                continue

                
            if detection.confidence < 0.3:
                    rospy.logwarn(f"Low confidence: {detection.confidence}. Ignoring detection.")
                    continue
                     
            if len(dets) == 0:
                self.vehicle.go_straight()
                rospy.sleep(5)
                if len(dets) == 0:
                    self.vehicle.turn_degrees(20)
                    self.vehicle.go_straight()
                    rospy.sleep(2)
                    if len(dets) == 0:
                        self.vehicle.turn_degrees(-40)
                        self.vehicle.go_straight()
                        rospy.sleep(2)

                    else:
                        if detection.self.chosen_shape_id  is not None:
                            if  abs(self.offset) < 0.1: 
                                while self.sensor_distance > 125:
                                    self.vehicle.go_straight()
                                    if self.sensor_distance == 125:
                                        self.vehicle.stop()
                                        if self.offset < -0.1:
                                            rospy.loginfo("The object is to the right, and the vehicle turns to the right.")
                                            self.vehicle.turn_degrees(5)
                                            while self.sensor_distance > 125:
                                                self.vehicle.go_straight()
                                                if self.sensor_distance == 125:
                                                    self.vehicle.stop()
                                                    return True
                                        elif self.offset > 0.1:
                                            rospy.loginfo("The object is to the left, and the vehicle turns to the left.")
                                            self.vehicle.turn_degrees(-5)
                                            while self.sensor_distance > 125:
                                                self.vehicle.go_straight()
                                                if self.sensor_distance == 125:
                                                    self.vehicle.stop()
                                                    return True
                else:
                    if detection.self.chosen_shape_id is not None:
                        if  abs(self.offset) < 0.1: 
                            while self.sensor_distance > 125:
                                self.vehicle.go_straight()
                                if self.sensor_distance == 125:
                                    self.vehicle.stop()
                                    if self.offset < -0.1:
                                        rospy.loginfo("The object is to the right, and the vehicle turns to the right.")
                                        self.vehicle.turn_degrees(5)
                                        while self.sensor_distance > 125:
                                            self.vehicle.go_straight()
                                            if self.sensor_distance == 125:
                                                self.vehicle.stop()
                                                return True
                                    elif self.offset > 0.1:
                                        rospy.loginfo("The object is to the left, and the vehicle turns to the left.")
                                        self.vehicle.turn_degrees(-5)
                                        while self.sensor_distance > 125:
                                            self.vehicle.go_straight()
                                            if self.sensor_distance == 125:
                                                self.vehicle.stop()
                                                return True
            else:
                if detection.self.chosen_shape_id is not None:
                    if  abs(self.offset) < 0.1: 
                        while self.sensor_distance > 125:
                            self.vehicle.go_straight()
                            if self.sensor_distance == 125:
                                self.vehicle.stop()
                                if self.offset < -0.1:
                                    rospy.loginfo("The object is to the right, and the vehicle turns to the right.")
                                    self.vehicle.turn_degrees(5)
                                    while self.sensor_distance > 125:
                                        self.vehicle.go_straight()
                                        if self.sensor_distance == 125:
                                            self.vehicle.stop()
                                            return True
                                elif self.offset > 0.1:
                                    rospy.loginfo("The object is to the left, and the vehicle turns to the left.")
                                    self.vehicle.turn_degrees(-5)
                                    while self.sensor_distance > 125:
                                        self.vehicle.go_straight()
                                        if self.sensor_distance == 125:
                                            self.vehicle.stop()
                                            return True
                                        

    def detection_callback(self, msg):
        # aracın limana yanaşmasını sağlamak için.
        for detection in msg.detections:
            self.sensor_distance = get_sensor_distance()
            self.screen_center = 0.5
            self.offset = self.screen_center - self.chosen_shape_id.x_center 

            if detection.class_id == self.objects["red_circle"]:
                self.chosen_shape_id = detection
            
            
            if self.find_dock(self,msg): # bakcam tekrar.
                self.vehicle.stop()
                if get_sensor_distance() < 125:
                    rospy.loginfo("The dock is not empty.")
                    self.vehicle.turn_degrees(-90)
                    self.vehicle.go_straight()
                    rospy.sleep(5)
                    self.vehicle.stop()
                    self.vehicle.turn_degrees(90)
                    self.vehicle.go_straight()
                    rospy.sleep(3)
                    self.vehicle.stop()
                    self.vehicle.turn_degrees(20)
                    self.vehicle.go_straight()
                    rospy.sleep(3)
                    self.vehicle.turn_degrees(160)
                    if detection.self.chosen_shape_id is None:
                        self.vehicle.turn_degrees(-20)
                        if detection.self.chosen_shape_id is None:
                            self.vehicle.turn_degrees(40)
                            if detection.self.chosen_shape_id  is not None:
                                self.vehicle.go_straight()
                                rospy.sleep(2)
                                self.vehicle.turn_degrees(-20)
                                if abs(self.offset) < 0.1: 
                                    while self.sensor_distance > 22:
                                        self.vehicle.go_straight()
                                        if self.sensor_distance == 22:
                                            self.vehicle.stop()

                                if self.offset < -0.1:
                                    rospy.loginfo("The object is to the right, and the vehicle turns to the right.")
                                    self.vehicle.turn_degrees(5)
                                    while self.sensor_distance > 22:
                                        self.vehicle.go_straight()
                                        if self.sensor_distance == 22:
                                            self.vehicle.stop()
                                            

                                elif self.offset > 0.1:
                                    rospy.loginfo("The object is to the left, and the vehicle turns to the left.")
                                    self.vehicle.turn_degrees(-5)
                                    while self.sensor_distance > 22:
                                        self.vehicle.go_straight()
                                        if self.sensor_distance == 22:
                                            self.vehicle.stop()
                        
                                
                        
                        else:
                            self.vehicle.go_straight()
                            rospy.sleep(2)
                            self.vehicle.turn_degrees(20)
                            if abs(self.offset) < 0.1: 
                                while self.sensor_distance > 22:
                                    self.vehicle.go_straight()
                                    if self.sensor_distance == 22:
                                        self.vehicle.stop()

                            if self.offset < -0.1:
                                rospy.loginfo("The object is to the right, and the vehicle turns to the right.")
                                self.vehicle.turn_degrees(5)
                                while self.sensor_distance > 22:
                                    self.vehicle.go_straight()
                                    if self.sensor_distance == 22:
                                        self.vehicle.stop()
                                        

                            elif self.offset > 0.1:
                                rospy.loginfo("The object is to the left, and the vehicle turns to the left.")
                                self.vehicle.turn_degrees(-5)
                                while self.sensor_distance > 22:
                                    self.vehicle.go_straight()
                                    if self.sensor_distance == 22:
                                        self.vehicle.stop()
                    
                    else:
                        if abs(self.offset) < 0.1: 
                            while self.sensor_distance > 22:
                                self.vehicle.go_straight()
                                if self.sensor_distance == 22:
                                    self.vehicle.stop()
                        
                        elif self.offset < -0.1:
                            rospy.loginfo("The object is to the right, and the vehicle turns to the right.")
                            self.vehicle.turn_degrees(5)
                            while self.sensor_distance > 22:
                                self.vehicle.go_straight()
                                if self.sensor_distance == 22:
                                    self.vehicle.stop()
                                   

                        elif self.offset > 0.1:
                            rospy.loginfo("The object is to the left, and the vehicle turns to the left.")
                            self.vehicle.turn_degrees(-5)
                            while self.sensor_distance > 22:
                                self.vehicle.go_straight()
                                if self.sensor_distance == 22:
                                    self.vehicle.stop()
                

                    
def _stop(self):
    pass

