import rclpy
import rclpy.logging
from rclpy.node import Node
from std_msgs.msg import Float64

MAX_SPEED = 2000.0

class Thrusters():
    def __init__(self, node):
        self.publisherLeft = node.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.publisherRight = node.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.timer = node.create_timer(1, self.timer_callback)
        self.leftSpeed = 0.0
        self.rightSpeed = 0.0
        self.speed = 0.0  # Initialize speed
        self.turning_factor = 0.2  # Adjust this value to control the sharpness of turns

    def timer_callback(self):
        msg = Float64()
        msg.data = self.leftSpeed
        self.publisherLeft.publish(msg)

        msg.data = self.rightSpeed
        self.publisherRight.publish(msg)

    def setSpeedLeft(self, speed):
        if speed < -MAX_SPEED:
            self.leftSpeed = -MAX_SPEED
        else:
            self.leftSpeed = min(speed, MAX_SPEED)
        self.timer_callback()

    def setSpeedRight(self, speed):
        if speed < -MAX_SPEED:
            self.rightSpeed = -MAX_SPEED
        else:
            self.rightSpeed = min(speed, MAX_SPEED)
        self.timer_callback()

    def increaseSpeed(self):
        self.speed = min(self.speed + MAX_SPEED / 5, MAX_SPEED)
        self.setSpeedLeft(self.speed)
        self.setSpeedRight(self.speed)

    def decreaseSpeed(self):
        self.speed = max(self.speed - MAX_SPEED / 5, -MAX_SPEED)
        self.setSpeedLeft(self.speed)
        self.setSpeedRight(self.speed)

    def turnLeft(self):
        # Adjust the speed difference based on the current speed and the turning factor
        turn_difference = self.speed * self.turning_factor
        self.setSpeedLeft(self.speed - turn_difference)
        self.setSpeedRight(self.speed + turn_difference)

    def turnRight(self):
        # Adjust the speed difference based on the current speed and the turning factor
        turn_difference = self.speed * self.turning_factor
        self.setSpeedLeft(self.speed + turn_difference)
        self.setSpeedRight(self.speed - turn_difference)
