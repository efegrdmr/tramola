#!/home/tramola/vision/bin/python3.8

from tramola.vehicle import Vehicle
import time

if __name__ == '__main__':
    vehicle = Vehicle()
    
    vehicle.go_straight(0.1)
    time.sleep(2)
    vehicle.go_left(0.1)
    time.sleep(2)
    vehicle.go_right(0.1)
    time.sleep(2)
    vehicle.stop()