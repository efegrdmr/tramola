#!/usr/bin/env python

from tramola.vehicle import Vehicle
import time

if __name__ == '__main__':
    vehicle = Vehicle()
    
    vehicle.go_straight(throttle=1700)
    time.sleep(2)
    vehicle.go_left(throttle=1600)
    time.sleep(2)
    vehicle.go_right(throttle=1600)
    time.sleep(2)
    vehicle.stop()