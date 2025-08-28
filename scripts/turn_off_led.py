#!/usr/bin/env python2
import Jetson.GPIO as GPIO
import sys

PIN = 11  # Board pin 11 (GPIO17)

GPIO.setwarnings(False)  # Disable warnings
GPIO.setmode(GPIO.BOARD)
GPIO.setup(PIN, GPIO.OUT)
GPIO.output(PIN, GPIO.LOW)

sys.exit(0)  # Exit with success status code