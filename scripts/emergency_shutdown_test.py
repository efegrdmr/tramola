import Jetson.GPIO as GPIO
import time as time

LED_Pin = 11

GPIO.setmode(GPIO.BOARD)
GPIO.setup(LED_Pin, GPIO.OUT)

while (True):
   GPIO.output(LED_Pin, GPIO.HIGH)
   time.sleep(0.5)
   GPIO.output(LED_Pin, GPIO.LOW)
   time.sleep(0.5)
GPIO.cleanup()