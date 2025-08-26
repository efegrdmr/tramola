#!/usr/bin/env python2
import Jetson.GPIO as GPIO
import time
import sys

LED_Pin = 11

def setup():
    GPIO.setmode(GPIO.BOARD)
    GPIO.setup(LED_Pin, GPIO.OUT)
    print("GPIO setup complete.")

def cleanup():
    GPIO.cleanup()
    print("GPIO cleanup complete.")
    
def turn_on():
    try:
        GPIO.output(LED_Pin, GPIO.HIGH)
        print("LED turned ON")
    except Exception as e:
        print("Error turning LED on: {}".format(e))
        return False
    return True

def turn_off():
    try:
        GPIO.output(LED_Pin, GPIO.LOW)
        print("LED turned OFF")
    except Exception as e:
        print("Error turning LED off: {}".format(e))
        return False
    return True

def show_instructions():
    print("\n--- LED Control Instructions ---")
    print("Press 'q' to turn the LED ON")
    print("Press 'w' to turn the LED OFF")
    print("Press 'x' to exit")
    print("-------------------------------")

def main():
    try:
        print("Starting program...")
        setup()
        show_instructions()
        
        while True:
            print("Waiting for command...")
            choice = raw_input("Enter command: ")
            print("You entered: '{}'".format(choice))
            
            if choice == "q":
                result = turn_on()
                print("Turn on result: {}".format(result))
            elif choice == "w":
                result = turn_off()
                print("Turn off result: {}".format(result))
            elif choice == "x":
                print("Exiting program...")
                break
            else:
                print("Invalid command. Use 'q', 'w', or 'x'")
            
            print("Command processed, continuing...")
                
    except KeyboardInterrupt:
        print("\nProgram interrupted by user")
    except Exception as e:
        print("Unexpected error: {}".format(e))
    finally:
        print("Cleaning up...")
        cleanup()
        print("Exiting program.")
        sys.exit(0)

if __name__ == "__main__":
    main()