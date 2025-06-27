#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LoRa Vehicle program for testing communication.
Run this on the computer that will simulate the vehicle.
Python 2 compatible version.
"""
import time
import signal
import sys
import math
from loralib import Lora

# Global flag for clean shutdown
running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C to clean up resources"""
    global running
    print("Shutting down vehicle...")
    running = False

class Vehicle(object):
    """Represents the vehicle-side logic for LoRa communication"""
    def __init__(self, lora):
        self.lora = lora
        self.location = (37.7749, -122.4194)  # Default location (San Francisco)
        self.speed_real = 0.0
        self.heading = 90.0  # East
        self.yaw_real = 0.0
        self.thruster_requested = 0.0
        self.speed_requested = 0.0
        self.yaw_requested = 0.0
        
        self.is_in_mission = False
        self.is_in_manual_control = False
        self.waypoints = []
        
        # Set up the message handler
        self.lora.set_message_callback(self.handle_message)
        
    def handle_message(self, message):
        """Handle incoming messages from the GCS"""
        print("Vehicle received:", message)
        
        # Process command messages
        if message == "start_mission":
            self.is_in_mission = True
            self.is_in_manual_control = False
            print("Mission started")
            return "mission_started"
            
        elif message == "emergency_shutdown":
            self.is_in_mission = False
            self.is_in_manual_control = False
            self.speed_requested = 0.0
            self.yaw_requested = 0.0
            self.thruster_requested = 0.0
            print("Emergency shutdown executed")
            return "shutdown_complete"
            
        elif message.startswith("add_waypoint"):
            parts = message.split(",")
            if len(parts) >= 3:
                try:
                    lat = float(parts[1])
                    lon = float(parts[2])
                    self.waypoints.append((lat, lon))
                    print("Waypoint added: %f, %f" % (lat, lon))
                    return "waypoint_added:%d" % len(self.waypoints)
                except ValueError:
                    return "error:invalid_waypoint_format"
                    
        # Handle data request messages
        elif message == "location":
            return "%f,%f" % (self.location[0], self.location[1])
            
        elif message == "speed_real":
            return str(self.speed_real)
            
        elif message == "heading":
            return str(self.heading)
            
        elif message == "yaw_real":
            return str(self.yaw_real)
            
        elif message == "thruster_requested":
            return str(self.thruster_requested)
            
        elif message == "speed_requested":
            return str(self.speed_requested)
            
        elif message == "yaw_requested":
            return str(self.yaw_requested)
            
        return "error:unknown_command"
    
            

def main():
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description="LoRa Vehicle Test Program")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port for LoRa device")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate")
    args = parser.parse_args()
    
    print("Vehicle: Starting with port %s at %d baud" % (args.port, args.baud))
    
    try:
        # Initialize the Lora communication
        lora = Lora(
            port=args.port,
            baud_rate=args.baud,
            timeout=1.0,
            response_timeout=2000
        )
        
        # Start the receiver
        lora.start_receiver()
        
        # Initialize the vehicle
        vehicle = Vehicle(lora)
        
        # Register signal handler for clean exit
        signal.signal(signal.SIGINT, signal_handler)
        
        print("Vehicle: Ready to receive commands...")
        
        
        while running:
            # Update vehicle simulation
            #             
            # Print status periodically
            
            # Small delay to prevent CPU hogging
            time.sleep(0.1)
    
    except Exception as e:
        print("Error:", e)
    finally:
        # Clean up resources
        if 'lora' in locals():
            lora.close()
        print("Vehicle: Shutdown complete")

if __name__ == "__main__":
    main()