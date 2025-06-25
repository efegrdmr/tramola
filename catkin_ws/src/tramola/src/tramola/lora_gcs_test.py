#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
LoRa GCS (Ground Control Station) program for testing communication.
Run this on the computer that will act as the control station.
Python 2 compatible version.
"""
import time
import signal
import sys
from loralib import Lora, LoraGCSClient

# Global flag for clean shutdown
running = True

def signal_handler(sig, frame):
    """Handle Ctrl+C to clean up resources"""
    global running
    print("Shutting down GCS...")
    running = False

def main():
    # Parse command line arguments
    import argparse
    parser = argparse.ArgumentParser(description="LoRa Ground Control Station Test Program")
    parser.add_argument("--port", type=str, default="/dev/ttyUSB0", help="Serial port for LoRa device")
    parser.add_argument("--baud", type=int, default=9600, help="Baud rate")
    args = parser.parse_args()
    
    print("GCS: Starting with port %s at %d baud" % (args.port, args.baud))
    
    try:
        # Initialize the Lora communication
        lora = Lora(
            port=args.port,
            baud_rate=args.baud,
            timeout=1.0,
            response_timeout=2000
        )
        
        # Initialize the GCS client
        client = LoraGCSClient(lora)
        
        # Register signal handler for clean exit
        signal.signal(signal.SIGINT, signal_handler)
        
        print("GCS: Running communication tests...")
        
        # Test 1: Request basic data
        print("\nTest 1: Basic data request")
        print("Requesting location data...")
        client.sync_data()
        print("Location:", client.location)
        print("Speed:", client.speed_real)
        print("Heading:", client.heading)
        
        # Test 2: Add waypoints
        print("\nTest 2: Add waypoints")
        waypoints = [
            (37.7749, -122.4194),  # San Francisco
            (37.8716, -122.2727),  # Berkeley
        ]
        
        for lat, lon in waypoints:
            print("Adding waypoint: %f, %f" % (lat, lon))
            response = client.add_waypoint(lat, lon)
            print("Response:", response)
            time.sleep(1)
        
        
        # Test 4: Mission control
        print("\nTest 4: Mission control")
        print("Starting mission...")
        response = client.start_mission()
        print("Response:", response)
        
        # Let mission run for a while
        print("Mission running for 2 seconds...")
        time.sleep(2)
        
        # Emergency shutdown
        print("Sending emergency shutdown...")
        response = client.emergency_shutdown()
        print("Response:", response)
        
        # Interactive mode
        print("\nEntering interactive mode (type 'exit' to quit)")
        while running:
            cmd = raw_input("GCS> ")  # Python 2 uses raw_input instead of input
            if cmd.lower() in ["exit", "quit", "q"]:
                break
                
            if cmd == "help":
                print("Available commands:")
                print("  sync - Request all data from vehicle")
                print("  location - Show current location")
                print("  waypoint lat lon - Add waypoint at lat,lon")
                print("  auto - Exit manual control mode")
                print("  mission - Start mission")
                print("  shutdown - Emergency shutdown")
                print("  exit/quit - Exit program")
                continue
                
            if cmd == "sync":
                client.sync_data()
                print("Location:", client.location)
                print("Speed:", client.speed_real)
                print("Heading:", client.heading)
                print("Yaw:", client.yaw_real)
                continue
                
            if cmd == "location":
                print("Location:", client.location)
                continue
                
            if cmd.startswith("waypoint "):
                parts = cmd.split()
                if len(parts) >= 3:
                    try:
                        lat = float(parts[1])
                        lon = float(parts[2])
                        response = client.add_waypoint(lat, lon)
                        print("Response:", response)
                    except ValueError:
                        print("Invalid coordinates. Use: waypoint lat lon")
                else:
                    print("Invalid command. Use: waypoint lat lon")
                continue
                
                
            if cmd == "mission":
                response = client.start_mission()
                print("Response:", response)
                continue
                
            if cmd == "shutdown":
                response = client.emergency_shutdown()
                print("Response:", response)
                continue
                
            # Custom command
            if cmd:
                response = client.send_message(cmd)
                print("Response:", response)
    
    except Exception as e:
        print("Error:", e)
    finally:
        # Clean up resources
        if 'client' in locals():
            client.close()
        if 'lora' in locals():
            lora.close()
        print("GCS: Shutdown complete")

if __name__ == "__main__":
    main()