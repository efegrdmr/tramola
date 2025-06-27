#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python 2 compatible LoRa communication module
# Date: 2025-06-25

import serial
import time
import threading

class Lora(object):
    def __init__(self, port='/dev/ttyUSB0', baud_rate=9600, timeout=1.0, response_timeout=5000, message_callback=None):
        try:
            self.serial_port = serial.Serial(port, baud_rate, timeout=timeout)
            self.response_timeout = response_timeout
            self.message_callback = message_callback
            self.running = False
            self.receiver_thread = None
            
        except serial.SerialException as e:
            raise Exception("Failed to open serial port: %s" % e)
    
    def set_message_callback(self, callback):
        self.message_callback = callback
        
    def start_receiver(self):
        if self.receiver_thread and self.running:
            return  # Already running
            
        self.running = True
        self.receiver_thread = threading.Thread(target=self._receiver_loop)
        self.receiver_thread.daemon = True
        self.receiver_thread.start()
    
    def stop_receiver(self):
        self.running = False
        if self.receiver_thread:
            self.receiver_thread.join(timeout=2.0)
            self.receiver_thread = None
    
    def _receiver_loop(self):
        while self.running:
            try:
                # Read a packet from the serial port
                packet = self.read_packet()
                if packet:
                    print(packet + "  came")
                if packet and self.message_callback:
                    # Call the callback function with the processed response
                    response = self.message_callback(packet)
                    if response:
                        # Send the result
                        time.sleep(0.02)
                        self.send_message(response)
            except Exception as e:
                print("Error in receiver loop:", e)
            time.sleep(0.01)  # Small delay to prevent CPU hogging

    def encode_message(self, message):
        if isinstance(message, unicode):
            return bytearray(message.encode('utf-8'))
        return bytearray(message)
    
    def decode_message(self, message):
        try:
            return str(message)
        except UnicodeDecodeError:
            return None
        
    def send_message(self, message):
        """
        Sends data over the serial port without checksum verification.
        The data should be a byte array.
        """
        try:
            packet = self.encode_message(message)
            # Just append newline - no checksum
            packet.append(ord('\n'))
            # Send the raw bytes
            self.serial_port.write(bytes(packet))
            print("message sent: " + message)
            return True
        except Exception as e:
            print("Error sending message:", e)
            return False
    
    def send_message_and_wait_for_response(self, message):
        # Wait for response at most RESPONSE_TIMEOUT milliseconds
        if not self.send_message(message):
            return None
            
        response = None
        start_time = time.time()
        while (time.time() - start_time) * 1000 < self.response_timeout:
            response = self.read_packet()
            if response:
                break
            time.sleep(0.01)  # Small delay to prevent CPU hogging
        return response
            
    def calculate_checksum(self, data):
        """
        Calculates a simple XOR-based checksum for the given data.
        """
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum & 0xFFFF
    
    def read_packet(self):
        """
        Reads from the serial port until a '\n' delimiter or timeout.
        Returns the complete packet as a bytearray (excluding the '\n'),
        or None on timeout or malformed data.
        """
        try:
            start = time.time()
            buffer = bytearray()

            while time.time() < start + self.serial_port.timeout:
                if self.serial_port.inWaiting() < 1:
                    time.sleep(0.01)
                    continue
                byte = self.serial_port.read(1)
                if byte == '\n' or byte == b'\n':
                    # End-of-packet delimiter found
                    break
                buffer.append(ord(byte) if isinstance(byte, str) else byte)

            # Return if we have any data
            if len(buffer) > 0:
                return self.decode_message(buffer)
            return None
        except Exception as e:
            print("Error reading packet:", e)
            return None
            
    def close(self):
        """Close the serial port and clean up resources"""
        self.stop_receiver()
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()


class LoraGCSClient(object):
    def __init__(self, lora):
        self.requested_datas = {"speed_real", "heading", "yaw_real", "thruster_requested",
                           "speed_requested", "yaw_requested", "location"}
        
        self.location = None
        self.speed_real = 0
        self.heading = 0
        self.yaw_real = 0
        self.thruster_requested = 0
        self.speed_requested = 0
        self.yaw_requested = 0

        self.manual_speed = 0
        self.manual_yaw = 0
        self.lora = lora
        self.data_request_running = False
        self.data_request_thread = None
  
    def send_message(self, message):
        return self.lora.send_message_and_wait_for_response(message)
        
    def start_mission(self):
        return self.send_message("start_mission")
        
    def emergency_shutdown(self):
        return self.send_message("emergency_shutdown")

    def add_waypoint(self, latitude, longitude):
        message = "add_waypoint,%f,%f" % (latitude, longitude)
        return self.send_message(message)

    def start_data_requests(self):
        """Start a thread to periodically request data updates"""
        if self.data_request_thread and self.data_request_running:
            return  # Already running
            
        self.data_request_running = True
        self.data_request_thread = threading.Thread(target=self._data_request_loop)
        self.data_request_thread.daemon = True
        self.data_request_thread.start()
    
    def stop_data_requests(self):
        """Stop the data request thread"""
        self.data_request_running = False
        if self.data_request_thread:
            self.data_request_thread.join(timeout=2.0)
            self.data_request_thread = None
    
    def _data_request_loop(self):
        """Thread function to continuously request data updates"""
        while self.data_request_running:
            self.sync_data()
            time.sleep(1.0)  # Request update every second

    def sync_data(self):
        """Request updates for all data values"""
        for message in self.requested_datas:
            try:
                response = self.lora.send_message_and_wait_for_response(message)
                if response:
                    if message == "location":
                        parts = response.split(",")
                        if len(parts) >= 2:
                            try:
                                self.location = (float(parts[0]), float(parts[1]))
                            except ValueError:
                                print("Invalid location format:", response)
                    elif message == "speed_real":
                        try:
                            self.speed_real = float(response)
                        except ValueError:
                            print("Invalid speed_real format:", response)
                    elif message == "heading":
                        try:
                            self.heading = float(response)
                        except ValueError:
                            print("Invalid heading format:", response)
                    elif message == "yaw_real":
                        try:
                            self.yaw_real = float(response)
                        except ValueError:
                            print("Invalid yaw_real format:", response)
                    elif message == "thruster_requested":
                        try:
                            data = response.split(",")
                            self.thruster_requested = (float(data[0]), float(data[1]))
                        except ValueError:
                            print("Invalid thruster_requested format:", response)
                    elif message == "speed_requested":
                        try:
                            self.speed_requested = float(response)
                        except ValueError:
                            print("Invalid speed_requested format:", response)
                    elif message == "yaw_requested":
                        try:
                            self.yaw_requested = float(response)
                        except ValueError:
                            print("Invalid yaw_requested format:", response)
            except Exception as e:
                print("Error syncing data for %s: %s" % (message, e))
            time.sleep(0.2)  # Small delay between requests to prevent flooding
    
    def close(self):
        """Clean up resources"""
        self.stop_data_requests()