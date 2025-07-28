#!/usr/bin/env python
# -*- coding: utf-8 -*-
# Python 2 compatible LoRa communication module
# Date: 2025-06-25

import serial
import time
import threading

class Lora(object):
    def __init__(self, port="/dev/ttyUSB0", baud_rate=9600, timeout=1.0, response_timeout=2000, message_callback=None):
        try:
            self.serial_port = serial.Serial(port, baud_rate, timeout=timeout)
            self.response_timeout = response_timeout
            self.message_callback = message_callback
            self.running = False
            self.receiver_thread = None
            
        except serial.SerialException as e:
            raise Exception("Failed to open serial port: %s" % e)

        self.coding_of_messages = {
            "start_mission": "0",
            "emergency_shutdown": "1",
            "add_waypoint": "2",
            "location": "3",
            "speed_real": "4",
            "heading": "5",
            "yaw_real": "6",
            "thruster_requested": "7",
            "speed_requested": "8",
            "yaw_requested": "9",
            "OK": "a",
            "ERR": "b",
            "start_manual_mode" : "c",
            "stop_manual_mode" : "d",
            "manual" : "e",
            "state" : "f",
            "set_color" : "g"}
        
        self.message_of_codings = {v: k for k, v in self.coding_of_messages.items()}

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
    
    def encode(self, message):
        data = message.split(",")
        data[0] = self.coding_of_messages.get(data[0], data[0])
        return ",".join(data)
    def decode(self, message):
        data = message.split(",")
        data[0] = self.message_of_codings.get(data[0], data[0])
        return ",".join(data)

    def _receiver_loop(self):
        while self.running:
            try:
                # Read a packet from the serial port
                packet = self.read_packet()
                if packet and self.message_callback:
                    # Call the callback function with the processed response
                    packet = self.decode(packet)
                    response = self.message_callback(packet)
                    if response:
                        # Send the result
                        self.send_message(response)
            except Exception as e:
                print("Error in receiver loop:", e)
            time.sleep(0.01)  # Small delay to prevent CPU hogging

        
    def calculate_checksum(self, data):
        """
        Calculates a simple XOR-based checksum for the given data (str or bytearray).
        Returns a 16-bit integer.
        """
        checksum = 0
        # Ensure data is iterable of byte values
        if isinstance(data, (str, unicode)):
            data = bytearray(data)
        for b in data:
            checksum ^= b
        return checksum & 0xFFFF

    def send_message(self, message):
        """
        Sends data over the serial port with an appended checksum.
        Message is a str or unicode in Python 2.
        Format: payload|<checksum>\n
        Returns True on success, False on error.
        """
        try:
            # Build payload string
            if isinstance(message, unicode):
                payload = message.encode("utf-8")
            else:
                payload = str(message)

            # Calculate checksum over raw bytes
            checksum = self.calculate_checksum(payload)
            full_msg = payload + "|" + str(checksum) + "\n"

            # Write to serial port
            self.serial_port.write(full_msg)
            return True
        except Exception as e:
            print("Error sending message:", e)
            return False

    def send_message_and_wait_for_response(self, message):
        """
        Sends a message and waits up to response_timeout for a response packet.
        Returns decoded string or None on timeout/error.
        """
        if not self.send_message(message):
            return None

        deadline = time.time() + (self.response_timeout / 1000.0)
        while time.time() < deadline:
            resp = self.read_packet()
            if resp is not None:
                return resp
            time.sleep(0.01)
        return None

    def read_packet(self):
        """
        Reads from serial until "\n". Expects format payload|<checksum>\n.
        Verifies checksum, returning payload or None if invalid.
        """
        try:
            start = time.time()
            timeout = self.serial_port.timeout
            buffer = []

            # Read until newline or timeout
            while time.time() - start < timeout:
                if self.serial_port.inWaiting() > 0:
                    char = self.serial_port.read(1)
                    if not char:
                        continue
                    if char == "\n":
                        break
                    buffer.append(char)
                else:
                    time.sleep(0.01)

            if not buffer:
                return None

            # Join the buffer only once, at the end
            buffer_str = ''.join(buffer)

            # Split payload and checksum
            if "|" not in buffer_str:
                return None
            payload_str, checksum_str = buffer_str.rsplit("|", 1)
            try:
                recv_checksum = int(checksum_str)
            except ValueError:
                return None

            # Verify checksum
            calc_checksum = self.calculate_checksum(payload_str)
            if recv_checksum != calc_checksum:
                print("Checksum mismatch: received %d, calculated %d" % (recv_checksum, calc_checksum))
                return None
            
            return payload_str
        except Exception as e:
            print("Error reading packet:", e)
            return None
            
    def close(self):
        """Close the serial port and clean up resources"""
        self.stop_receiver()
        if self.serial_port and self.serial_port.isOpen():
            self.serial_port.close()


from actionlib_msgs.msg import GoalStatus
class LoraGCSClient(object):
    def __init__(self, lora):
        self.requested_datas = {"speed_real", "heading", "yaw_real", "thruster_requested",
                           "speed_requested", "yaw_requested", "location", "state"}
        
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
        self.state = ""

  
    def send_message(self, message, retry_count=0):
        message = self.lora.encode(message)
        res = self.lora.send_message_and_wait_for_response(message)
        if res is None and retry_count > 0:
            res = self.send_message(message, retry_count-1)
        return res
    
    def start_manual_mode(self):
        self.stop_data_requests()
        self.state = "MANUAL"
        res = self.send_message("start_manual_mode")
        if res == "OK":
            return res
        self.start_data_requests()
        return "ERR"

    def stop_manual_mode(self,):
        res = self.send_message("stop_manual_mode", 5)
        if res == "OK":
            self.start_data_requests()
            return "OK"
        return "ERR"

    def send_manual_control_request(self, speed_normalized, yaw_normalized):
        message = f"manual,%.2f,%.2f" % speed_normalized, yaw_normalized
        self.lora.send_message(message)
    
    def set_objective_color(self, color):
        if color == "RED":
            color = 0
        elif color == "GREEN":
            color = 1
        elif color == "BLACK":
            color = 2
        else:
            raise Exception("Error in color")
        
        self.stop_data_requests()  # Ensure data requests are stopped before starting mission
        res = self.send_message(f"set_color,%d" % color, 5)
        self.start_data_requests()  # Start data requests after mission starts
        return res

        

    def start_mission(self):
        self.stop_data_requests()  # Ensure data requests are stopped before starting mission
        res = self.send_message("start_mission", 5)
        self.start_data_requests()  # Start data requests after mission starts
        return res
        
    def emergency_shutdown(self):
        self.stop_data_requests()  # Ensure data requests are stopped before starting mission
        res = self.send_message("emergency_shutdown", 5)
        self.start_data_requests()  # Start data requests after mission starts
        return res

    def add_waypoint(self, latitude, longitude):
        self.stop_data_requests()  # Ensure data requests are stopped before adding waypoint
        message = "add_waypoint,%f,%f" % (latitude, longitude)
        res = self.send_message(message, 5)
        self.start_data_requests()  # Restart data requests after adding waypoint
        return res

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
            self.data_request_thread.join()
            self.data_request_thread = None

    def _process_state_message(self, message):
        data = message.split(",")
        if data[0] != "GOTO":
            return message + " " + self._get_status_text(int(data[1]))


    def _get_status_text(status_int):
        """Converts an actionlib status integer to its string representation."""
        status_map = {
            GoalStatus.PENDING: "PENDING",
            GoalStatus.ACTIVE: "ACTIVE",
            GoalStatus.PREEMPTED: "PREEMPTED",
            GoalStatus.SUCCEEDED: "SUCCEEDED",
            GoalStatus.ABORTED: "ABORTED",
            GoalStatus.REJECTED: "REJECTED",
            GoalStatus.PREEMPTING: "PREEMPTING",
            GoalStatus.RECALLING: "RECALLING",
            GoalStatus.RECALLED: "RECALLED",
            GoalStatus.LOST: "LOST",
        }
        return status_map.get(status_int, "UNKNOWN_STATUS")

    
    def _data_request_loop(self):
        """Thread function to continuously request data updates"""
        while self.data_request_running:
            self.sync_data()

    def sync_data(self):
        """Request updates for all data values"""
        for message in self.requested_datas:
            if not self.data_request_running:
                return
            try:
                response = self.send_message(message)
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
                    elif message == "state":
                        try:
                            self.state = self._process_state_message(response)
                        except ValueError:
                            print("Invalid state message")
            except Exception as e:
                print("Error syncing data for %s: %s" % (message, e))
    
    def close(self):
        """Clean up resources"""
        self.stop_data_requests()