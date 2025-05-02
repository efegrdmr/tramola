import serial
import time


# Configuration values (adjust these to your environment)
SERIAL_PORT = '/dev/ttyUSB0'  # On Linux or macOS this might be '/dev/ttyUSB0' or similar
BAUD_RATE = 9600
TIMEOUT = 1  # Timeout for serial read operations
RESPONSE_TIMEOUT = 500  # Timeout for response wait in milliseconds


class Lora:
    def __init__(self, message_callback=None):
        self.serial_port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        self.message_callback = message_callback
        if message_callback:
            self.start_receiver()
    
    def set_message_callback(self, callback):
        self.message_callback = callback

    def start_receiver(self):
        while True:
            # Read a packet from the serial port
            packet = self.read_packet()
            if not packet:
                continue  # No packet received, continue to read
            # Process the received packet
            print("Got packet:", packet.decode('utf-8'))
            if packet:
                # Call the callback function with the processed response
                response = self.message_callback(packet)
                if response:
                    # send the result
                    self.send(response)

    def encode_message(self, message):
        return bytearray(message.encode('utf-8'))
    
    def decode_message(self, message):
        return message.decode('utf-8')
        
    def send_message(self, message):
        """
        Sends data over the serial port.
        The data should be a byte array.
        """
        packet = self.encode_message(message)
        checksum = self.calculate_checksum(packet) & 0xFFFF
        checksum_bytes = checksum.to_bytes(2, byteorder='big')
        packet += checksum_bytes
        self.serial_port.write(packet)
        
    
    def send_message_and_wait_for_response(self, message):
        # Wait for response at most RESPONSE_TIMEOUT milliseconds
        self.send_message(message)
        start_time = time.time()
        while (time.time() - start_time) * 1000 < RESPONSE_TIMEOUT:
            response = self.read_packet()
            if response:
                break
        return response
            
    def calculate_checksum(self, data):
        """
        Calculates a simple XOR-based checksum for the given data.
        """
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    
    
    def read_packet(self):
        """
        Reads from the serial port
        Returns a complete packet as a bytearray.
        """
        buffer = bytearray()
        while True:
            # Read a single byte from serial (blocking with timeout)
            byte = self.serial_port.read(1)
            if len(byte) == 0:
                # Timeout reached without receiving new byte
                break
            buffer += byte
        if len(buffer) == 0:
            return None

        packet_checksum = int.from_bytes(buffer[-2:], byteorder="big")
        if self.calculate_checksum(buffer[:-2]) != packet_checksum:
            return None
        packet = self.decode_message(buffer[:-2])
        return packet
    

import time
import threading

class LoraGCSClient:
    def __init__(self):
        self.pq = []
        self.requested_datas = {"state", "latitude", "longitude", "degree_from_north",
                           "speed"}
        
        self.state = None
        self.latitude = None
        self.longitude = None
        self.degree_from_north = None
        self.speed = None
        self.manual_speed = 0
        self.manual_yaw = 0
        self.waypoints = []
        self.lora = Lora()
        self.data_thread = None
        self.manual_control_thread = None
        self.manual_control = False
        self.getData = False
        self.start_data_requests()

    def start_data_requests(self):
        if self.getData:
            return
        
        self.getData = True
        self.data_thread = threading.Thread(target=self.data_loop, daemon=True)
        self.data_thread.start()

    def stop_data_requests(self):
        if not self.getData:
            return
        self.getData = False
        self.data_thread.join()
        self.data_thread = None

    def send_message(self, message):
        getData = self.getData
        if self.getData:
            self.stop_data_requests()
        response = self.lora.send_message_and_wait_for_response(message)
        if getData:
            self.start_data_requests()
        return response

    def start_mission(self):
        return self.send_message("start_mission")
        

    def add_waypoint(self, lat, long):
        self.waypoints.append((lat, long))
        message = f"add_waypoint,{lat},{long}"
        return self.send_message(message)
        

    def emergency_shutdown(self):
        return self.send_message("emergency_shutdown")

    def start_manual_control(self):
        self.stop_data_requests()
        response = self.send_message("start_manual_control")
        self.manual_control_thread = threading.Thread(target=self.manual_control_loop, daemon=True)
        self.manual_control_thread.start()
        return response

    def stop_manual_control(self):
        self.manual_control = False
        self.manual_control_thread.join()
        self.manual_control_thread = None
        self.start_data_requests()


    def data_loop(self):
        while True:
            for message in self.requested_datas:
                if not self.getData:
                    return
                response = self.lora.send_message_and_wait_for_response(message)
                if response:
                    if message == "state":
                        self.state = response
                    elif message == "latitude":
                        self.latitude = float(response)
                    elif message == "longitude":
                        self.longitude = float(response)
                    elif message == "degree_from_north":
                        self.degree_from_north = int(response)
                    elif message == "speed":
                        self.speed = float(response)
                
                
    def manual_control_loop(self):
        while self.manual_control:
            self.send_message(f"manual_control,{self.manual_speed},{self.manual_yaw}")





        

            
