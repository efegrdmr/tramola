import serial
import time


# Configuration values (adjust these to your environment)
SERIAL_PORT = '/dev/ttyUSB0'  # On Linux or macOS this might be '/dev/ttyUSB0' or similar
BAUD_RATE = 9600
TIMEOUT = 1.0  # Timeout for serial read operations
RESPONSE_TIMEOUT = 2000  # Timeout for response wait in milliseconds


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
            if packet:
                # Call the callback function with the processed response
                response = self.message_callback(packet)
                if response:
                    # send the result
                    self.send_message(response)


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
        checksum = self.calculate_checksum(packet)
        checksum_bytes = checksum.to_bytes(2, byteorder='big')
        packet += checksum_bytes + '\n'.encode('utf-8')
        self.serial_port.write(packet)
        
    
    def send_message_and_wait_for_response(self, message):
        # Wait for response at most RESPONSE_TIMEOUT milliseconds
        response = None
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
        return checksum & 0xFFFF

    
    def read_packet(self):
        """
        Reads from the serial port until a '\n' delimiter or timeout.
        Returns a complete packet (excluding the '\n' and checksum) as a bytearray,
        or None on timeout, checksum failure, or malformed data.
        """
        start = time.time()
        buffer = bytearray()

        while time.time() < start + TIMEOUT:
            if self.serial_port.in_waiting < 1:
                time.sleep(0.01)
                continue
            byte = self.serial_port.read(1)
            if byte == b'\n':
                # End-of-packet delimiter
                break
            buffer += byte

        # Must have at least 2 bytes for checksum
        if len(buffer) < 2:
            return None
        # Split out and verify checksum
        packet_checksum = int.from_bytes(buffer[-2:], byteorder="big")
        data = buffer[:-2]
        if self.calculate_checksum(data) != packet_checksum:
            return None

        # Decode and return payload
        return self.decode_message(data)


import time

class LoraGCSClient:
    def __init__(self):
        self.pq = []
        self.requested_datas = {"speed_real", "heading", "yaw_real", "thruster_requested"
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
        self.lora = Lora()

  
    def send_message(self, message):
        return self.lora.send_message_and_wait_for_response(message)
        

    def start_mission(self):
        return self.send_message("start_mission")
        

    def emergency_shutdown(self):
        return self.send_message("emergency_shutdown")


    def add_waypoint(self, latitude, longitude):
        message = f"add_waypoint,{latitude},{longitude}"
        return self.send_message(message)


    def start_manual_control(self):
        self.stop_data_requests()
        response = self.send_message("start_manual_control")
        return response
    

    def send_manuel_request(self):
        self.send_message(f"manual_control,{self.manual_speed},{self.manual_yaw}")


    def sync_data(self):
        for message in self.requested_datas:
            response = self.lora.send_message_and_wait_for_response(message)
            if response:
                if message == "location":
                    self.location = response.split(",")
                    self.location = (float(self.location[0]), float(self.location[1]))
                elif message == "speed_real":
                    self.speed_real = float(response)
                elif message == "heading":
                    self.heading = float(response)
                elif message == "yaw_real":
                    self.yaw_real = float(response)
                elif message == "thruster_requested":
                    self.thruster_requested = float(response)
                elif message == "speed_requested":
                    self.speed_requested = float(response)
                elif message == "yaw_requested":
                    self.yaw_requested = float(response)
            time.sleep(0.5)
                