import serial
import time


# Configuration values (adjust these to your environment)
SERIAL_PORT = 'COM3'  # On Linux or macOS this might be '/dev/ttyUSB0' or similar
BAUD_RATE = 9600
TIMEOUT = 1  # Timeout for serial read operations
RESPONSE_TIMEOUT = 300  # Timeout for response wait in milliseconds

# Frame delimiters and constants
START_DELIMITER = 0x7E
END_DELIMITER = 0x7F
MIN_PACKET_SIZE = 4  # Start + Length + Checksum + End (plus payload)

class Lora:
    def __init__(self, reciever=False, reveiver_callback=None):
        self.serial_port = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=TIMEOUT)
        self.reveiver_callback = reveiver_callback
        if reciever:
            self.start_receiver()

    def start_receiver(self):
        while True:
            # Read a packet from the serial port
            packet = self.read_packet()
            if not packet:
                continue  # No packet received, continue to read
            # Process the received packet
            packet = self.process_received_packet(packet)
            if packet and self.reveiver_callback:
                # Call the callback function with the processed response
                response = self.reveiver_callback(packet)
                if response:
                    # send the result
                    self.send(response, tries=0)

                
        
    def send(self, data, tries=3):
        """
        Sends data over the serial port.
        The data should be a byte array.
        Returns the response as a byte array.
        """
        packet = None
        # Prepend start delimiter and append end delimiter
        packet = bytearray([0x7E]) + data + bytearray([0x7F])
        self.serial_port.write(packet)
        # Wait for response at most RESPONSE_TIMEOUT milliseconds
        start_time = time.time()
        while (time.time() - start_time) * 1000 < RESPONSE_TIMEOUT:
            response = self.read_packet()
            if response:
                # Process the received packet
                response = self.process_received_packet(response)
                if response:
                    break
        
        if not response and tries > 0:
            return self.send(data, tries - 1)
        elif not response:
            return None

        return response
            
    def calculate_checksum(self, data):
        """
        Calculates a simple XOR-based checksum for the given data.
        """
        checksum = 0
        for byte in data:
            checksum ^= byte
        return checksum

    def process_received_packet(self,packet):
        """
        Process and validate a packet according to:
        [START_DELIMITER][LENGTH][PAYLOAD...][CHECKSUM][END_DELIMITER]
        Returns the payload if valid; otherwise, returns None.
        """
        packet_size = len(packet)

        # Check for minimum packet size
        if packet_size < MIN_PACKET_SIZE:
            print("Packet too short.")
            return None

        # Verify start and end delimiters
        if packet[0] != START_DELIMITER or packet[-1] != END_DELIMITER:
            print("Start or end delimiter not found.")
            return None

        # Read payload length
        payload_length = packet[1]

        # Expected length = start + length byte + payload + checksum + end
        if packet_size != payload_length + 4:
            print("Packet length mismatch.")
            return None

        # Retrieve payload and checksum
        payload = packet[2:2+payload_length]
        received_checksum = packet[2+payload_length]

        # Validate checksum
        if self.calculate_checksum(payload) != received_checksum:
            print("Checksum error.")
            return None

        return payload
    
    def read_packet(self):
        """
        Reads from the serial port until a complete packet is found
        or the function times out. Returns a complete packet as a bytearray.
        """
        buffer = bytearray()

        while True:
            # Read a single byte from serial (blocking with timeout)
            byte = self.serial_port.read(1)
            if len(byte) == 0:
                # Timeout reached without receiving new byte
                break
            buffer += byte

            # If we find a valid start delimiter in the buffer, try to identify a full packet.
            if buffer[0] != START_DELIMITER:
                # If the first byte isn't the start delimiter, pop it off and continue.
                buffer.pop(0)
                continue

            # Check if we have at least the minimum number of bytes
            if len(buffer) >= MIN_PACKET_SIZE:
                # Determine expected packet length from the length byte (at index 1)
                expected_length = buffer[1] + 4
                if len(buffer) >= expected_length:
                    # We have read enough bytes; extract the packet
                    packet = buffer[:expected_length]
                    # Remove the processed packet from the buffer
                    buffer = buffer[expected_length:]
                    return packet
        return None