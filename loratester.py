import serial
import time
import sys
import argparse
import signal

def signal_handler(sig, frame):
    print('\nExiting gracefully...')
    if 'arduino_serial' in globals() and arduino_serial.is_open:
        arduino_serial.close()
    sys.exit(0)

def setup_serial(port_number, baud_rate):
    try:
        port = f'/dev/ttyUSB{port_number}'
        ser = serial.Serial(port, baud_rate, timeout=1)
        print(f"Connected to {port} at {baud_rate} baud")
        return ser
    except serial.SerialException as e:
        print(f"Error opening serial port: {e}")
        sys.exit(1)

def send_message(ser, message):
    try:
        ser.write((message + '/n').encode('utf-8'))
        print(f"Sent: {message}")
    except Exception as e:
        print(f"Error sending message: {e}")

def receive_messages(ser, timeout=None):
    start_time = time.time()
    print("Listening for messages. Press Ctrl+C to exit.")
    try:
        while timeout is None or (time.time() - start_time) < timeout:
            if ser.in_waiting > 0:
                line = ser.readline().decode('utf-8', errors='replace').rstrip()
                print(f"Received: {line}")
            time.sleep(0.1)
    except Exception as e:
        print(f"Error receiving message: {e}")

def main():
    parser = argparse.ArgumentParser(description='LoRa Communication Tester')
    parser.add_argument('port', type=str, help='USB port number (e.g., 0 for /dev/ttyUSB0)')
    parser.add_argument('mode', choices=['send', 'receive'], help='Operation mode: send or receive')
    parser.add_argument('-m', '--message', type=str, default='Hello World', help='Message to send (only in send mode)')
    parser.add_argument('-b', '--baud', type=int, default=9600, help='Baud rate (default: 9600)')
    parser.add_argument('-t', '--timeout', type=int, help='Timeout in seconds for receive mode (default: none)')
    
    args = parser.parse_args()
    
    global arduino_serial
    arduino_serial = setup_serial(args.port, args.baud)
    
    if args.mode == 'send':
        send_message(arduino_serial, args.message)
    elif args.mode == 'receive':
        receive_messages(arduino_serial, args.timeout)
    
    arduino_serial.close()

if __name__ == "__main__":
    signal.signal(signal.SIGINT, signal_handler)
    main()
