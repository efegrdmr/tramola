import serial
import time

# Open the serial port (adjust as needed)
ser = serial.Serial('/dev/ttyUSB0', 9600)  # Linux
# ser = serial.Serial('COM3', 9600)       # Windows

# Give the connection a second to settle
time.sleep(2)

# Send data
while True:
    ser.write("Hello from Python 2!\n")  # Add newline if receiver uses readline()
    print("Sent message.")
    time.sleep(1)
