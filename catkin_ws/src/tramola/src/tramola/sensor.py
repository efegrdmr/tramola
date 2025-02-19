import serial
import time
import re  # Import regex module
import rospy
# Open Serial Connection
arduino = serial.Serial(
    port='/dev/ttyUSB0',  # Change based on your device
    baudrate=115200,
    bytesize=serial.EIGHTBITS,
    parity=serial.PARITY_NONE,
    stopbits=serial.STOPBITS_ONE,
    timeout=0.05,
    xonxoff=False,
    rtscts=False,
    dsrdtr=False,
    writeTimeout=0.1
)


def get_sensor_distance():
    try:
        arduino.flushInput()  # Clear buffer
        arduino.write("connected from Jetson Nano\n".encode())

        while True:
            data = arduino.readline().decode('utf-8').strip()
            if data:
                rospy.loginfo(f"Received: {data}")  # Debugging

                # Extract numbers only (remove 'cm' or other text)
                match = re.search(r"\d+(\.\d+)?", data)  # Find a number with optional decimal
                if match:
                    return float(match.group())  # Convert to float

                rospy.logwarn("Invalid data, retrying...")

            time.sleep(0.1)  # Allow time for a new reading

    except Exception as e:
        rospy.logwarn(f"Error: {e}")
        return None

    # Main Loop


if __name__ == "__main__":
    while True:
        distance = get_sensor_distance()
        if distance is not None:
            rospy.loginfo(f"Distance: {distance} cm")
        else:
            rospy.logwarn("Failed to get distance")

