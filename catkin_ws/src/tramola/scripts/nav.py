#!/home/tramola/vision/bin/python3.8
import rospy
from geometry_msgs.msg import Twist
import tkinter as tk

# Define speed and turn rates
LINEAR_SPEED = 0.5
ANGULAR_SPEED = 0.5

# Initialize speed variables
linear_speed = 0
angular_speed = 0

# ROS publisher
pub = rospy.Publisher('/mavros/setpoint_velocity/cmd_vel_unstamped', Twist, queue_size=10)
rospy.init_node('usv_teleop_gui')

def update_speed():
    """Update speed display on the GUI."""
    linear_label.config(text=f"Linear Speed: {linear_speed:.2f} m/s")
    angular_label.config(text=f"Angular Speed: {angular_speed:.2f} rad/s")

def publish_speed():
    """Publish the current speed to the ROS topic."""
    twist = Twist()
    twist.linear.x = linear_speed
    twist.angular.z = angular_speed
    pub.publish(twist)

def on_key_press(event):
    global linear_speed, angular_speed

    # Adjust speeds based on key press
    if event.keysym == 'w':
        linear_speed = LINEAR_SPEED
    elif event.keysym == 's':
        linear_speed = -LINEAR_SPEED
    elif event.keysym == 'a':
        angular_speed = ANGULAR_SPEED
    elif event.keysym == 'd':
        angular_speed = -ANGULAR_SPEED

    update_speed()
    publish_speed()

def on_key_release(event):
    global linear_speed, angular_speed

    # Reset speeds when keys are released
    if event.keysym in ['w', 's']:
        linear_speed = 0
    elif event.keysym in ['a', 'd']:
        angular_speed = 0

    update_speed()
    publish_speed()

# Set up Tkinter window
root = tk.Tk()
root.title("USV Teleop Control")
root.geometry("300x200")

# Speed display labels
linear_label = tk.Label(root, text="Linear Speed: 0.00 m/s", font=("Helvetica", 14))
linear_label.pack(pady=10)

angular_label = tk.Label(root, text="Angular Speed: 0.00 rad/s", font=("Helvetica", 14))
angular_label.pack(pady=10)

# Bind keys to control functions
root.bind("<KeyPress>", on_key_press)
root.bind("<KeyRelease>", on_key_release)

# Run Tkinter event loop
root.mainloop()
