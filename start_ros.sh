#!/bin/bash

# Ensure ROS environment is loaded BEFORE running anything
source /opt/ros/melodic/setup.bash
source /home/tramola/catkin_ws/devel/setup.bash

# Give tramola user access to /dev/ttyTHS1 (preserve env with -E)
sudo -E chown tramola:tramola /dev/ttyTHS1
sudo -E chmod 660 /dev/ttyTHS1

# Launch your ROS launch file
exec roslaunch tramola robot.launch
