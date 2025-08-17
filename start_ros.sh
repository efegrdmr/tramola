#!/bin/bash

# Give tramola user access to /dev/ttyTHS1
sudo chown tramola:tramola /dev/ttyTHS1
sudo chmod 660 /dev/ttyTHS1

# Source ROS environment
source /opt/ros/melodic/setup.bash
# Source your workspace
source ~/catkin_ws/devel/setup.bash
# Launch your file
roslaunch tramola robot.launch
