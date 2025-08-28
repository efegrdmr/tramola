#!/bin/bash

# ----------------------------
# Configurable parameters
# ----------------------------
BAG_RATE=3.0                # Playback speed
BAG_LOOP=false               # Loop bags? true/false
ROSLAUNCH_FILE="tramola costmap_visualization.launch"  # Your launch file
BAG_FILES="*.bag"           # Bag files pattern
USE_SIM_TIME=true           # Use simulated time

# ----------------------------
# Function to build rosbag args
# ----------------------------
build_bag_args() {
    local args="--clock"
    [ "$BAG_LOOP" = true ] && args="$args --loop"
    [ -n "$BAG_RATE" ] && args="$args -r $BAG_RATE"
    echo $args
}

# ----------------------------
# Trap Ctrl+C to kill all child processes
# ----------------------------
cleanup() {
    echo "Stopping all processes..."
    kill 0  # kills all child processes in the same process group
    exit 0
}
trap cleanup SIGINT

# ----------------------------
# 1. Start roscore in background
# ----------------------------
if ! rostopic list > /dev/null 2>&1; then
    echo "No roscore detected. Starting roscore..."
    roscore &
    ROSCORE_PID=$!
    sleep 5  # wait for roscore to be ready
else
    echo "roscore is already running."
fi

# ----------------------------
# 2. Start rosbag play in background
# ----------------------------
if [ "$USE_SIM_TIME" = true ]; then
    rosparam set use_sim_time true
fi

BAG_ARGS=$(build_bag_args)
echo "Playing rosbag(s) with args: $BAG_ARGS $BAG_FILES"
rosbag play $BAG_ARGS $BAG_FILES &

# ----------------------------
# 3. Start roslaunch in foreground
# ----------------------------
echo "Launching ROS launch file: $ROSLAUNCH_FILE"
roslaunch $ROSLAUNCH_FILE
