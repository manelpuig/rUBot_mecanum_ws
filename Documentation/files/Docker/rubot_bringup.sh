#!/bin/bash
set -e

# Load ROS configurations
#export ROS_MASTER_URI=http://localhost:11311
#export ROS_HOSTNAME=localhost
#export DISPLAY=${DISPLAY}

# Source ROS setup files
source /opt/ros/noetic/setup.bash
source /root/rUBot_mecanum_ws/devel/setup.bash

# Execute the ROS launch command
roslaunch rubot_mecanum_description rubot_bringup_hw_arduino.launch
