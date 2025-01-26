#!/bin/bash
set -e

# Source ROS setup files
source /opt/ros/noetic/setup.bash
source /root/rUBot_mecanum_ws/devel/setup.bash
cd /root/rUBot_mecanum_ws
chmod -R +x *

# Execute the ROS launch command
roscore &
sleep 5
roslaunch rubot_mecanum_description rubot_bringup_hw_arduino.launch
