#!/bin/bash
set -e

export ROS_IPV6=${ROS_IPV6:-off}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
export ROS_HOSTNAME=${ROS_HOSTNAME:-localhost}

# Source ROS setup files
source /opt/ros/noetic/setup.bash
source /root/rUBot_mecanum_ws/devel/setup.bash
cd /root/rUBot_mecanum_ws
chmod -R +x *

# Execute the ROS launch command
roscore &
sleep 5
roslaunch rubot_mecanum_description rubot_bringup_hw_arduino.launch
