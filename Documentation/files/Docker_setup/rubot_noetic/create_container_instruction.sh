#!/bin/bash
sudo docker run \
  --network host \
  -it \
  --name container-ros-noetic-rubot-mecanum \
  --env DISPLAY=$DISPLAY \
  --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device=/dev/video0 \
  --device=/dev/ttyUSB0 \
  --device=/dev/ttyACM0 \
  ros-noetic-rubot-mecanum /bin/bash -c "\
    source /opt/ros/noetic/setup.bash && \
    source /root/rUBot_mecanum_ws/devel/setup.bash && \
    export ROS_MASTER_URI=http://$RUBOT:11311 && \
    export ROS_IP=$RUBOT && \
    cd /root/rUBot_mecanum_ws && \
    roslaunch rubot_mecanum_description rubot_bringup_hw_arduino.launch"