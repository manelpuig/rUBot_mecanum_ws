# Base official ROS Noetic image for ARM64 (Raspberry Pi)
FROM arm64v8/ros:noetic

# Set environment variables to avoid interactive prompts during installations
ENV DEBIAN_FRONTEND=noninteractive

# Update the system, install dependencies, and clean up in a single RUN command
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    apt-utils \
    git \
    curl \
    netcat \
    ros-noetic-rosserial \
    ros-noetic-rosserial-arduino \
    ros-noetic-slam-gmapping \
    ros-noetic-navigation \
    arduino \
    python3-serial \
    ros-noetic-teleop-twist-keyboard \
    x11-apps \
    libx11-dev \
    libgtk-3-dev \
    build-essential \
    python3-catkin-tools \
    python3-pip \
    unzip \
    ros-noetic-rviz \
    gazebo11 \
    ros-noetic-gazebo-ros-pkgs \
    ros-noetic-joint-state-publisher \
    ros-noetic-robot-state-publisher \
    ros-noetic-usb-cam \
    ros-noetic-rplidar-ros \
    && rm -rf /var/lib/apt/lists/* \
    && pip3 install --no-cache-dir setuptools numpy

# Set up the working directory
WORKDIR /root

# Copy and unzip the rUBot_mecanum_ws.zip file
COPY rUBot_mecanum_ws.zip /root/rUBot_mecanum_ws.zip
RUN unzip /root/rUBot_mecanum_ws.zip -d /root/ && rm /root/rUBot_mecanum_ws.zip

# Build the ROS workspace
RUN /bin/bash -c "source /opt/ros/noetic/setup.bash && cd /root/rUBot_mecanum_ws && catkin_make"

# Add workspace and ROS environment setup to .bashrc
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc && \
    echo 'source /root/rUBot_mecanum_ws/devel/setup.bash' >> /root/.bashrc && \
    echo 'cd /root/rUBot_mecanum_ws' >> /root/.bashrc

# Default command to execute the ROS launch file
CMD ["roslaunch", "rubot_mecanum_description", "rubot_bringup_hw_arduino.launch"]
