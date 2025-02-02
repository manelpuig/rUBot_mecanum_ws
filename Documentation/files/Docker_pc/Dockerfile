# Base official ROS Noetic-Desktop-Full image for Windows PC
FROM osrf/ros:noetic-desktop-full AS base

# Update and install dependencies
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
    apt-utils \
    iputils-ping \
    nano \
    git \
    python3-serial \
    build-essential \
    python3-catkin-tools \
    python3-pip \
    unzip \
    ros-noetic-teleop-twist-keyboard \
    x11-apps \
    libx11-dev \
    libgtk-3-dev \
    ros-noetic-slam-gmapping \
    ros-noetic-navigation \
    sudo \
    wget \
    liblua5.3-dev \
    libboost-all-dev \
    libceres-dev \
    libprotobuf-dev \
    protobuf-compiler \
    libgflags-dev \
    libgoogle-glog-dev \
    libeigen3-dev \
    libsdl2-dev \
    libyaml-cpp-dev \
    libcurl4-openssl-dev \
    && rm -rf /var/lib/apt/lists/*  # Clean up package cache

# Add lines to the .bashrc to source ROS and workspace setup files, and change directory to workspace
RUN echo 'source /opt/ros/noetic/setup.bash' >> /root/.bashrc 

# Configure the user and working directory (default to root)
USER root
WORKDIR /root

# Command to run when the container starts
CMD ["/bin/bash"]
