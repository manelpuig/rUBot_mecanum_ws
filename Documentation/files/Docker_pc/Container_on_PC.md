### Create Docker Image with ROS Packages

1.  **Create a file named `Dockerfile`** with the following content:

```dockerfile
FROM osrf/ros:noetic-desktop-full AS base

# Update and install dependencies
RUN apt-get update && apt-get install -y --no-install-recommends \
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

# Expose ROS ports
EXPOSE 11311

# Define ROS entrypoint
ENTRYPOINT ["/ros_entrypoint.sh"]

# Command to run when the container starts
CMD ["/bin/bash"]
```

To create a container: 

1.  **Save the `docker-compose.yaml` file:**
    Create a file named `docker-compose.yaml` in your desired project directory and paste the following content into it:

```yaml
name: pc-noetic

services:
    pc-noetic-container:
    image: noetic_ports
    pull_policy: always
    privileged: true
    network_mode: host
    container_name: noetic_ports_container
    environment:
        - ROS_MASTER_URI=http://master:11311
        - ROS_HOSTNAME=master
    volumes:
        - c:/test:/root/test  # Adjust host path if needed
        - /dev:/dev
        - ./volumes/launch_ros_drivers.sh:/launch_ros_drivers.sh
    devices:
        - "/dev/bus/usb:/dev/bus/usb"
    device_cgroup_rules:
        - 'c 189:* rmw'
    entrypoint: /ros_entrypoint.sh
    command: ["/launch_ros_drivers.sh"]
    restart: always
```

    **Remember to adjust the host path** `c:/test pc` in the `volumes` section if your local `test pc` directory is located elsewhere on your Windows machine.

2.  **Create the `volumes` directory:**
    In the same directory where you saved the `docker-compose.yaml` file, create a new directory named `volumes`:

    ```bash
    mkdir volumes
    ```

3.  **Place your launch script:**
    Copy your ROS launch script (e.g., `launch_ros_drivers.sh`) into the newly created `volumes` directory.

4.  **Build the Docker image (if not already built):**
    If you haven't already built the Docker image named `noetic_ports`, navigate to the directory containing your `Dockerfile` and run the following command:

    ```bash
    docker build -t noetic_ports .
    ```

5.  **Launch the container:**
    Open your terminal or command prompt, navigate to the directory where you saved the `docker-compose.yaml` file, and execute the following command to start the container in detached mode:

    ```bash
    docker-compose up -d
    ```

6.  **(Optional) Check container status:**
    You can verify that the container is running by using the command:

    ```bash
    docker-compose ps
    ```

7.  **(Optional) View container logs:**
    To see the output of your `launch_ros_drivers.sh` script or other container logs, run:

    ```bash
    docker-compose logs pc-noetic-container
    ```

8.  **(Optional) Stop the container:**
    When you need to stop the running container, navigate to the same directory and execute:

    ```bash
    docker-compose down
    ```

**Before running:**

* **Docker Compose Installation:** Ensure you have Docker Compose installed on your system.
* **File Sharing (Windows):** If you are using Docker Desktop on Windows and mounting local directories, make sure the drive containing the host path (e.g., `C:`) is shared in Docker Desktop's settings (Settings -> Resources -> File Sharing).