# **ROBOTsetup**

The objectives are:

- Configure the raspberrypi
- Install and bringup the robot model

We will use a raspberrypi4 board with ubuntu20 ROS Noetic and a hotspot wifi connection.

## **1. Configure the raspberrypi**

We will use a raspberrypi4 board with ubuntu20 ROS Noetic intalled.

There are 2 different options for this installation:

- Install ubuntu20 server image and ROS Noetic
- Install Raspberrypi Desktop and ROS Noetic with Docker


### **1.1. Install Ubuntu20 server image and ROS Noetic**

Follow the steps in order to properly install the Raspberrypi:

- Run Raspberry Pi Imager (https://www.raspberrypi.org/software/)
  - select Device: Raspberrypi4
  - select OS Ubuntu --> Ubuntu 20 server 64bits to the SD card
  - Select the configurations:
    - Name: Raspberry
    - User: ubuntu
    - Pass: ubuntu1234
    - LAN config: wifi you want to connect (i.e. robotics-ub)
    - Regional settings: ES
    - Services: activate ssh
- Insert the SD in a RBPi board and connect it to screen and ethernet cable to the router
- power the raspberrypi4 and login:
  - login: ubuntu
  - password: ubuntu1234
- update the OS:
````shell
sudo apt update
sudo apt upgrade
sudo reboot
````
- Install ubuntu Desktop (optional):
````shell
sudo apt install ubuntu-desktop
````
> this will take 10 minutes and you have to select "gdm3" during process
- shutdown and you will be on Desktop
````shell
sudo shutdown now
````

### **Create Wi-Fi connections**

To can create other wifi connections and a Hotspot

To create a **Hotspot** follow instructions in: https://www.debugpoint.com/2020/04/how-to-create-wifi-hotspot-in-ubuntu-20-04-lts/

- Select Wi-Fi settings
- Select "Turn On Wi-Fi Hotspot"
  >Carefull!:
  >- If "Turn On Wi-Fi Hotspot" is disabled select another setting (i.e. Bluetooth) and come back to Wi-Fi setting
- Choose a SSID corresponding to your robot name
  - Name: rubot_01
  - Pass: rUBot_Mec
- To change the Hotspot settings (name or password):
  ```shell
  sudo nm-connection-editor
  ```
- Make the connection "Hotspot" start automatically:
  ```shell
  nmcli con mod Hotspot connection.autoconnect yes
  ``` 
- This can be done without graphical interface using a terminal:
  ````shell
  sudo nmcli connection add type wifi ifname wlan0 mode ap con-name "Hotspot" ssid "rUBot_xx"
  sudo nmcli connection modify "Hotspot" wifi-sec.key-mgmt wpa-psk
  sudo nmcli connection modify "Hotspot" wifi-sec.psk "your_password"
  ````
To create other **wi-fi connections**, a very intesresting configuration is to connect the robots to a speciffic WiFi network connection (if available) or gererate this Hotspot if this network is not available.

- Add and Connect to the desired Wi-Fi Network on startup (i.e. "Manel" Network)
````shell
sudo nmcli connection add type wifi ifname wlan0 con-name "Manel" ssid "Manel"
sudo nmcli connection modify "Manel" wifi-sec.key-mgmt wpa-psk
sudo nmcli connection modify "Manel" wifi-sec.psk "your_password"
````
- Configure the Raspberry Pi to connect to "Desired WIFI" when available and start the "Hotspot" only if "Desired WIFI" isn’t found.
````shell
sudo nmcli connection modify "Manel" connection.autoconnect yes
sudo nmcli connection modify "Manel" connection.autoconnect-priority 10
sudo nmcli connection modify "Hotspot" connection.autoconnect yes
sudo nmcli connection modify "Hotspot" connection.autoconnect-priority 1
````

### **Install ROS Noetic Desktop**

Follow the instructions on: http://wiki.ros.org/noetic/Installation/Ubuntu
> Is recommended to update and upgrade first:
```shell
sudo apt update
sudo apt upgrade
```

### **Install Arduino**

Interesting review: https://www.clearpathrobotics.com/assets/guides/noetic/ros/Driving%20Husky%20with%20ROSSerial.html

- You need to install it with snap to be sure you have all dependencies. You can install Arduino IDE on Ubuntu and add your user to the dialout and tty groups:
  ````shell
  sudo snap install arduino
  sudo usermod -a -G tty ubuntu
  sudo usermod -a -G dialout ubuntu
  sudo reboot
  ````
- Install ROS Packages for Arduino:
  ````shell
  sudo apt install ros-noetic-rosserial
  sudo apt install ros-noetic-rosserial-arduino
  ````
- Install ROS Libraries:

  Go to the Arduino Sketchbook location and install libraries:

  ````shell
  cd /home/ubuntu/snap/arduino/current/Arduino/
  rosrun rosserial_arduino make_libraries.py .
  ````
  
  Restart your Arduino IDE and you should see the ros_lib part of your libraries!

### **Install rplidar**
You need to install the package: http://wiki.ros.org/rplidar

```shell
sudo apt install ros-noetic-rplidar-ros
```

### **Install usb-cam**
You need to install the package: https://wiki.ros.org/usb_cam

```shell
sudo apt install ros-noetic-usb-cam
```

## **2. Install Raspberrypi Desktop and ROS Noetic with Docker**

To create a fast and robust image of ROS Noetic for our robot, an improved method is to use Docker.

- Run Raspberry Pi Imager (https://www.raspberrypi.org/software/)
  - select Device: Raspberrypi4
  - select OS: RaspberryPi OS (64Bits) to the SD card
  - Select the configurations:
    - Name: Raspberry
    - User: ubuntu
    - Pass: ubuntu1234
    - LAN config: wifi you want to connect (i.e. robotics-ub)
    - Regional settings: ES
    - Services: activate ssh
- Insert the SD in a RBPi board and connect an ethernet cable to the router
- power the raspberrypi4 and login:
  - login: ubuntu
  - password: ubuntu1234
- update the OS:
  ````shell
  sudo apt update
  sudo apt upgrade
  sudo reboot
  ````
- Connect to the raspberrypi with ssh and activate VNC connections:
  - type: sudo raspi-config
  - Navigate to: Interface Options > VNC > Select Yes to enable it.
  - sudo reboot
- In your PC install Remote desktop on RealVNC Viewer: https://www.realvnc.com/es/connect/download/viewer

- If you want to connect to another network, you have to be connected first manually to the different networks to enable raspberrypi to connect to on reboot
- reboot and it will be connected to the first network available
- Open VScode and connect remotelly to the Raspberrypi with ssh -X ubuntu@192.168.72.xxx
- If you can not connect to the raspberrypi, perhaps you have to regenerate permissions (replace IP-raspberrypi by 192.168.xx.xx):
  ````shell
  ssh-keygen -R IP-raspberrypi
  ````
### **Docker setup**

In raspberrypi, add Docker’s official repository for Ubuntu
````shell
sudo apt update
sudo apt upgrade
# install Docker automatically in function of the Raspbian version installed
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
# Start Docker service
sudo systemctl start docker
# Enable Docker to start on boot
sudo systemctl enable docker
sudo systemctl enable containerd.service
# Add your user to the Docker group (to avoid using sudo for Docker commands)
sudo usermod -aG docker $USER
# Reboot to apply changes (especially for the user group change)
sudo reboot
````

### **Create a custom Docker image**

We first create a Docker_rubot folder where we:
- Copy the rUBot_mecanum_ws
- Install the Docker extension
- Create a Dockerfile file to specify the image characteristics:
    - Image from arm64v8/ros:noetic
    - install git, ros-noetic-rosserial, ros-noetic-rosserial-arduino, arduino with ros libraries, slam gmapping 
    - X11 libraries to allow graphical applications
    - Optionally: Copy the rUBot_mecanum_ws to the /root/ folder and run "roslaunch rubot_mecanum_description rubot_bringup_hw_arduino.launch"
- Create the Docker Image
````shell
cd /home/ubuntu/Desktop/Docker
sudo docker build -t ros-noetic-rubot-mecanum .
or
sudo docker build -t ros-noetic-rubot-mecanum:v2 -f Dockerfile2 .
````
**Start Docker Container automatically**

Docker Compose is the best way to automate and manage container startup, as it allows you to easily specify the configuration for starting your container

- Create first the DISPLAY and RUBOT environment variables:
  ````shell
  export DISPLAY=192.168.88.72
  export RUBOT=192.168.88.93
  ````
- Create a file "docker-compose.yaml" with:
  ````shell
  services:
    ros-noetic-rubot-mecanum:
      image: ros-noetic-rubot-mecanum  # Your custom image
      container_name: container-ros-noetic-rubot-mecanum
      environment:
        - DISPLAY=${DISPLAY}
        - ROS_MASTER_URI=http://${RUBOT}:11311
        - ROS_IP=${RUBOT}
      volumes:
        - /tmp/.X11-unix:/tmp/.X11-unix:rw
      ports:
        - "11311:11311"  # Expose the ROS master port
      devices:
        - /dev/video0  # Uncomment if the device is available
        - /dev/ttyUSB0  # Uncomment if the device is available
        - /dev/ttyACM0  # Uncomment if the device is available
      network_mode: "host"  # Ensures the container shares the network with the host
      #command: /bin/bash -c "source /opt/ros/noetic/setup.bash && roscore"  # Start roscore
      command: /bin/bash -l -c "\
        source /opt/ros/noetic/setup.bash && \
        source /root/rUBot_mecanum_ws/devel/setup.bash && \
        cd /root/rUBot_mecanum_ws && \
        roslaunch rubot_mecanum_description rubot_bringup_hw_arduino.launch"
      restart: always  # Automatically restart the container if it stops
  ````

- Start the container manually the first with Docker Compose to test if works properly
  ````shell
  export DISPLAY=192.168.88.72:0
  docker compose up
  or
  docker compose -f docker-compose2.yaml up -d
  ````
  > You can create a .env folder to store the Environment variables

- Close remote connection
- Verify on reboot

  > You have NOT to remove the container to ensure it starts automatically on reboot.

This start the container and next time when you connect the raspberrypi, this container will be executed automatically.
