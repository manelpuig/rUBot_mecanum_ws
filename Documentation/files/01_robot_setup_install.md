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


### **1.1. Install Ubuntu20 server image**

Follow the steps in order to properly install the Ubuntu20 on Raspberrypi:

- Run Raspberry Pi Imager (https://www.raspberrypi.org/software/)
  - select Device: Raspberrypi4
  - select OS Ubuntu --> Ubuntu 20 server 64bits to the SD card
  - Select the configurations:
    - Name: rUBotxx
    - User: ubuntu
    - Pass: ubuntu1234
    - LAN config: wifi you want to connect (i.e. rUBotics)
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
- If you want to change the hostname later:
  ````shell
  sudo hostnamectl set-hostname nou_hostname
  sudo reboot
  ````

If you want to create other **wifi connections and a Hotspot**.

- Install ubuntu Desktop:
  ````shell
  sudo apt install ubuntu-desktop
  sudo reboot
  ````
  > this will take 10 minutes and you have to select "gdm3" during process

- In Ubuntu Desktop graphical interface, create a **Hotspot** following instructions in: https://www.debugpoint.com/2020/04/how-to-create-wifi-hotspot-in-ubuntu-20-04-lts/

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
- To create other **wifi networks**, use the Desktop graphical interface to easy select the available wifi and configure to start automatically with priorities (higher number corresponds to higher priority)
  >Review the default wifi network defined on the Ubuntu20 image creation with "Imager"
- If you want to use the robot Hotspot within Nomachine remote desktop, you will have to install Nomachine in raspberypi: https://downloads.nomachine.com/es/download/?id=106&distro=Raspberry&hw=Pi4
- If you want to configure a fixed IP:
    - "sudo nm-connection-editor"
    - Select the "Robotics_UB" wifi connection
    - In section "IPv4 settings" Select "Manual"
    - Add IP address and choose the desired IP (192.168.0.61), Netmask (255.255.255.0), Gateway (192.168.0.1)
    - Add DNS server (8.8.8.8) for google DNS
    - Apply and reboot

### **1.2. Install ROS Noetic Desktop**

Follow the instructions on: http://wiki.ros.org/noetic/Installation/Ubuntu
> Is recommended to update and upgrade first:
```shell
sudo apt update
sudo apt upgrade
```

### **1.3. Install Arduino**

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

  Go to the Arduino Sketchbook location and install libraries. With Arduino core you will surelly have to create the directory. If you have arduino programed, you will not need the ros_libraries:

  ````shell
  cd /home/ubuntu/snap/arduino/current/libraries/
  rosrun rosserial_arduino make_libraries.py .
  ````
  
  Restart your Arduino IDE and you should see the ros_lib part of your libraries!

### **1.4. Install rplidar**
You need to install the package: http://wiki.ros.org/rplidar

```shell
sudo apt install ros-noetic-rplidar-ros
```

### **1.5. Install usb-cam**
You need to install the package: https://wiki.ros.org/usb_cam

```shell
sudo apt install ros-noetic-usb-cam
```

### **1.6. Create the Service to bringup on boot**

It will be good to bringup the robot on boot:

- Create the script file:
  ````shell
  nano /home/ubuntu/start_rubot.sh
  ````
- Edit the script file:
  ````shell
  #!/bin/bash
  source /opt/ros/noetic/setup.bash
  source /home/ubuntu/rUBot_mecanum_ws/devel/setup.bash
  cd /home/ubuntu/rUBot_mecanum_ws
  roslaunch rubot_mecanum_description rubot_bringup_hw_arduino.launch
  ````
- Make the script executable:
  ````shell
  chmod +x /home/ubuntu/start_rubot.sh
  ````
- Create the service file:
  ````shell
  sudo nano /etc/systemd/system/rubot.service
  ````
- Edit the service file:
  ````shell
  [Unit]
  Description=Run robot bringup
  After=network.target

  [Service]
  ExecStart=/home/ubuntu/start_rubot.sh
  User=ubuntu
  Restart=always
  RestartSec=5
  Environment=HOME=/home/ubuntu
  WorkingDirectory=/home/ubuntu

  [Install]
  WantedBy=multi-user.target
  ````
- Reload systemd services:
  ````shell
  sudo systemctl daemon-reload
  ````
- Enable and start the service:
  ````shell
  sudo systemctl enable rubot.service
  sudo systemctl start rubot.service
  ````
**Test the Service**

- Check the status of the service:
  ````shell
  sudo systemctl status rubot.service
  sudo journalctl -u rubot.service -b
  ````
- Check ROS topics:
  ````shell
  rostopic list
  ````
**Modify the Service**

- Stop the servide:
  ````shell
  sudo systemctl stop rubot.service
  ````
- Make modifications in sh file or service file
- restart the service
  ````shell
  sudo systemctl daemon-reload
  sudo systemctl start rubot.service
  ````
> If the service was already enabled it is not needed to disable and enable again

**Remove the Service**
- Disable the service:
  ````shell
  sudo systemctl disable rubot.service
  ````
- Stop the service:
  ````shell
  sudo systemctl stop rubot.service
  ````
- Remove the service file:
  ````shell
  sudo rm /etc/systemd/system/rubot.service
  ````
- Reload systemd services:
  ````shell
  sudo systemctl daemon-reload
  ````

## **2. Configure Raspberrypi with Docker**

To create a fast and robust image of ROS Noetic for our robot, an improved method is to use Docker.

References:
- Docker for LIMO robot (ROS1): https://bitbucket.org/theconstructcore/agilex_limo/src/master/
- Docker for LIMO robot (ROS2): https://hub.docker.com/repository/docker/theconstructai/limo/general


### **2.1. Install Raspberrypi Desktop**

- Run Raspberry Pi Imager (https://www.raspberrypi.org/software/)
  - select Device: Raspberrypi4
  - select OS: RaspberryPi OS (64Bits) to the SD card
  - Select the configurations:
    - Name: rUBot_XX
    - User: ubuntu
    - Pass: ubuntu1234
    - LAN config: wifi you want to connect (i.e. rUBotics)
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
  - If you want to change the hostname:
  ````shell
  sudo hostnamectl set-hostname nou_hostname
  sudo reboot
  ````
  - If you want to configure a fixed IP:
    - "Edit connections"
    - Select the "Robotics_UB" wifi connection
    - In section "IPv4 settings" Select "Manual"
    - Add IP address and choose the desired IP (192.168.0.61), Netmask (255.255.255.0), Gateway (192.168.0.1)
    - Add DNS server (8.8.8.8) for google DNS
    - Apply and reboot

  ### **2.2. Install VNC connections**

- Connect to the raspberrypi with ssh and activate **VNC connections**:
  - type: sudo raspi-config
  - Navigate to: Interface Options > VNC > Select Yes to enable it.
  - sudo reboot
- In your PC install Remote desktop on RealVNC Viewer: https://www.realvnc.com/es/connect/download/viewer

- If you want to connect to another network, you have to be connected first manually to the different networks to enable raspberrypi to connect to on reboot
- reboot and it will be connected to the first network available

### **2.3. Using VScode remote explorer**

You can install the Extension "Remote explorer" on VScode:

- Open VScode and connect remotelly to the Raspberrypi with ssh -X ubuntu@192.168.xxx.xxx
- If you can not connect to the raspberrypi, perhaps you have to regenerate permissions (replace IP-raspberrypi by 192.168.xx.xx):
  ````shell
  ssh-keygen -R IP-raspberrypi
  ````

### **2.4. Install Docker**

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

**Create a custom Docker image**

We first create a home/ubuntu/Desktop/Docker folder where we place:
- rUBot_mecanum_ws.zip
- Dockerfile
- rubot_bringup.sh (in executable mode!)
- docker-compose.yaml
- .env folder to store the Environment variables (DISPLAY, etc.)

These files are located in this repository on Documentation/files/Docker folder

Follow the instructions:
- Build the Image
````shell
cd /home/ubuntu/Desktop/Docker
docker build -t rubot_ros_noetic_image .
````
- Start the Container
````shell
docker compose up -d
````
- Stop the Container
````shell
docker compose down
````
- Enable Docker to Start on Boot
````shell
sudo systemctl enable docker
````
- To check the running container and to check logs for troubleshooting:
````shell
docker ps
docker logs rubot_ros_noetic_container
````
- To modify the rubot_bringup.sh file: Simply edit rubot_bringup.sh on your host machine. Changes will reflect in the container on the next restart.

To verify if the container is working type on terminal:
````shell
docker exec -it rubot_ros_noetic_container /bin/bash
````

