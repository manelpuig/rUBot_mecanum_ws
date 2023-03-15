# **rUBot mecanum in Jetson-nano Install**

The main objectives are:
- Setup the Jetson nano 
- Add init with Hotspot
- Install Nomachine remote desktop
- Update to ubuntu20.04
- Install ROS Noetic
- Clone and sync a repository


We can finf this board in 2 versions:
- J1010 NVIDIA Jetson Nano, CPU ARM, GPU Maxwell, 4GB RAM, 16GB eMMc: https://es.farnell.com/seeed-studio/110061362/placa-portadora-nvidia-jetson/dp/4126473?MER=sy-me-pd-mi-acce
- J2021 NVIDIA Jetson Nano, CPU ARM, GPU Maxwell, 8GB RAM, 16GB eMMc: https://es.farnell.com/seeed-studio/110061381/kit-des-nvidia-jetson-xavier-nx/dp/4126474?MER=sy-me-pd-mi-acce

References:
- https://www.seeedstudio.com/Jetson-10-1-A0-p-5336.html?queryID=ed17f3b6b606fb4179f10e4059d72879&objectID=5336&indexName=bazaar_retailer_products
- https://github.com/dusty-nv/jetson-inference
- https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#intro

## **1. Setup the Jetson nano**

For the first time, let's follow the instructions in: https://developer.nvidia.com/embedded/learn/get-started-jetson-nano-devkit#intro


## **2. Add init with Hotspot**

For remote connection we need the Jetson-nano board to start in hotspot mode. To install this performance, follow the instructions:
- Plug a dongle USB: tp-link AC600 Nano Wireless USB Adapter Archer T2U Nano
- Select "System settings" and "Network" and "Wireless"
- Select "Use as Hotspot" and turn on.
- Select "Network connections" from top menu and edit the wi-fi hotspot
- Change the mode to "Hotspot"
- Change the SSID name to "rUBot_01" for exemple and the password
- reboot the board to start the board with hotspot
- From another computer you can see this network and you can connect to it.

To select this Hotspot automatically on restart:
- open a terminal and see all the connection names:
  ```shell
  nmcli con show
  ```
- Make the connection "Hotspot" start automatically:
  ```shell
  nmcli con mod Hotspot connection.autoconnect yes
  ``` 
- Verify listing all the Hotspot parameters including "autoconnect"
  ```shell
  nmcli con show Hotspot
  ```
- Identify the IP of the rbpi4 Hotspot:
  - type ifconfig
  - in wlan0 you identify the inet address: 10.42.0.1

## **3. Install Nomachine remote desktop**
For a remote connection, download and install nomachine in PC and Board:
- In Jetson nano Board, download Nomachine for ARMv8 DEB: https://downloads.nomachine.com/download/?id=114&distro=ARM
- In PC, download Nomachine for windows 64bits: https://downloads.nomachine.com/download/?id=8

## **4. Update to ubuntu20.04**

The Jetson nano J1010 board is preinstalled with Ubuntu 18 

- First update and upgrade the ubuntu version:
```shell
sudo apt update
sudo apt upgrade
sudo apt update
```
- Open "software updater"
- Select the Ubuntu20.04 to work with ROS Noetic

Follow the steps in order to properly install the Raspberrypi:



### **2.2. ROS Noetic Desktop installation**

Follow the instructions on: http://wiki.ros.org/noetic/Installation/Ubuntu
> Is recommended to update and upgrade first:
```shell
sudo apt update
sudo apt upgrade
sudo apt update
```
### **2.3. Create WiFi Hotspot and setup**

Follow instructions in: https://www.debugpoint.com/2020/04/how-to-create-wifi-hotspot-in-ubuntu-20-04-lts/

![](./Images/2_hotspot.png)
>Carefull!:
>- If "Turn On Wi-Fi Hotspot is disabled select another setting (i.e. Bluetooth) and come back to Wi-Fi setting
>- Choose a SSID corresponding to your robot name

To select this Hotspot automatically on restart:
- open a terminal and see all the connection names:
```shell
nmcli con show
```
- Make the connection "Hotspot" start automatically:
```shell
nmcli con mod Hotspot connection.autoconnect yes
``` 
- Verify listing all the Hotspot parameters including "autoconnect"
```shell
nmcli con show Hotspot
```
- Identify the IP of the rbpi4 Hotspot:
  - type ifconfig
  - in wlan0 you identify the inet address: 10.42.0.1

**Setup**

The raspberrypi4 is configured:
- to generate a hotspot "rUBot_xx"
- NoMachine activated 
- raspicam activated 

When powering the raspberrypi4, generates a hotspot you have to connect to:
- SSID name: rUBot_01 
- password "rUBot_Mec"

Once you are connected to this network you will be able to connect your computer to the raspberrypi4 using NoMachine viewer:
- In PC open NoMachine viewer
- Select the raspberrypi IP address: 10.42.0.1
- you have to specify:
    - user: ubuntu
    - password: ubuntu1234
- You will have the raspberrypi4 desktop on your windows NoMachine screen

![](./Images/2_nomachine.png)

If you want to change the Hotspot name (one for each robot):
- Change the Hotspot settings (name or password):
```shell
sudo nm-connection-editor
```

### **2.4. Create your workspace**

We will create the workspace where we will install all needed packages for our Hardware project
```shell
mkdir -p ~/Desktop/rubot_rbpi4_ws/src
cd ~/Desktop/rubot_rbpi4_ws/
catkin_make
echo "source ~/Desktop/rubot_rbpi4_ws/devel/setup.bash" >> ~/.bashrc
```
We will install:
- Raspberrypi camera
- rpLIDAR
- Arduino board with rosserial
### **Install raspberrypi camera**
Information is located on: https://picamera.readthedocs.io/en/release-1.11/install.html


You need to activate the raspicam in Ubuntu:
- open the file config.txt
```shell
sudo nano /boot/firmware/config.txt
```
- add the magic line start_x=1 at the end
![](./Images/2_picam1.png)

- reboot
- update your system to install the necessary drivers:
```shell
sudo apt update
sudo apt upgrade
```
Now we will install the python module for picamera:
```shell
sudo apt install python3-pip
pip install picamera
```
And the needed libraries for raspicam package:
```shell
sudo apt-get install -y libyaml-cpp-dev
sudo apt-get install -y libogg-dev libvorbis-dev libtheora-dev
```
Finally to work with raspicam in ROS, you need to install the packages:
```shell
git clone https://github.com/UbiquityRobotics/raspicam_node.git
git clone --single-branch --branch=noetic-devel https://github.com/ros-perception/image_transport_plugins.git
git clone --single-branch --branch=noetic https://github.com/ros-perception/vision_opencv.git
git clone --single-branch --branch=noetic-devel https://github.com/ros-perception/image_common.git
```
> Follow the Build instructions in: https://github.com/UbiquityRobotics/raspicam_node.

> Make sure that your user is in the video group by running groups|grep video 
```shell
sudo usermod -a -G video ubuntu
```


Then you are able to compile the workspace:
```shell
cd ~/Desktop/rubot_rbpi4_ws/
catkin_make
```

### **Install rpLIDAR**

Open a terminal in src folder and type:
```shell
cd ~/Desktop/rubot_rbpi4_ws/src
git clone https://github.com/Slamtec/rplidar_ros
cd ..
catkin_make
```
To test the sensor, connect the LIDAR sensor to RB Pi and execute:
```shell
roslaunch rplidar_ros view_rplidar.launch
```
### **Install ARDUINO**

This robot will be controlled by an Arduino Mega board.

The arduino program will start a serial_node with all the topics

The Arduino programm is located in "files/Arduino_node" folder.

Considering your Raspberry computer uses Ubuntu 20, as explained in previous sections, you need to download Arduino IDE version for Linux ARM 64 bits from the following link: https://www.arduino.cc/en/software.

After dowloading the zip file, reclocate it and unzip it in the Tools folder: ~/Tools/Arduino-1.8.18. From this directory, open a terminal and execute the following commands:

```shell
sudo ./install.sh
cd ~
gedit .bashrc
export PATH=$PATH:$HOME/Desktop/Arduino-1.8.18
```
Save and close the file and install rosserial for ROS Noetic using:
```shell
sudo apt-get install ros-noetic-rosserial-arduino
sudo apt-get install ros-noetic-rosserial
```
Go to ~/Desktop/Arduino-1.8.18/libraries directory and remove ros_lib folder. From this directory execute:
```shell
rosrun rosserial_arduino make_libraries.py .
```

Test Arduino ROS library with "Hello World" exemple: http://wiki.ros.org/rosserial_arduino/Tutorials/Hello%20World


### **Manage USB ports**

The rbpi4 manage arduino board and rpLIDAR with 2 USB ports. 

We have to force the same USB port to the same device:
- "arduino" port to arduino mega board
- "rplidar" port to rpLIDAR

This can be done creating UDEV rules for each devide:

**a) Arduino udev rules**

Follow instructions: 
- https://steve.fi/hardware/arduino-basics/
- https://medium.com/@darshankt/setting-up-the-udev-rules-for-connecting-multiple-external-devices-through-usb-in-linux-28c110cf9251

  Connect the Arduino Mega in USB port. To see the port name type:
  ```shell
  ls -l /dev/ttyAMC*
  ```
  Give permissions rwx to this port ttyACM*
  ```shell
  sudo chmod 777 /dev/ttyACM*
  ```
  To see the needed properties to create udev rules, type:
  ```shell
  lsusb
  ```
  You will see:
  ```shell
  ubuntu@ubuntu:/usr/lib/udev/rules.d$ lsusb
  Bus 002 Device 001: ID 1d6b:0003 Linux Foundation 3.0 root hub
  Bus 001 Device 007: ID 2341:0042 Arduino SA Mega 2560 R3 (CDC ACM)
  Bus 001 Device 004: ID 0e8f:00fb GreenAsia Inc. 
  Bus 001 Device 003: ID 04d9:a01c Holtek Semiconductor, Inc. wireless multimedia keyboard with trackball [Trust ADURA 17911]
  Bus 001 Device 002: ID 2109:3431 VIA Labs, Inc. Hub
  Bus 001 Device 001: ID 1d6b:0002 Linux Foundation 2.0 root hub
  ```
  The create a file /etc/udev/rules.d/99-arduino.rules and give it the following contents:
  ```xml
  # Arduino port definition
  SUBSYSTEM=="tty", GROUP="plugdev", MODE="0660"

  ACTION=="add", SUBSYSTEMS=="usb", ATTRS{idVendor}=="2341", ATTRS{idProduct}=="0042", SYMLINK+="arduino"
  ```
  >The idProduct and idVendor corresponds to the numbers in the Arduino line.

  Now restart the service:
  ```shell
  sudo /etc/init.d/udev restart 
  ```
  Unplug and plug again the Arduino Board and verify if the symlink is created:
  ```shell
  ls -l /dev/arduino /dev/ttyACM0
  ```
  If all is OK, you will see:
  ```shell
  lrwxrwxrwx 1 root root         7 dic 23 08:58 /dev/arduino -> ttyACM0
  crw-rw---- 1 root plugdev 166, 0 dic 23 08:58 /dev/ttyACM0
  ```

**b) rpLIDAR udev rules**

Follow instructions:  
- https://github.com/Slamtec/rplidar_ros/tree/master/scripts

  Connect the rpLIDAR in USB port. To see the port name type:
  ```shell
  ls -l /dev/ttyUSB*
  ```
  Give permissions rwx to this port ttyUSB*
  ```shell
  sudo chmod 777 /dev/ttyACM*
  ```
  To see the needed properties to create udev rules, type:
  ```shell
  lsusb
  ```
  You will see your rpLIDAR device properties:
  ```shell
  Bus 001 Device 010: ID 10c4:ea60 Silicon Labs CP210x UART Bridge
  ```
  The create a file /etc/udev/rules.d/rplidar.rules and give it the following contents:
  ```xml
  # set the udev rule , make the device_port be fixed by rplidar
  #
  KERNEL=="ttyUSB*", ATTRS{idVendor}=="10c4", ATTRS{idProduct}=="ea60", MODE:="0777", SYMLINK+="rplidar"
  ```
  Now restart the service:
  ```shell
  sudo /etc/init.d/udev restart 
  ```
  Unplug and plug again the Arduino Board and verify if the symlink is created:
  ```shell
  ls -l /dev/arduino /dev/ttyACM0 /dev/rplidar /dev/ttyUSB0
  ```
  If all is OK, you will see:
  ```shell
  lrwxrwxrwx 1 root root         7 dic 23 09:31 /dev/arduino -> ttyACM0
  lrwxrwxrwx 1 root root         7 dic 23 09:31 /dev/rplidar -> ttyUSB0
  crw-rw---- 1 root plugdev 166, 0 dic 23 09:31 /dev/ttyACM0
  crwxrwxrwx 1 root plugdev 188, 0 dic 23 09:31 /dev/ttyUSB0
  ```

You have now your rUBot_mecanum ready to work-with!!


### **Install joy control**

Almost all robots subscribe /cmd_vel topic whose message type is Twist for controlling robots.

#### **PS2 device**
Follow the steps:
- Connect the USB to the rbp4 and push start
- review the new device associated to the ps2
```shell
dmesg -w
```
- identify the assigned input. In raspberryPi4 appears dev/input/js0. In rock4 appears dev/hidraw1
- change mod to "rwxrwxrw-" in dev/input/js0
```shell
sudo chmod a+rw dev/input/js0
```
- install gstest-gtk
```shell
sudo apt install gstest-gtk
```
- Install teleop-twist-joy
```shell
sudo apt install ros-noetic-teleop-twist-joy
```
- create a config folder in src and place inside the "ps3-holonomic.config.yaml" file from teleop-twist-joy package. Configure the button position as desired
- copy the teleop.launch file from teleop-twist-joy package to launch folder of rubot_control package. Change the config folder location in the specific parameter
- run
```shell
roslaunch teleop_twist_joy teleop.launch
```

Test the /joy and /cmd_vel topic's contents

### **Copy the final work-space**

The best way is to copy a zip file of the final work-space (including the build and devel folders) in a pen drive and copy it to each raspberrypi robot.

Usually the file properties are lost and in the ws folder you have to make all files executable:
```shell
cd ~/Desktop/rUBot_mecanum_ws
sudo chmod -R +x ./*
```
You will have the ws ready to work with!