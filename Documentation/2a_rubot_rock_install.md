# **rUBot mecanum in Rock Install**

The main objectives are:

- Setup the HW drivers in rock4 SE
- Control the rUBot movement

Let's see how to fulfill these objectives

References:

## **1. Setup the rubot in rock4**
You can connect to the rock4 onboard with:
- Nomachine remote desktop
- Visual Studio Code

### **1.1. Setup the rubot in rock4 using nomachine remote desktop**
The rock4 onboard is preinstalled with:
- Ubuntu 20 server 64bits
  - NoMachine remote desktop
- ROS Noetic

First you connect to the rock4 hotspot:
- SSID: rubot_00
- Password: rockubuntu1234

Second you connect to rock4 with Nomachine remote desktop:
- IP: 10.42.0.1
- user: pi, ubuntu, rock
- password: rockbian, rockubuntu, rock

You will need to change the Keyboard to Spanish keyboard. 

Open a terminal and type:
```shell
sudo setxkbmap -layout 'es,es' -model pc105
setxkbmap es sundeadkeys
```
Change to a bash terminal:
```shell
bash
```
Open .bashrc and add:
```shell
source /opt/ros/noetic/setup.bash
```
### **1.2. Connection using VS Code**
You need the latest version (1.75.1).

First time you will need to connect the rock-board to internet with ethernet cable ti install some capabilities.

Install the extensions.
- Remote development
- Git Extension Pack (in your remote 10.42.0.1 board)

The first time you connect the VS Code to the Remote machine:
- You will need to sign autorization to VS code to access github (will be in left-side bar menu, accounts symbol)
- When sync the changes, you will have to type your email and user name (following the git output information)
- You can work without internet connection, but When you want to sync your repository you will need the ethernet cable connected.

Follow the instructions:
- In windows open VS Code as administrator
- Select the rUBot_XX wifi network 
- From "remote Explorer" (left-side bar menu) select "Remote" and type the IP
- Select linux system for remote connection
- type the password

You will have the VS Code attached to remote machine (rock board)

## **2. Install drivers**
You will need to install:
- rplidar
- usbcam
- Rosserial

First step is:
- Setup your computer to accept software from packages.ros.org.
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
sudo apt upgrade
```

### **2.1. Install rplidar**
You need to install the package: http://wiki.ros.org/rplidar

```shell
sudo apt install ros-noetic-rplidar-ros
```

### **2.2. Install usb-cam**
You need to install the package: https://wiki.ros.org/usb_cam

```shell
sudo apt install ros-noetic-usb-cam
```
You need to test which video device is connected and change it on the launch file (usb_cam_rock.launch)

### **2.3. Install Rosserial**
You need to install the package: http://wiki.ros.org/rosserial

```shell
sudo apt-get install ros-noetic-rosserial
```

## **3. rUBot mecanum bringup**
You can bringup your robot with:
```shell
roslaunch rubot_mecanum_description rubot_bringup_hw.launch
```
You are ready to wotrk with