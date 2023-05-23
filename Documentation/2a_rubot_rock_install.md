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

> **Important!**: in order to have available all the USB connectors, commute the switch to the position closer to USB3. This control the USB mode from Device to Host.

**Remote desktop**

In order to have remote access within Nomachine you have to:

- Install the package: xserver-xorg-video-dummy
```shell
sudo apt install xserver-xorg-video-dummy
```
- Create a configuration file at "/usr/share/X11/xorg.conf.d" folder
```shell
sudo mousepad /usr/share/X11/xorg.conf.d/20-dummy.conf
```
- This file has the contents:
```python
Section "Monitor"
   Identifier "Monitor0"
   HorizSync 28.0-80.0
   VertRefresh 48.0-75.0
   Modeline "1920x1080_60.00" 172.80 1920 2040 2248 2576 1080 1081 1084 
1118 -HSync +Vsync
EndSection

Section "Device"
   Identifier "Card0"
   Driver "dummy"
   VideoRam 256000
EndSection

Section "Screen"
   DefaultDepth 24
   Identifier "Screen0"
   Device "Card0"
   Monitor "Monitor0"
   SubSection "Display"
     Depth 24
     Modes "1920x1080_60.00"
   EndSubSection
EndSection
```
- reboot

- In your computer choose the rock4 hotspot:
  - SSID: rubot_00
  - Password: rockubuntu1234

- Open Nomachine and connect to rock4 with:
  - IP: 10.42.0.1
  - port: 4000
  - user: pi
  - password: rockbian

- You will need to change the Keyboard to Spanish keyboard. 
```shell
sudo setxkbmap -layout 'es,es' -model pc105
setxkbmap es sundeadkeys
```
- Change to a bash terminal:
```shell
bash
```
- Open .bashrc and add:
```shell
source /opt/ros/noetic/setup.bash
```
You will have remote connection within Nomachine with an optimized resolution!

**Wifi access**

In order to have wifi access we will use a generic wifi dongle and install a specific driver:
- https://github.com/morrownr/8821au-20210708

Follow instructions for installation within Ubuntu.

**Install VS Code**

You need to:
- download the linux .deb arm64 program version in: https://code.visualstudio.com/#alt-downloads
- Install with terminal instruction:
```shell
sudo apt install ./code_arm64.deb
```
  >Use the name of the deb file downloaded
- You will see the Code program in the Ubuntu programs available

**Sync your github repository with VS Code**

Once updated the repository in rock4 board and with ethernet connection:
- add "Git Extension Pack"
- Select "source control"
- Select the changes to sync and add commit
- Select sync and follow the instructions
  - accept communication with github
  - insert your username and password of your github account
  - accep git authorisation when prompted

You will be able to update your github repository!
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

You will be able to perform all in rock4 board but you will not have screen monitor graphic service


## **2. Setup the rubot in rock5**

The rock5 image is preinstalled with:
- Ubuntu 20 server 64bits

Insert the SD card in rock5b board and power it. 

You will be asked for:
- login: rock
- password: rock

Now we have to update the public key of radxa apt. The one used before is expired. (https://github.com/radxa/apt)

You can execute the following command to get the new available.
```shell
sudo apt-get install -y wget
export DISTRO=focal-stable
wget -O - apt.radxa.com/$DISTRO/public.key | sudo apt-key add -
sudo apt update
sudo apt upgrade
```

You will need to install:
- Ubuntu-desktop
- nomachine
- Access to Remote desktop

### **2.1. Install Ubuntu-desktop**

In the terminal, type:
```shell
sudo apt update
sudo apt install ubuntu-desktop
```
Now reboot


### **2.2. Hotspot configuration**
With internet connection, install hostapd
```shell
sudo apt install hostapd
```

In Ubuntu desktop, select "Configuration settings" --> "network" and select "hotspot"

If you want to change the Hotspot name (one for each robot):

Change the Hotspot settings (name or password):
```shell
sudo nm-connection-editor
```
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


### **2.3. Install nomachine**
You have to install ARM version: https://downloads.nomachine.com/download/?id=114&distro=ARM

In the terminal, type:
```shell
sudo dpkg -i nomachine_8.4.2_1_arm64.deb
```
>Verify the correct syntax of nomachine version

### **2.4. Access to Remote desktop**
In order to have remote access within Nomachine you have to:

- Install the package: xserver-xorg-video-dummy
```shell
sudo apt install xserver-xorg-video-dummy
```
- Create a configuration file at "/usr/share/X11/xorg.conf.d" folder
```shell
sudo mousepad /usr/share/X11/xorg.conf.d/20-dummy.conf
```
>Copy the same contents as the rock4 case

## **2.5. Install VS Code**
Open web browser and find VS Code DEB version for Ubuntu in ARM64
- Download and install:
```shell
sudo apt install ./codexxx.deb
```
- Clone the repository to work with
- Make changes and sync
- Follow instructions to insert your email and name
- when syncing you will have to insert ypur github account name and password

## **2.6. Install Arduino**
Open web browser and download the linux arm64bits version
```shell
https://www.arduino.cc/en/software
```
After downloading the zip file, realocate it and unzip it in the Tools folder: ~/Desktop/Arduino. From this directory, open a terminal and execute the following commands:

```shell
sudo ./install.sh
cd ~
gedit .bashrc
export PATH=$PATH:$HOME/Desktop/Arduino
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

## **3. Install drivers**
You will need to install:
- rplidar
- usbcam

First step is:
- Setup your computer to accept software from packages.ros.org.
```shell
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt update
sudo apt upgrade
```

### **3.1. Install rplidar**
You need to install the package: http://wiki.ros.org/rplidar

```shell
sudo apt install ros-noetic-rplidar-ros
```

### **3.2. Install usb-cam**
You need to install the package: https://wiki.ros.org/usb_cam

```shell
sudo apt install ros-noetic-usb-cam
```
You need to test which video device is connected and change it on the launch file (usb_cam_rock.launch)

### **3.3. Install Rosserial**
You need to install the package: http://wiki.ros.org/rosserial

```shell
sudo apt-get install ros-noetic-rosserial
```

## **4. rUBot mecanum bringup**
You can bringup your robot with:
```shell
roslaunch rubot_mecanum_description rubot_bringup_hw_rock.launch
```
You are ready to work with