# **rUBot mecanum setup**

The rUBot mecanum robot we will work is represented in the picture:

His main characteristics are: 
- Arduino based control for sensors & actuators
  - Servomotor actuators for the 4 mecanum wheels
  
- High-level onboard control with Ubuntu20 and ROS Noetic
  - RPlidar distance sensor
  - usb camera sensor

![](./Images/01_Setup/1_osoyoo.png)


**Bibliography:**
- https://bitbucket.org/theconstructcore/workspace/projects/PS
- Arduino original program: https://blog.csdn.net/baidu_23831861/article/details/106938752

The main objectives of this chapter are:

- Getting started with rUBot_mecanum in simulation environment
- Getting started with rUBot_mecanum in real robot

## **1. Getting started with rUBot_mecanum in simulation environment**

To **setup** the repository in your ROS environment, you need to:
- Fork my repository in your github account
- Open your ROS Noetic environment: https://app.theconstructsim.com/
- Clone your forked directory in your home directory
```shell
cd /home/user
git clone https://github.com/yourusername/rUBot_mecanum_ws
cd rUBot_mecanum_ws
catkin_make
```
- Open .bashrc file with VS Code (open file...)
- Ensure that you have the last 2 lines (review the exact name of your repository):
```shell
source /opt/ros/noetic/setup.bash
source /home/user/rUBot_mecanum_ws/devel/setup.bash
```
You are ready to work with your repository!

## **2. Getting started with rUBot_mecanum in real robot**

The main objectives are:

- Setup the rUBot with rock5b 16GB
- Clone the rUBot workspace to be ready


### **2.1. Setup the rubot with rock5b**

The rock5b onboard is preinstalled with:
- Ubuntu20.04 server 64bits
- NoMachine remote desktop
- ROS Noetic
- Arduino

You can find the image in OneDrive: Documents/Software/ROS/Rock/rock5b_32_original.img

When connected to power, it is configured to:
- generate a hotspot "rubot_XX"
- virtual monitor installed

### **Robot connection from PC**

To connect your PC to the Robot, we have to:
- select the rubot hotspot:
    - SSID name: rubot_XX 
    - password "rock1234"

To change the hotspot SSID, use:
```shell
sudo nm-connection-editor
```

### **Using nomachine remote desktop**
To connect your computer to the robot using Nomachine:
- Install in your computer Nomachine remote desktop: https://www.nomachine.com/es
- If you are using the Lab-PCs:
  - Plug the pendrive to USB port of PC in Lab IE
  - Execute "nxplayer.exe" 

Add new robot and edit the connection:
- Name: rUBot_XX
- Host: 10.42.0.1
- Port: 4000
- Protocol: NX

Connect
- user: rock
- password: rock

For a proper Display resolution in Nomachine, select: Display --> Change the size of remote screen

You will have the rUBot desktop on your windows nomachine screen

To change the hotspot SSID, use:
```shell
sudo nm-connection-editor
```

### **2.2. Setup the rubot with raspberrypi4**

The raspberrypi4 onboard is preinstalled with:
- Ubuntu20.04 server 64bits
- NoMachine remote desktop
- ROS Noetic
- Arduino

You can find the image in OneDrive: Documents/Software/ROS/RBPi/Noetic_rbpi4.img.gz (ready for Pi Imager)

When connected to power, it is configured to:
- generate a hotspot "rubot_XX"
- virtual monitor installed

### **Robot connection from PC**

To connect your PC to the Robot, we have to:
- select the rubot hotspot:
    - SSID name: rubot_XX 
    - password "rUBot_Mec"


### **Using nomachine remote desktop**
To connect your computer to the robot using Nomachine, follow the same procedure and take into account:

- user: ubuntu
- password: ubuntu1234

>You do not need a Dongle HDMI

For a proper Display resolution in Nomachine, select: Display --> Change the size of remote screen

You will have the rUBot desktop on your windows nomachine screen

To change the hotspot SSID, use:
```shell
sudo nm-connection-editor
```
### **Using windows Xrdp remote desktop**
To connect your computer to the robot using Xrdp, follow the instructions in: https://somebooks.es/escritorio-remoto-en-ubuntu-20-04-con-xrdp/

- Install xrdp en raspberrypi4 with ssh connexion:
```shell
sudo apt install xrdp -y
``` 
- In windows PC open "Remote Desktop Connexion"
- Use the Account data:
  - IP: 10.42.0.1
  - user: ubuntu
  - password: ubuntu1234

### **Extra Wifi connection**

To have internet access, you can connect and install USB wireless adapter:
https://www.amazon.es/Archer-T2U-Nano-inal%C3%A1mbrico-Escritorio/dp/B07LGSDBTF
or 
https://www.amazon.es/TP-Link-Archer-T2U-Nano-Adaptador/dp/B07PB1X4CN

- Install the device:
```shell
sudo apt install dkms
git clone https://github.com/aircrack-ng/rtl8812au.git
cd rtl8812au
sudo make dkms_install
```
You will need to shutdown the raspberrypi4 to finish the installation 

### **2.3. Clone a repository**

The first time you have to clone the "rUBot_mecanum_ws" repository to the home folder.
```shell
cd /home/rock
git clone https://github.com/your_username/rUBot_mecanum_ws
cd rUBot_mecanum_ws
catkin_make
```
> If you have not internet connection you can:
- obtain the zip file of repository from github or the ROS Noetic simulation environment
- drag the zip file from coputer directory to the rock5b home directory
- Unzip the file and compile

Review the ~/.bashrc: Verify the last lines:
```shell
source /opt/ros/noetic/setup.bash
source /home/rock/rUBot_mecanum_ws/devel/setup.bash
```
