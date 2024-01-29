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
  - Review the installation proces using Rock5b or raspberryPi4

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


### **Setup the rubot with rock5b**

The rock5b onboard is preinstalled with:
- Ubuntu20.04 server 64bits
- NoMachine remote desktop
- ROS Noetic
- Arduino

When connected to power, it is configured to:
- generate a hotspot "rubot_XX"
- virtual monitor installed
- LIDAR activated 
- usb_cam activated 

### **Robot connection from PC**

To connect your PC to the Robot, we have to:
- select the rubot hotspot:
    - SSID name: rubot_XX 
    - password "rock1234"

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

For a proper Display resolution in Nomachina select: Display --> Change the size of remote screen

You will have the rUBot desktop on your windows nomachine screen

### **Clone a repository**

The first time you have to clone the "rUBot_mecanum_ws" repository to the home folder.
```shell
cd /home
git clone https://github.com/your_username/rUBot_gopigo_ws
cd rUBot_mecanum_ws
catkin_make
```
> If you have not internet connection you can:
- obtain the zip file of repository from github
- copy the zip file from a pendrive on the rock5b USB port to the home directory
- or drag the zip file from coputer directory to the rock5b Desktop directory

Review the ~/.bashrc: Verify the last lines:
```shell
source /opt/ros/noetic/setup.bash
source /home/rUBot_mecanum_ws/devel/setup.bash
```

## **3. Rock5b setup for real robot**

You can download the image in:
https://ubarcelona-my.sharepoint.com/:u:/g/personal/manel_puig_ub_edu/Ee4sYzrzwPtNt-PCSUIgMG0Bixm4dodL5_3-tY7SWHInAQ?e=yvXi18

Insert the SD card in rock5b board and power it. 
- A hotspot will be created:
  - SSID: rUBot_xx
  - Pass: rock1234
- Connect your computer to this Hotspot
- Install in your computer Nomachine remote desktop: https://www.nomachine.com/es
- Open Nomachine and connect to the rUBot 
  - Name: the rUBot name
  - Host: 10.42.0.1
  - Port: 4000
  - Protocol: NX
- You will be asked for:
  - login/user: rock
  - password: rock

After few seconds you will have the rUBot desktop in your computer screen!

You will need to perform some configurations:

### **Hotspot configuration**

In Ubuntu desktop, select "Configuration settings" --> "network" and select "hotspot"

You will have to change the Hotspot name (one for each robot). Change the name and password with:
```shell
sudo nm-connection-editor
```
You will be able to change:
- the name (corresponding to your number): rUBot_xx
- The password maintain rock1234

### **wifi configuration**

The rock5b board has a wireless module (https://es.rs-online.com/web/p/complementos-para-placas-rock/2563901) able to:
- configure a Hotspot
- and a wifi connection

In Ubuntu desktop, select "Configuration settings" --> "network" and select the other wifi connection. 

Select the wifi you want to connect to and type the password. You will now have internet access when you are connected to the rUBot!

## **4. RaspberryPi4 setup for real robot**

If you choose a raspberryPi4 as a computer onboard to your robot, you can:
- make the configuration from scratch
- Use a pre-configured image

### **4.1. RaspberryPi4 configuration from scratch**

You will have to install:
- Ubuntu 20 server 64bits
- ROS Noetic
- Arduino
- NoMachine remote desktop

#### **4.1.1. Install Ubuntu 20**

We have to install first **Ubuntu 20 server 64bits**:

- Install Raspberry Pi OS using Raspberry Pi Imager (download for windows): https://www.raspberrypi.org/software/
- Run the application and save the image:
  - Ubuntu --> Ubuntu 20 server 64bits to the SD card
- Insert the SD in a RBPi board and connect it to screen and ethernet cable to the router
- power the raspberrypi4 and login:
  - login: ubuntu
  - password: ubuntu
- You will have to change the password (we use ubuntu1234)

- Update the Ubuntu server
```shell
sudo apt update
sudo apt upgrade
```

**Install Ubuntu Desktop**

In the terminal, type:
```shell
sudo apt-get install ubuntu-desktop
sudo apt update
sudo apt upgrade
```
> Note: Keyboard is not ESP and "-" is on " ' " key


Type:
```shell
reboot
```
You will get the ubuntu 20 desktop

- choose a wifi network and change the timezone, language and password

**Create WiFi Hotspot**

Follow instructions in: https://www.debugpoint.com/2020/04/how-to-create-wifi-hotspot-in-ubuntu-20-04-lts/

- Choose a SSID corresponding to your robot name: rUBot_XX
  >If "Turn On Wi-Fi Hotspot is disabled select another setting (i.e. Bluetooth) and come back to Wi-Fi setting
- Specify the password: rUBot_Mec

If you want to change the Hotspot name (one for each robot). Change the Hotspot settings (name or password):
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
Identify the IP of the rbpi4 Hotspot:
  - type "ip address show"
  - in wlan0 you identify the inet address: 10.42.0.1

Restart the raspberrypi4 to verify the automatic Hotspot configuration

**Install nomachine remote desktop**

Download nomachine in RaspberryPi and PC:
- In raspberryPi:
  - Download the Raspberrypy4 version ARMv8 DEB in: https://www.nomachine.com/download/linux&id=29&s=Raspberry
  - open a terminal in download folder and install following the instructions
  - Run the NoMachine application you have installed (Not the NoMachine server!)
  - restart your raspberrypi4
- In PC: https://www.nomachine.com/


#### **4.1.2. ROS Noetic Desktop installation**

Follow the instructions on: http://wiki.ros.org/noetic/Installation/Ubuntu

You can connect to raspberrypi4 using NoMachine remote desktop.

> Is recommended to update and upgrade first:
```shell
sudo apt update
sudo apt upgrade
```
**Install sensor packages**
- Install **rplidar**:
You need to install the package: http://wiki.ros.org/rplidar
```shell
sudo apt install ros-noetic-rplidar-ros
```
- Install **usb-cam**: You need to install the package: https://wiki.ros.org/usb_cam
```shell
sudo apt install ros-noetic-usb-cam
```
You need to test which video device is connected and change it on the launch file (usb_cam_rock.launch)

- Install **Arduino**:

Interesting review: https://www.clearpathrobotics.com/assets/guides/noetic/ros/Driving%20Husky%20with%20ROSSerial.html

You need to install it with snap to be sure you have all dependencies. You can install Arduino IDE on Ubuntu using command line:
```shell
sudo snap install arduino
sudo usermod -a -G tty ubuntu
sudo usermod -a -G dialout ubuntu
```
Reboot

To avoid any possible problems when using Arduino IDE, you have added your system user to the dialout and tty groups.

If there is a problem, contact: https://github.com/snapcrafters/arduino/issues/

Install ROS Packages for Arduino:
```shell
sudo apt install ros-noetic-rosserial
sudo apt install ros-noetic-rosserial-arduino
```
- Configure **Arduino IDE**:

To install **ESP32 boards**:

In the Arduino IDE, go to File > Preferences.

In the "Additional Boards Manager URLs" field, add the following URL to install the ESP32 boards: https://espressif.github.io/arduino-esp32/package_esp32_index.json

Now go to Tools  --> Board Manager and search for ESP32. Install Arduino ESP32 and esp32 boards.

To install **ROS Libraries**:

Go to the Arduino Sketchbook location and install libraries:
```shell
cd /home/ubuntu/snap/arduino/current/Arduino/libraries
rosrun rosserial_arduino make_libraries.py .
```
Restart your Arduino IDE and you should see the ros_lib part of your libraries!

**Install Teleop control**

Follow instructions in:
- https://wiki.ros.org/teleop_twist_keyboard
- https://wiki.ros.org/joy
- https://github.com/ros-drivers/joystick_drivers/tree/main/ps3joy

Run in a terminal:
```shell
sudo apt install ros-noetic-teleop-twist-keyboard
sudo apt install ros-noetic-joy
```
You need to pair the devices.

You can change the PS3 PAD and buttons assignments.


### **4.2. RaspberryPi4 configuration from image**

When using an image, the raspberrypi4 onboard is preinstalled with:
- Ubuntu 20 server 64bits
- ROS Noetic
- Arduino
- NoMachine remote desktop
- rubot_rbpi4_ws repository is located in /home/pi/Desktop folder 

You can dowload the image from:
https://ubarcelona-my.sharepoint.com/:u:/g/personal/manel_puig_ub_edu/EZ2unyO-xzJEoQin_RgrG0wBbgXEB5aal5nDtirZ7ZhfDA?e=YoZaLm

