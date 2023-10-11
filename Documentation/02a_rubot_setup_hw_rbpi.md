# **rUBot mecanum in Raspberrypi4 Install**

The main objectives are:

- Setup the raspberrypi4
- Setup the rUBot workspace in raspberrypi4


Let's see how to fulfill these objectives


## **1. Setup the raspberrypi4**

The raspberrypi4 onboard is preinstalled with:
- Ubuntu 20 server 64bits
  - NoMachine remote desktop
- ROS Noetic
- rubot_rbpi4_ws repository is located in /home/pi/Desktop folder 

### **1.1. Install Ubuntu 20**

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


### **1.2. ROS Noetic Desktop installation**

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

You can install Arduino IDE on Ubuntu using command line:
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


## **2. Setup the rubot workspace in raspberrypi4**

The raspberrypi4 is configured:
- to generate a hotspot "rUBot_xx"
- NoMachine activated 
- Sensor packages activated 

**Connect remotelly** to your RaspberryPy4:

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

**Create your workspace**

We will clone the rUBot_mecanum_ws from our github to complete the project
```shell
cd /home/ubuntu
git clone https://github.com/yourGitHubUserName/rUBot_mecanum_ws
cd rUBot_mecanum_ws/
catkin_make
echo "source /home/ubuntu/rUBot_mecanum_ws/devel/setup.bash" >> ~/.bashrc
```
We are ready to work with our project.
