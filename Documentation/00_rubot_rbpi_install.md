# **rUBot mecanum in Raspberrypi4 Install**

The raspberrypi4 onboard has to be preinstalled with:
- Ubuntu 20 server 64bits
  - NoMachine remote desktop
- ROS Noetic

You can find the image in OneDrive: Documents/Software/ROS/RBPi/Noetic_rbpi4.img.gz (ready for Pi Imager), or you can install all from scratch in a 32GB SD card


## **1. Install Ubuntu 20 server 64bits**

Follow the steps in order to properly install the Raspberrypi:

- Install Raspberry Pi OS using Raspberry Pi Imager (download for windows): https://www.raspberrypi.org/software/
- Run the application and save the image:
  - select Device: Raspberrypi4
  - select OS Ubuntu --> Ubuntu 20 server 64bits to the SD card
  - Select the configurations:
    - Name: Raspberry
    - User: ubuntu
    - Pass: ubuntu1234
    - LAN config: optional to the wifi you want to connect (not needed in our case)
    - Regional settings: ES
    - Services: activate ssh

![](./Images/2_rbpi4_imager2.png)

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
- Install Desktop:
````shell
sudo apt install ubuntu-desktop
````
> this will take 10 minutes and you have to select "gdm3" during process
- shutdown and you will be on Desktop
````shell
sudo shutdown now
````

## **2. Create Wi-Fi Hotspot**

To create a Hotspot, follow instructions in: https://www.debugpoint.com/2020/04/how-to-create-wifi-hotspot-in-ubuntu-20-04-lts/

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
- Verify listing all the Hotspot parameters including "autoconnect"
```shell
nmcli con show Hotspot
```
- Identify the IP of the rbpi4 Hotspot:
  - type "ip add"
  - in wlan0 you identify the inet address: 10.42.0.1


## **3. Install nomachine remote desktop**

Download nomachine in RaspberryPi and PC:
- In raspberryPi:
  - Download the Raspberrypy4 version ARMv8 DEB in: https://www.nomachine.com/download/linux&id=29&s=Raspberry
  - open a terminal in download folder and install following the instructions
- In PC: https://www.nomachine.com/


## **4. Install ROS Noetic Desktop**

Follow the instructions on: http://wiki.ros.org/noetic/Installation/Ubuntu
> Is recommended to update and upgrade first:
```shell
sudo apt update
sudo apt upgrade
```


## **5. Extra Wifi connection**

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

## **6. Install PIGPIO**

To work properly with raspberrypi GPIOs properly, you have to install PIGPIO library:
https://abyz.me.uk/rpi/pigpio/download.html

````shell
wget https://github.com/joan2937/pigpio/archive/master.zip
unzip master.zip
cd pigpio-master
make
sudo make install
````
To use this library you have to open a terminal and type:
````shell
sudo pigpiod
````
## **7. Install Arduino**

Interesting review: https://www.clearpathrobotics.com/assets/guides/noetic/ros/Driving%20Husky%20with%20ROSSerial.html

- You need to install it with snap to be sure you have all dependencies. You can install Arduino IDE on Ubuntu using command line:
  ````shell
  sudo snap install arduino
  sudo usermod -a -G tty ubuntu
  sudo usermod -a -G dialout ubuntu
  sudo reboot
  ````

  To avoid any possible problems when using Arduino IDE, you have added your system user to the dialout and tty groups.

  If there is a problem, contact: https://github.com/snapcrafters/arduino/issues/

- Install ROS Packages for Arduino:
  ````shell
  sudo apt install ros-noetic-rosserial
  sudo apt install ros-noetic-rosserial-arduino
  ````

- Install ROS Libraries:

  Go to the Arduino Sketchbook location and install libraries:

  ````shell
  cd /home/ubuntu/snap/arduino/current/Arduino/libraries
  rosrun rosserial_arduino make_libraries.py .
  ````

  Restart your Arduino IDE and you should see the ros_lib part of your libraries!

## **8. Install Keyboard and Joy control**

You will be able to control your robot either with the Keyboard or with a PS2 gamepad.

**a) Keyboard control**

Follow instructions in: https://wiki.ros.org/teleop_twist_keyboard

You can control the rUBot with the keyboard installing the following package:
````shell
sudo apt install ros-noetic-teleop-twist-keyboard
````
Then you will be able to control the robot with the Keyboard typing:
````shell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
````
**b) Joy control**

You can control the rUBot with the Joypad following the instructions in: https://dev.to/admantium/radu-control-the-robot-using-a-joystick-976

- In order to work with any gamepad, we need to install additional ROS packages:
````shell
sudo apt-get install ros-noetic-teleop-twist-joy ros-noetic-joy
````
These packages provide several ways to interact with a connected joypad. 
- To get started, we will run a ROS node called joy_node with the parameter of the detected device file.
````shell
rosrun joy joy_node dev:=/dev/input/js0
````
- to translate the messages from the /joy topic to TWIST messages, another ROS package already performs this translation. We just need to start the teleop_twist_joy node:
````shell
rosrun teleop_twist_joy teleop_node 
````

- connect the gamepad and select the Mode2 (green and red leds)
- Subscribe to the topic /cmd_vel. Then, on your gamepad, identify the deadman switch button (in our gamepad is the triangle button) and you should see messages published on /cmd_vel topic.
- You can see that this enable a non-holonomic movement
- To enable the holonomic movement, you have to configure the proper parameters.
- An exemple is created to start the 2 nodes with the proper configuration parameters. This is the rubot_joy.launch file.
````shell
roslaunch rubot_control rubot_joy.launch
````
For button mapping documentation refer to: http://wiki.ros.org/joy or http://wiki.ros.org/ps3joy 

For detailed configuration of launch file refer to: https://github.com/ros-teleop/teleop_twist_joy/blob/indigo-devel/launch/teleop.launch

Cool! If you have made the bringup of your robot, you will automatically feed these messages to your robot, and you can start moving around, controlled with a gamepad.

Your SD is ready for ROS programming rUBots!
