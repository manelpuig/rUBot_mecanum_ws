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

- Install Raspberry Pi OS using Raspberry Pi Imager (download for windows): https://www.raspberrypi.org/software/
- Run the application and save the image:
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
  cd /home/ubuntu/snap/arduino/current/Arduino/
  rosrun rosserial_arduino make_libraries.py .
  ````
  >To know the path for "arduino" program type:
  ````shell
  which arduino
  ````
  > In this case you need to install the libraries in the /home/ubuntu/snap/arduino/current/Arduino/ folder
  
  Restart your Arduino IDE and you should see the ros_lib part of your libraries!


### **1.2. Install Raspberrypi Desktop and ROS Noetic with Docker**

To create a fast and robust image of ROS Noetic for our robot, an improved method is to use Docker.

Follow the steps in order to properly install the Raspberrypi: