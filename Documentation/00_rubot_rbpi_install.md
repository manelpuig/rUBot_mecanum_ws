# **rUBot mecanum in Raspberrypi4 Install**

The raspberrypi4 onboard has to be preinstalled with:
- Ubuntu 20 server 64bits
  - NoMachine remote desktop
- ROS Noetic
 

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
If the desktop is not working, restart the display manager:
````shell
sudo systemctl restart gdm3
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
  - type ifconfig
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
Your SD is ready