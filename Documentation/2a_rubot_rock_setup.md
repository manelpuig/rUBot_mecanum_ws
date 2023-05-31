# **rUBot mecanum in Rock Install**

The main objectives are:

- Setup the rUBot with rock5b 16GB
- Clone the rUBot workspace to be ready


## **1. Setup the rubot with rock5b**

The rock5b onboard is preinstalled with:
- Ubuntu 20 server 64bits
- NoMachine remote desktop
- ROS Noetic
- Arduino
- rUBot workspace version

Insert the SD card in rock5b board and power it. 
- A hotspot will be created:
  - SSID: rock5_tplink
  - Pass: rock1234
- Connect your computer to this Hotspot
- Install in your computer Nomachine remote desktop: https://www.nomachine.com/es
- Open Nomachine and connect to the rUBot 
  - Name: the rUBot name
  - Host: 10.42.0.1
  - Port: 4000
  Protocol: NX
- You will be asked for:
  - login: rock
  - password: rock

After few seconds you will have the rUBot desktop in your computer screen!

You will need to perform some configurations:

### **1.1. Hotspot configuration**

In Ubuntu desktop, select "Configuration settings" --> "network" and select "hotspot"

You will have to change the Hotspot name (one for each robot):

Change the Hotspot settings (name or password):
```shell
sudo nm-connection-editor
```
You will be able to change the name and password. Change only the name to rUBotXX (with the robot number)

### **1.2. wifi configuration**

The rock5b board have a wireless module (https://es.rs-online.com/web/p/complementos-para-placas-rock/2563901) able to:
- configure a Hotspot
- and a wifi connection

In Ubuntu desktop, select "Configuration settings" --> "network" and select the other wifi connection. 

Select the wifi you want to connect to and type the password. You will now have internet access when you are connected to the rUBot!

## **2. Clone the updated rUBot github repository**

The updated version of rUBot repository is located in:https://github.com/manelpuig/rUBot_mecanum_ws

You will have to:
- Fork this repository to your github account
- Clone your github rUBot workspace in the rUBot Desktop:
  - Delete the older "rUBot_mecanum_ws" folder
  - open a new terminal in Desktop 
  - type:
  ```shell
  git clone https://github.com/your_github_mane/rUBot_mecanum_ws
  cd /Home/Desktop/rUBot_mecanum_ws
  catkin_make
  ```

  You are ready to work with your robot!

