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

### **1.1. Hotspot configuration**

In Ubuntu desktop, select "Configuration settings" --> "network" and select "hotspot"

You will have to change the Hotspot name (one for each robot). Change the name and password with:
```shell
sudo nm-connection-editor
```
You will be able to change:
- the name (corresponding to your number): rUBot_xx
- The password maintain rock1234

### **1.2. wifi configuration**

The rock5b board has a wireless module (https://es.rs-online.com/web/p/complementos-para-placas-rock/2563901) able to:
- configure a Hotspot
- and a wifi connection

In Ubuntu desktop, select "Configuration settings" --> "network" and select the other wifi connection. 

Select the wifi you want to connect to and type the password. You will now have internet access when you are connected to the rUBot!

## **2. Clone the updated rUBot github repository**

The updated version of rUBot repository is located in:https://github.com/manelpuig/rUBot_mecanum_ws

You will have to:
- In your PC: 
  - Fork this repository to your github account
- In rUBot remote desktop:
  - Open VS Code
  - Clone your github rUBot workspace:
    - Delete the older "rUBot_mecanum_ws" folder
    - open a new terminal in /home/rock 
    - type:
    ```shell
    cd /home/rock
    git clone https://github.com/your_github_name/rUBot_mecanum_ws
    cd rUBot_mecanum_ws
    catkin_make
    ```
  - Open the .bashrc file in home folder
  - copy these lines at the end
  ```shell
  source /opt/ros/noetic/setup.bash
  source /home/rock/rUBot_mecanum_ws/devel/setup.bash
  export GAZEBO_MODEL_PATH=/home/rock/rUBot_mecanum_ws/src/rubot_mecanum_description/models:$GAZEBO_MODEL_PATH
  ```

  You are ready to work with your robot!

