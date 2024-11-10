# **rUBot mecanum setup**

The rUBot mecanum robot we will work is represented in the picture:

His main characteristics are: 

- RaspberryPi4 computer onboard control with Ubuntu20 and ROS Noetic
  - RPlidar distance sensor
  - usb camera sensor
- Arduino MEGA with shield for sensors & actuators control
  - Servomotor actuators for the 4 mecanum wheels
  
![](./Images/01_Setup/1_osoyoo.png)


**Bibliography:**
- https://bitbucket.org/theconstructcore/workspace/projects/PS
- Arduino original program: https://blog.csdn.net/baidu_23831861/article/details/106938752

The main objectives of this chapter are:

- Getting started with rUBot_mecanum in simulation environment
- Getting started with rUBot_mecanum in real robot

## **1. Getting started with rUBot_mecanum in simulation environment**

To **setup the repository** in your ROS environment, you need to:
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

To **sync the repository** with the one in your github account, follow the instructions:
- Access to the TheConstruct environment local repository:
  ````shell
  cd /local repository path
  ````
- Update the local repository with possible changes in github origin repository
  ````shell
  git pull origin master
  ````
- You can work with your local repository for the speciffic project session
- Once you have finished and you want to syncronize the changes you have made and update the github origin repository, type:
  ````shell
  git add .
  git commit -m "Message"
  git push origin master
  ````
- You will have to specify the Username and Password (Token you have generated)

Your github origin repository has been updated!

## **2. Getting started with rUBot_mecanum in real robot**

The main objectives are:

- Setup the rUBot with raspberrypi4-B 8GB
- Copy the "rUBot_mecanum_ws" workspace to be ready

The raspberrypi4 onboard is preinstalled with:
- Ubuntu20.04 server 64bits
- NoMachine remote desktop
- ROS Noetic

When connected to power, it is configured to:
- generate a hotspot "rubot_XX"
- virtual monitor installed

### **Robot connection from PC**

To connect your PC to the Robot, we have to:
- select the rubot hotspot:
    - SSID name: rubot_XX 
    - password "rUBot_Mec"

To control remotelly your raspberrypi from your local PC, you have different options:

### **2.1. Using nomachine remote desktop**
To connect your computer to the robot using Nomachine, follow the same procedure and take into account:

- user: ubuntu
- password: ubuntu1234

>You do not need a Dongle HDMI

For a proper Display resolution in Nomachine, select: Display --> Change the size of remote screen

You will have the rUBot desktop on your windows nomachine screen


#### **Copy a repository**

Every laboratory session you have to copy your updated "rUBot_mecanum_ws" repository to the home/ubuntu/Desktop folder.

If you have not internet connection you can:
- obtain the zip file of repository from github 
- drag the zip file from computer directory to the raspberrypi4 Desktop folder
- Unzip the file. Take care with the name of folder has surely changed to "rUBot_mecanum_ws-master" and perhaps another subfolder with the same name has been added. Make the necessary modifications before compilation.
- Compile with "catkin_make"

Review the ~/.bashrc: Verify the last lines:
```shell
source /opt/ros/noetic/setup.bash
source /home/ubuntu/Desktop/rUBot_mecanum_ws/devel/setup.bash
```

You are ready to work for the laboratory session!

### **2.2. Using VS code remote connection with graphical display**
To connect your computer to the robot using VS code with "Remote Explorer" extension installed:
- Open VS code and select "Remote Explorer"
- Add a new SSH connection as: "ssh -X ubuntu@10.42.0.1"
- Verify on ssh/settings "ForwardX11 yes" and "user ubuntu" 
- After connection specify the password as "ubuntu1234"

>Note: When you connect to your rUBot from another computer, you will have perhaps to regenerate the KEYS. In a new cmd on your PC, type the instuction and you will be able to connect with VScode:
````shell
ssh-keygen -R 10.42.0.1
````

You are now inside the rUBot_xx raspberrypi!

To finish, follow the steps:
- in a VScode rUBot terminal type:
````shell
sudo shutdown now
or
sudo poweroff
````
- in VS code choose "Close remote connection"

You will be automatically disconnected from VS code and after 1 minute, you can switch off the raspberryPi.

**Graphical display**
To have graphical display we need:
- In your PC: Install Xlaunch
  - Download and install "Xming X Server for Windows" from: https://sourceforge.net/projects/xming/
  - Start XLaunch and in first window choose "Multiple windows" and Display "0"
  - in second window select "Start no client".
  - In the third window, ensure that "Disable access control" is selected (this allows your Raspberry Pi to connect).
  - Finish the setup and let XLaunch run in the background.
- Obtain the IP of your PC using "cmd" and type "ipconfig" (i.e. 10.42.0.78)
- In VScode raspberrypi4:
  - update and upgrade the raspberrypi ubuntu
  - Perhaps the firewall will block the connection, you have to create a new firewall rule:
    - Open the Windows Defender Firewall settings.
    - Go to Advanced Settings.
    - Create a new inbound rule:
      - Rule Type: Custom
      - Program: All programs
      - Protocol: TCP
      - Local Port: All ports
      - Remote IP Address: 10.42.0.0/24
      - Action: Allow the connection
      - Profile: Check only "Private" to allow traffic when connected to private networks.
  - reboot
  - Add to .bashrc file the lines: 
  ````shell
  export DISPLAY=10.42.0.78:0.0
  source /opt/ros/noetic/setup.bash
  source /home/ubuntu/Desktop/rUBot_mecanum_ws/devel/setup.bash
  ````

Graphical windows will be displayed in your PC Display!

### **2.3. Using The Construct Environment**

To connect your rUBot mecanum robot to the The Construct environment you need the rUBot to be connected to Ethernet within a WiFi network. 

Let's configure our rUBots:
- In the **TheConstruct account**, we have to first Add a new robot:
  - Open the TheConstruct account and choose "Real Robots". Here you can add your robot.
  - Specify the name and the ROS version
  - Copy the "robot setup code line" used later to setup the robot to the TheConstruct environment

- In the **VScode terminal on rUBot-raspberrypi**:
  - It is better first to update and upgrade the ubuntu20
  - Paste the "robot setup code line". After some minutes the robot will be properly installed.
  - Run 'source ~/.bashrc' to re-export ROS variables before running roscore.
  - Bringup the robot
  >To know the IP address of the raspberrypi4 when connected to "Manel" WiFi network, the best tool is "Advanced IP Scanner" (https://www.advanced-ip-scanner.com/download/). 
  >- Run this application in your PC when your PC is also connected to the "Manel" WiFi network.
  >- You can change the name of the Device with "rUBot_xx" to identify the robot connected

- In the **TheConstruct environment ROS_Noetic_course** from "my ROSjects": 
  - Select "Real robots", select the rUBot you have configured previously and "connect"
  - You can control the robot from the same TheConstruct environment you are using for simulation

If **another student** wants to connect to the same robot from his own "TheConstruct" environment, he has to:
- Be sure the rUBot is not connected to any other user
- Be sure the bringup and all the open processes are closed
- Be sure that the previous student in his own "TheConstruct environment" has: 
  - Closed rviz, gazebo and all the opened processes 
  - Disconnected the rUBot from the "Real robots" menu
  - Exit the ROSject
  - logout his TheConstruct account
- He has then to proceed to Add and install the robot in his TheConstruct account and connect the robot to its TheConstruct environment ROS_Noetic_course RosJect like the first student.