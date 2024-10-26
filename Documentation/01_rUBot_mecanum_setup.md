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


### **Using nomachine remote desktop**
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
- Unzip the file and compile

Review the ~/.bashrc: Verify the last lines:
```shell
source /opt/ros/noetic/setup.bash
source /home/ubuntu/Desktop/rUBot_mecanum_ws/devel/setup.bash
```

You are ready to work for the laboratory session!

### **Using VS code remote connection**
To connect your computer to the robot using VS code with "Remote connection" extension:
- Open VS code and select "Remote Explorer"
- Select the connection "SSH-10.42.0.1"
- Verify on ssh/settings the user as "ubuntu"
- specify the password as "ubuntu1234"

>Note: When you connect to another rUBot from the same computer, you will have to regenerate the KEYS. In a new cmd on your PC, type:
````shell
ssh-keygen -R 10.42.0.1
````

You are now inside the rUBot_xx raspberrypi!

To finish, follow the steps:
- in a VScode rUBot terminal type:
````shell
sudo shutdown now
````
- in VS code choose "Close remote connection"

You will be automatically disconnected from VS code and after 1 minute, you can switch off the raspberryPi.

## **2.2. Using The Construct Environment**

To connect your rUBot mecanum robot to the The Construct environment you have to first Add a new robot:
- Open TheConstruct environment and choose "Real Robots". Here you can add your robot.
- Specify the name and the ROS version
- Copy the code line to setup the robot to the TheConstruct environment

Then connect to your own rUBot:
- Connect VScode to the rUBot
- It is better to update and upgrade the ubuntu
- Open a terminal and paste the "robot setup code line". After some minutes the robot will be properly installed.
- Run 'source ~/.bashrc' to re-export ROS variables before running roscore.
- Bringup the robot

Now you connect the The Construct environment to your own rUBot:
- Open the The Construct environment and choose "Your own ROSjects"
- Select "Real robots" and "connect"
- You can control the robot from TheConstruct Terminal

If you want to connect the same robot to another TheConstruct environment account, you have to:
- In rUBot VScode terminal: Close the bringup and all the open processes
- In TheConstruct environment: 
  - Close rviz, gazebo and all the open processes 
  - Disconnect the rUBot from the "Real robots" menu
  - Exit the ROSject
  - logout your TheConstruct account

Open a new account of TheConstruct environment:
- Select my ROSjects
- Select the rUBot from the "Real robots" menu
- Select "Configure" and copy the code line to setup the rUBot to this new environment
- In rUBot VScode terminal:
  - Run this code line to setup the rUBot in the VScode terminal
  - Run 'source ~/.bashrc' to re-export ROS variables before running roscore.
  - Bringup the robot
- In TheConstruct environment:
  - Select "Real robots" and "connect"
  - You can control the robot from TheConstruct Terminal