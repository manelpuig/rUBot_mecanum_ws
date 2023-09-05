# **Docker tutorial**

ROS Installation will be made using Docker and we will work and program ROS using Visual Studio Code


## **1. Docker Installation**
Installation instructions could be found in: https://docs.docker.com/desktop/

Some other interessant links could be found:
- http://wiki.ros.org/docker
- http://wiki.ros.org/docker/Tutorials/Docker

You can find a lot of ROS images available. 

Usefull **images that could be opened with browser**:
- ROS_Noetic: https://hub.docker.com/r/arvinskushwaha/ros-noetic-desktop-vnc/tags
- ROS2_Foxy: https://hub.docker.com/r/husarion/ros2-desktop-vnc

  
The **official images** are mantained by Open Source Robotics Foundation (OSRF).

You can find them in:
- https://registry.hub.docker.com/_/ros/
- https://hub.docker.com/r/osrf/ros/tags

We will use the VNC image to work properly with windows and MAC:
- Open a terminal in your computer and type:
```shell
docker pull arvinskushwaha/ros-noetic-desktop-vnc:latest
```
- Verify the Docker desktop the image installed. 
![](./Images/00_Docker/00_Docker_imageVNC.png)
- Run The image with the following options:
![](./Images/00_Docker/00_Docker_container_settings.png)
- Verify the Docker containers created:
![](./Images/00_Docker/00_Docker_container_created.png)
- Run the container in Docker Desktop
- Select in 3-dot menu the option "Open with browser"
![](./Images/00_Docker/00_Docker_container_browser.png)
- A new window appear in your web browser with the graphical interface:
![](./Images/00_Docker/00_Docker_container_vnc.png)

You will have now a graphical interface working properly in windows and MAC

## **2. Visual Studio Code with Docker**

The best way to work with ROS within Docker is to use Visual Studio Code

The following extensions have to be installed:
- Docker from Microsoft
- Dev Container from Microsoft


Execute the container and Connect to it within VS Code:
    - from left-side menu choose Docker
    - right-click on the running container and select "Attach VS Code"
- Update your docker Container:
```shell
apt update
apt upgrade
```
>repeat these instructions until you see "All packages are up to date"

- Install some functionalities:
```shell
apt install -y git && apt install -y python3-pip
```
Open the file ".bashrc" on root folder and add the lines at the end
```shell
source /opt/ros/noetic/setup.bash
```
To test Run the Master
```shell
roscore
```

Start another terminal in the same container ID and open turtlesim node.
```shell
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtlesim_node
```

## **3. Clone and syncronize your repository**

You can now **clone** any repository:
- Open a new terminal in /home:
```shell
git clone https://github.com/manelpuig/rUBot_mecanum_ws
cd rUBot_mecanum_ws
source /opt/ros/noetic/setup.bash
catkin_make
```
- Open .bashrc from /root and type at the end:
```shell
source /opt/ros/noetic/setup.bash
source /home/rUBot_mecanum_ws/devel/setup.bash
export GAZEBO_MODEL_PATH=/home/rUBot_mecanum_ws/src/rubot_mecanum_description/models:$GAZEBO_MODEL_PATH
```
- Open a new terminal from any place and verify the correct behaviour

When finished, **syncronize** the changes with your github. 
- Select "Source control" to see the changes you have made
- Select all changes and add a commit message
- When you will Push them, the first time you will be asked to link the repository to your github account:
- Open a terminal in VS Code and type the first time:
```shell
git config --global user.email mail@alumnes.ub.edu
git config --global user.name 'your github username'
```
- for succesive times, you only need to select changes, Commit a message and Push

>You can **update** your repository using web-based Visual Studio Code:
>
>       - pressing "·" key (or add .dev extension)
>       - performing repository modifications
>       - typing "git pull" to syncronize

## **4. Alternative ROS environment**

Mainly for MAC users, if the graphical interface is not working properly, we suggest 2 different solutions:
- Ready to use limited performances web environment
- Use a Virtual machine with virtual box

### **4.1. Ready to use web environment**
You can use the ROS environment developed by "The Construct" (https://app.theconstructsim.com/), enterprise working on ROS academia and development and located in Barcelona.
- Open the link: https://app.theconstructsim.com/
- Create a free user account
- Select "myRosjects"
![](./Images/00_Docker/myRosjects.png)
- Create a New Project
![](./Images/00_Docker/ROBIO_Ros1.png)
You will have 2GB Virtual machine with Ubuntu 20 and ROS Noetic installed for free!!

### **4.2. Virtual machine with ROS**
You can always install a ROS virtual machine with Virtual Box for MAC
- Download Virtual Box for macOS systems (https://www.virtualbox.org/wiki/Downloads)
- Install Virtual Box in your computer
- Download the ROS virtual machine in this link:
- and Add this ROS virtual machine choosing file-->Import appliance (and follow instructions)

You will have a ready to use ROS environment


## **5. Software install within Ubuntu Image**

Once you have ubuntu image working properly, mainly for MAC users, you can install SW based on Ubuntu:

### **5.1. roboDK**
Mainly for MAC users, you can install roboDK:
- download the ubuntu version roboDK from: https://robodk.com/download
- Extract executable to /home/roboDK (create first the roboDK folder)
- roboDK icon will appear in Desktop. You can double click to start
- Locate path to python and editor
```shell
which python3
which idle
```
- Place the paths to "Tools --> Options --> Python"

You are ready to use roboDK with python in your Container!