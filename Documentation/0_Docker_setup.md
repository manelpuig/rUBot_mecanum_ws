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


### **1.5. Creating custom Docker**

In Visual Studio code, install the docker extensions:
- Docker (Microsoft)

Then create a new "Dockerfile" with the image configuration:
```python
# This is an example Docker File
#  Command to build it
# docker built -t <image name > .
FROM osrf/ros:noetic-desktop-full

RUN apt-get update
RUN apt-get install -y git && apt-get install -y python3-pip
RUN echo "git and pip Installed"
RUN apt install -y gedit
RUN apt install -y gedit-plugins
RUN echo "gedit Installed"
RUN apt install nautilus -y
RUN apt install gnome-terminal -y
RUN sudo apt install nautilus-actions gnome-terminal -y
RUN echo "Nautilus File manager Installed"

RUN cd /home/

RUN echo "ALL Done"
```
Open a Power Shell terminal in the Dockerfile location and type:
```shell
docker build -t ros1_noetic_mpuig .
```
The Dockerfile is a text file that will produce a Docker Image

Run the image:
```shell
docker run --name ROS1_mpuig -e DISPLAY=host.docker.internal:0.0 -it ros1_noetic_mpuig:latest
```
A "ROS1_mpuig" new container is created. 

For successive times, you have to run direcly the container from Docker Desktop, and open in terminal.

Open Nautilus
```shell
nautilus
```
In a gnome terminal 1 type:
```shell
source /opt/ros/noetic/setup.bash
roscore
```
In a gnome terminal 2 type:
```shell
source /opt/ros/noetic/setup.bash
rosrun turtlesim turtlesim_node
```
The Turtlesim appears in screen

### **1.6. Github sync from docker**

When finished, **syncronize** the changes with your github. 
- Open a terminal in your local repository and type the first time:
```shell
git config --global user.email mail@alumnes.ub.edu
git config --global user.name 'your github username'
git config --global credential.helper store
```
- for succesive times, you only need to do:
```shell
git add -A
git commit -a -m 'message'
git push
```
- you will need to insert the username and the saved PAT password
- syncronize your repository several times to save your work in your github account
> - You can **update** your repository either in your local or **remote repository**:
>   - Local: with the previous instructions
>   - Remote: using web-based Visual Studio Code:
>       - pressing "·" key
>       - performing repository modifications
>       - typing "**git pull**" to syncronize

### **1.7. Clone your repository**

You can now clone any repository:
- Open a new gnome-terminal in /home:
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

### **1.8. Visual Studio Code with Docker**
You can add Docker extension to your VS Code and connect it to a running Container:
- Add extension "Docker" and "Dev Container"
- Select Docker item in left side
- Select the Docker container you want to connect to
- right-click and select "Attach Visual Studio Code"

A new VS Code windows appears with the runnin container

You can syncronize the changes within github inside VS Code.