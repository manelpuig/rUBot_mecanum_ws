# **Migration to ROS2**

ROS1 is close to finish and you can switch to ROS2
![](./Images/7_ROS2_time.png)
ROS2 structure is based on the architecture:
![](./Images/7_ROS1_ROS2.png)
The main differences are:
![](./Images/7_ROS2_dif.png)

ROS2 is a very good choice.

Interesting references for courses:
- https://www.udemy.com/course/ros2-for-beginners/learn/lecture/20260476#overview
- https://www.udemy.com/course/learn-ros2-as-a-ros1-developer-and-migrate-your-ros-projects/learn/lecture/22003074#overview
- https://www.udemy.com/course/ros2-ultimate-mobile-robotics-course-for-beginners-opencv/learn/lecture/28143024#overview
- https://www.udemy.com/course/ros2-self-driving-car-with-deep-learning-and-computer-vision/learn/lecture/28236852#overview

Some interesting projects:
- https://github.com/noshluk2/ROS2-Ultimate-Mobile-Robotics-Course-for-Beginners-OpenCV
- https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV

And the ROS2 reference sites:
- https://docs.ros.org/
- https://docs.ros.org/en/foxy/

## 1. **ROS2 Installation & Tools**
Follow the instructions on:
https://docs.ros.org/en/foxy/Installation.html

Some other installations needed:
- python3 pip
```shell
sudo apt install python3-pip
pip3 install argcomplete
```
> needed for autoformating (press crtl+shift+i)
- Colcon compilation utility (https://docs.ros.org/en/foxy/Tutorials/Colcon-Tutorial.html)
```shell
sudo apt install python3-colcon-common-extensions
```
> To use autocompletion you need to add in ~/.bashrc file this line:
>- source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
- Visual Studio Code with some extensions:
    - python 
    - python for VS Code
    - python Intellisense
    - XML (Red-Hat)
    - XML Tools
    - Code runner
    - ROS
    - C/C++
    - cmake

- Terminator (https://cheatography.com/svschannak/cheat-sheets/terminator-ubuntu/):
```shell
sudo apt install terminator
```
Finally you have to reboot:
```shell
sudo reboot
```

## 2. **Create workspace**

To work with ROS2 first open your ~/.bashrc and be sure you have sourced ROS2 adding the lines:
- source /opt/ros/foxy/setup.bash
- source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash

Create "self_driving_car_ws2" workspace with "src" folder

Compile the ws:
```shell
colcon build
```

Create your first Package: https://docs.ros.org/en/foxy/Tutorials/Creating-Your-First-ROS2-Package.html
