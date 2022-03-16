# **rUBot mecanum in Raspberrypi4 Control**

The main objectives are:
- Assemble a real robot (rUBot mecanum)
- Control the rUBot movement

Let's see how to fulfill these objectives

References:
- https://github.com/ros-mobile-robots/diffbot
- https://johanschwind.medium.com/mobile-robot-teleoperation-with-the-jetson-nano-and-ros-d72b4b57e9be

## **1. rUBot assembling**

This tobot prototype is based on:
- Mecanum wheel chasis:
  - https://es.aliexpress.com/item/4001121790912.html?gatewayAdapt=glo2esp&spm=a2g0o.9042311.0.0.3c7963c0XKupJW
- Battery 12V&5V (3Ah):
  - https://www.amazon.es/gp/product/B072HR211P/ref=ppx_yo_dt_b_asin_title_o05_s00?ie=UTF8&psc=1
- Battery 12V (6,7 and 20Ah): 
  - https://www.amazon.es/gp/product/B07YSCXGNF/ref=ppx_yo_dt_b_asin_title_o07_s00?ie=UTF8&psc=1
  - https://www.amazon.es/THENAGD-Recargable-Interruptor-Encendido-DC-122000/dp/B091F465RP/ref=sr_1_48?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=3OCUQZILUVPAJ&keywords=bateria%2B5v%2B12v%2B2a%2Brecargable&qid=1642669235&sprefix=bateria%2B5v%2B12v%2B2a%2Brecargable%2Caps%2C65&sr=8-48&th=1
- Battery 5V (20 and 26Ah):
  - https://www.amazon.es/dp/B09MYQFXP5?psc=1&smid=AKVUAZL0K2E1J&ref_=chk_typ_imgToDp
  - https://www.amazon.es/SWEYE-Capacidad%EF%BC%BDCargador-M%C3%BAltiples-Protecciones-Smartphones/dp/B0814JMRLR/ref=sr_1_18?__mk_es_ES=%C3%85M%C3%85%C5%BD%C3%95%C3%91&crid=3OCUQZILUVPAJ&keywords=bateria+5v+12v+2a+recargable&qid=1642669235&sprefix=bateria+5v+12v+2a+recargable%2Caps%2C65&sr=8-18
- On-board Raspberrypi4
  - https://es.rs-online.com/web/p/raspberry-pi/2012367
  - https://es.rs-online.com/web/p/placas-hat-y-complementos-para-raspberry-pi/2208633
- Arduino mega: for HW driver control
  - https://es.rs-online.com/web/p/arduino/7154084
- Arduino motor driver shield TB6612FNG:
  - https://es.aliexpress.com/item/4001086592215.html?spm=a2g0o.productlist.0.0.55da155eRs0f1N&algo_pvid=523f34f9-da3e-4a7e-bcbd-927dc560fb14&algo_exp_id=523f34f9-da3e-4a7e-bcbd-927dc560fb14-40
  - https://github.com/Seeed-Studio/Grove_Motor_Driver_TB6612FNG
- RaspiCAM RGB camera:
  - https://es.rs-online.com/web/p/camaras-para-raspberry-pi/9132664
  - https://es.rs-online.com/web/p/cajas-para-raspberry-pi/8679049
- rpLidar:
  - https://www.robotshop.com/es/es/rplidar-a1m8-kit-desarrollo-escaner-laser-360-grados.html
- USB cables:
  - USB-A to USB-B: https://www.amazon.es/gp/product/B073XQ33L2/ref=ppx_yo_dt_b_asin_title_o01_s00?ie=UTF8&th=1
  - USB-C to USB-B: https://www.amazon.es/gp/product/B08RXQJJ9Z/ref=ppx_yo_dt_b_asin_title_o02_s00?ie=UTF8&th=1
  - microUSB to USB-B: https://www.amazon.es/gp/product/B00WMAQKS2/ref=ppx_yo_dt_b_asin_title_o04_s00?ie=UTF8&th=1

![Getting Started](./Images/1_osoyoo.png)

Other platforms:
- https://www.superdroidrobots.com/robotic-kits-platforms/mecanum-robots
- https://www.robotshop.com/es/es/kit-robot-arduino-con-rueda-mecanum-4wd-60-mm.html
- https://www.robotshop.com/es/es/4wd-robot-basico-mecanum-compatible-con-arduino.html

### **Setup your workspace**
The raspberrypi4 onboard is preinstalled with:
- Ubuntu 20 server 32bits
  - NoMachine remote desktop
- ROS Noetic
- rubot_rbpi4_ws repository is located in /home/pi/Desktop folder 

If you do not have the "rubot_rbpi4_ws" folder in your desktop, you can "transfer" folder from your PC (it takes 30s)

The first time you copy the folder, you need to compile the workspace:
- open a terminal in the ws
- type "catkin_make" (it takes 10min)

>Carefull!: Some actions have to be done:
>- review the ~/.bashrc: source to the ws and delete the environment variables
>- make executable the c++ and python files

## **2. rUBot_mecanum first control movements**
First, let's control the rUBot_mecanum movement to perform:
- movement control using keyboard
- movement control with specific python script
- autonomous navigation
- autonomous navigation following right or left wall
- Navigation to speciffic POSE

We will create a "rubot_control" package to perform the rUBot_mecanum control movements:
```shell
cd ~/Desktop/rubot_rbpi4_ws/src
catkin_create_pkg rubot_control rospy std_msgs sensor_msgs geometry_msgs nav_msgs
cd ..
catkin_make
```
First of all you need to bringup the rubot_mecanum robot.

### **3.1. Bringup rubot_mecanum**

The bringup consists to:
- launch the rUBot node in Arduino-Mega board
- launch the LIDAR node
- launch the raspicam node

You will launch first "rubot_hw_bringup.launch" file to setup the rUBot_mecanum.

```shell
roslaunch rubot_control rubot_hw_bringup.launch
```

### **3.2. Movement control using keyboard**

To control the gopigo robot with keyboard, we need to install "teleop_tools" package. Open a new terminal and install the packages:
```shell
sudo apt-get install ros-noetic-teleop-tools
sudo apt-get install ros-noetic-teleop-twist-keyboard
```
Proceed with:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_control rubot_bringup.launch
```
- Then open a new terminal and type:
```shell
rosrun key_teleop key_teleop.py /key_vel:=/cmd_vel
or
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### **3.3. Movement control with python script**

From the "rubot_rbpi4_ws" workspace, the src/rubot_control folder has 2 new folders:
- scrip folder: with the python programs for specific movement control
- launch folder: with programs to launch the movement control

Diferent navigation programs are created:

- Navigation control: to define a desired robot velocity
- Lidar test: to verify the LIDAR readings and angles
- Autonomous navigation: to perform a simple algorithm for navigation with obstacle avoidance using the LIDAR
- Wall follower: at a fixed distance to perform a good map
- go to POSE: attend a specific position and orientation

The nodes and topics structure corresponds to the following picture:
![Getting Started](./Images/2_rubot_nodes.png)

### **a) Navigation control**

We have created a first navigation python files in "src" folder:

- rubot_nav.py: to define a rubot movement with linear and angular speed to reach a maximum x-distance
- rubot_nav1.py: to define the movement with vx, vy and w to reach a maximum distance in x or y

A "rubot_nav.launch" and "rubot_nav1.launch" files have been created to launch the node and python file created above.

To properly perform a especific movement control we have first to:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_control rubot_bringup.launch
```
- Then open a new terminal to launch the rUBot_nav node to perform the specific movement control.
```shell
roslaunch rubot_control rubot_nav.launch
```

### **b) LIDAR test**

In order to navigate autonomously and avoid obstacles, we will use a specific rpLIDAR sensor. To verify the LIDAR readings and angles we have generated the "rubot_lidar_test.py" python file:

To properly perform a especific movement control we have first to:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_control rubot_bringup.launch
```
- Then open a new terminal to launch the rUBot_nav node to perform the rpLIDAR test. We have created specific python file and launch file for this test control
```shell
roslaunch rubot_control rubot_lidar_test.launch
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```

### **c) Autonomous navigation and obstacle avoidance**

Using rpLIDAR sensor you can perform the autonomous navigation avoiding obstacles.

This performance is defined in "rubot_self_nav.py"

To properly perform a especific self-navigation control we have first to:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_control rubot_bringup.launch
```
- Then open a new terminal to launch the rUBot_nav node to perform the self-navigation. We have created specific python file and launch file for this navigation control
```shell
roslaunch rubot_control rubot_self_nav.launch
```
The launch file contains some parameters you can modify:
```xml
<launch>
  <!-- launch obstacle avoidance   -->
    <arg name="distance_laser" default="0.3" />
    <arg name="speed_factor" default="0.1"/>
    <arg name="forward_speed" default="2" />
    <arg name="backward_speed" default="-1" />
    <arg name="rotation_speed" default="20" />
    <node name="rubot_nav" pkg="rubot_control" type="rubot_self_nav.py" output="screen" >
        <param name="distance_laser" value="$(arg distance_laser)"/>
        <param name="speed_factor" value="$(arg speed_factor)"/>
        <param name="forward_speed" value="$(arg forward_speed)"/>
        <param name="backward_speed" value="$(arg backward_speed)"/>
        <param name="rotation_speed" value="$(arg rotation_speed)"/>
    </node>    
</launch>
```
In order to see the rubot with the topics information we will use rviz. Open rviz in a new terminal.

In rviz, select the fixed frame to "base_scan", and add Camera and LaserScan with the corresponding topics names.

You can then save the config file as laserscan.rviz name and use it in the launch file

![](./Images/2_self_nav.png)

A launch file is created to integrate all the needed roslaunch parameters but you can change the defauld values with this syntax:
```shell
roslaunch rubot_control rubot_self_nav.launch distance_laser:=0.2 speed_factor:=1.3
```
### **D) Wall Follower**

This control task consist on find a wall and follow it at a certain distance. We will see that this is an important control task because this will be used later to make accurate maps of working environments.

We have developed 2 different methods for wall follower:
- Geometrical method
- Lidar ranges method

#### **Geometrical method**
In src folder you create the python file for wall follower purposes

The instructions to perform the python program are in the notebook: 

https://github.com/Albert-Alvarez/ros-gopigo3/blob/lab-sessions/develop/ROS%20con%20GoPiGo3%20-%20S4.md

![](./Images/2_wall_follower_gm.png)

To properly perform a especific self-navigation control we have first to:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_control rubot_bringup.launch
```
- Then open a new terminal to launch the rUBot_nav node to perform the wall-follower. We have created specific python file and launch file for this navigation control
```shell
roslaunch rubot_control rubot_wall_follower_gm.launch
```
The launch file contains different parameters you can modify:
```xml
<launch>
  <!-- launch follow wall   -->
  <arg name="kp" default="5" />
  <arg name="distance_reference" default="0.3" />
  <arg name="lookahead_distance" default="0.4" />
  <arg name="forward_speed" default="0.04" />
  <arg name="theta" default="50.0" />
  <node name="wall_follower_controller" pkg="gopigo_control" type="rubot_wall_follower_gm.py" output="screen" >
    <param name="kp" value="$(arg kp)"/>
    <param name="distance_reference" value="$(arg distance_reference)"/>
    <param name="lookahead_distance" value="$(arg lookahead_distance)"/>
    <param name="forward_speed" value="$(arg forward_speed)"/>
    <param name="theta" value="$(arg theta)"/>
  </node>
</launch>
```
You can see the video result:

[![Click here to watch the video](https://img.youtube.com/vi/z5sAyiFs-RU/maxresdefault.jpg)](https://youtu.be/z5sAyiFs-RU)

#### **b) ranges method**
In src folder you create the python file for wall follower purposes

The algorith is based on laser ranges test and depends on the LIDAR type:

![](./Images/2_wall_follower_rg1.png)

Take into account that:
- RP LIDAR has 180º rotation
- YDlidar in front direction has 2 different ranges [660:719] and [0:60]
- YDlidar sends some 0 values due to wrong readings. They have to be changed to high value to be able to take the minimum falue from the desired range.

To properly perform a especific self-navigation control we have first to:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_control rubot_bringup.launch
```
- Then open a new terminal to launch the rUBot_nav node to perform the wall-follower. We have created specific python file and launch file for this navigation control
```shell
roslaunch rubot_control rubot_wall_follower_rg.launch
```
The launch file has no parameters to modify:

```xml
<launch>
  <!-- launch follow wall   -->
  <node name="wall_follow" pkg="gopigo_control" type="rubot_wall_follower_rg.py" output="screen" >
  </node>
</launch>
```
### **E) Go to POSE**

The objective is to program the robot to reach a speciffic target POSE defining:
- x position
- y position
- angle orientation (from 0º to 180º)

To properly perform a especific self-navigation control we have first to:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_control rubot_bringup.launch
```
- Then open a new terminal to launch the rUBot_nav node to perform the go 2 pose control. We have created specific python file and launch file for this navigation control
```shell
roslaunch rubot_control rubot_go2pose.launch
```
The launch file has no parameters to modify:

```xml
<launch>
<!-- run navigation program  -->
    <arg name="x" default="0.7"/>
    <arg name="y" default="0.7"/>
    <arg name="f" default="120"/>
    <node pkg="nexus_control" type="rubot_go2pose.py" name="nexus_control" output="screen" >
      <param name="x" value="$(arg x)"/>
      <param name="y" value="$(arg y)"/>
      <param name="f" value="$(arg f)"/>
    </node>
</launch>
```
