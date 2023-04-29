# **Air Quality control**

We have designed and assembled an IoT closed loop air quality control device to be mounted on rUBot_2.0 prototype.

The mechanical structure is descrived bellow:
![](./Images/rubot_custom/1_osoyoo.png)

His main characteristics are: 
- xxx
- yyy

In this document we will describe:
- Device structure
- Web arduino SW description
- Sensors description
- Ozone generation device description
- PCB board design
- Arduino code 
- Experimental results


## **1. Device structure**

First of all, we have to create the "rubot_mecanum_description" package where we will create the rUBot model. In case you want to create it from scratch, type:
```shell
cd ~/Desktop/ROS_github/rubot_mecanum_ws/src
catkin_create_pkg rubot_mecanum_description rospy
cd ..
catkin_make
```