# **rUBot Custom model & control**

We have designed and assembles a custom mecanum robot.

The mechanical structure is descrived bellow:
![](./Images/rubot_custom/1_osoyoo.png)

His main characteristics are: 
- xxx
- yyy

In this document we will describe:
- rUBot_2.0 model
- Bringup SW and HW
- Kinematic Movement Control
- Self-Navigation
- Wall follower
- Line follower
- Traffic signal identification


## **1. rUBot_2.0_ model**

First of all, we have to create the "rubot_mecanum_description" package where we will create the rUBot model. In case you want to create it from scratch, type:
```shell
cd ~/Desktop/ROS_github/rubot_mecanum_ws/src
catkin_create_pkg rubot_mecanum_description rospy
cd ..
catkin_make
```
