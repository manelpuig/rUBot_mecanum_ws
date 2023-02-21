# **rUBot mecanum in Rock Install**

The main objectives are:

- Setup the HW drivers in rock4 SE
- Control the rUBot movement

Let's see how to fulfill these objectives

References:

## **1. Setup the rubot in rock4**
The rock4 onboard is preinstalled with:
- Ubuntu 20 server 64bits
  - NoMachine remote desktop
- ROS Noetic

First you connect to the rock4 hotspot:
- SSID: rubot_00
- Passworg: rUBot_Mec

Second you connect to rock4 with Nomachine remote desktop:
- IP: 10.42.0.1
- user: pi, ubuntu, rock
- password: rockbian, rockubuntu, rock

You will need to change the Keyboard to Spanish keyboard. 

Open a terminal and type:
```shell
sudo setxkbmap -layout 'es,es' -model pc105
setxkbmap es sundeadkeys
```
Change to a bash terminal:
```shell
bash
```
Open .bashrc and add:
```shell
source /opt/ros/noetic/setup.bash
```
