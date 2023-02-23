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
## **2. Connection using VS Code**
You need the latest version (1.75.1).

First time you will need to connect the rock-board to internet with ethernet cable ti install some capabilities.

Install the extensions.
- Remote development
- Git Extension Pack (in your remote 10.42.0.1 board)

The first time you connect the VS Code to the Remote machine:
- You will need to sign autorization to VS code to access github (will be in left-side bar menu, accounts symbol)
- When sync the changes, you will have to type your email and user name (following the git output information)
- You can work without internet connection, but When you want to sync your repository you will need the ethernet cable connected.

Follow the instructions:
- In windows open VS Code as administrator
- Select the rUBot_XX wifi network 
- From "remote Explorer" (left-side bar menu) select "Remote" and type the IP
- Select linux system for remote connection
- type the password

You will have the VS Code attached to remote machine!
