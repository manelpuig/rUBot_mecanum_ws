# **SLAM & Navigation rUBot mecanum**
Autonomous navigation refers that the robot is able to move autonomously around the environment avoiding any obstacle.

In a logistics warehouse, a delivery robot carries samples or food from one room to another. 

The main objectives are:
- use SLAM (Simultaneous Localization and Mapping) techniques to generate and store a map of the logistics warehouse flor
- use Navigation techniques to find an optimal trajectory to reach a speciffic logistics warehouse target position

SLAM is a technique used in robotics to explore and map an unknown environment while estimating the pose of the robot itself. As it moves all around, it will be acquiring structured information of the surroundings by processing the raw data coming from its sensors.

let's see how to fulfill these objectives


## **Install ROS navigation & SLAM packages**

First, let's prepare your machine with the required ROS packages needed for the navigation stack (http://wiki.ros.org/navigation):
```shell
sudo apt install ros-noetic-navigation
```
And finally the slam_gmapping package, that is already available in its binary version (https://wiki.ros.org/slam_gmapping)
```shell
sudo apt install ros-noetic-slam-gmapping
```
Also for some transformations you will need to install transforms3d:
```shell
sudo pip3 install transforms3d
```
Open the .bashrc file and verify to source to the proper workspace:
```shell
source /home/user/rUBot_mecanum_ws_/devel/setup.bash
```
change the path corresponding to your ws

To perform SLAM & Navigation, we need to create a specific "rubot_slam" package.

This package is already created in the simulation ws. Take the same "rubot_slam" package and make few modifications to use it to:
- generate the map of your maze
- navigate to speciffic target points within the map

This repository is essentially the one corresponding to turtlebot3 adapted for the mecanum robot prototype:

https://github.com/ROBOTIS-GIT/turtlebot3

Now you can follow the next steps:

### **1. Generate the map**

To generate the map we need first to:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_slam rubot_slam_bringup_sw.launch
```
- Start the slam_gmapping node
```shell
roslaunch rubot_slam rubot_slam.launch
```
This node is highly configurable and has lots of parameters you can change in order to improve the mapping performance. (http://wiki.ros.org/gmapping)

Let's now check some of the most important ones that usually have to be changed:

- base_frame (default: "base_link"): Indicates the name of the frame attached to the mobile base.
- map_update_interval (default: 5.0): Sets the time (in seconds) to wait until update the map. (I take 1s)
- delta (default: 0.05): Sets the resolution of the map (I take 0.01m)

Open the "gmapping.launch" file and change properly the parameters you consider. 

- use the navigation program you have designed to follow the walls for exemple to properly generate the map.
```shell
roslaunch rubot_control rubot_wall_follower_rg.launch
```
- or let's do this as usual with the teleoperation package:
```shell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
- Once you have finish the map, you need to launch the map_saver program from map_server package:
```shell
cd src/rubot_slam/maps
rosrun map_server map_saver -f test_map
```
The map files will be saved in local directory

You will get two files in the specified folder of your workspace: 
- test_map.pgm (2D B&W map picture)
- test_map.yaml (map parameters)

Provided with the map, we are ready to perform robot navigation with the rUBot_mecanum robot.

You can close now the "rubot_slam.launch" file.

### **2. Navigate to speciffic target points within the map**

When the robot moves around a map, it needs to know which is its POSE within the map.

The AMCL (Adaptive Monte Carlo Localization) package provides the amcl node, which uses the MCL system in order to track the localization of a robot moving in a 2D space. This node subscribes to the data of the laser, the laser-based map, and the transformations of the robot, and publishes its estimated position in the map.

On startup, the amcl node initializes its particle filter according to the parameters provided.

This AMCL node is also highly customizable and we can configure many parameters in order to improve its performance. (http://wiki.ros.org/amcl)

Let's have a look at some of the most important ones:

**General Parameters**

These parameters will allow you to configure how the localization is performed and are located in "amcl.launch" file:

- odom_model_type (default: "diff"): It puts the odometry model to use. It can be "diff," "omni," "diff-corrected," or "omni-corrected."
- base_frame_id (default: "base_link"): Indicates the frame associated with the robot base.

**Costmap Parameters**

These parameters will allow you to configure the way that the navigation is performed and are located in "costmap_common_params.yaml" file:

- footprint: dimensions of the base_footprint for colision information
- Inflation radius: increase dimensions of obstacles to prevent colisions
- cost scaling factor: to define allowed regions among obstacles to define the optimal trajectory.

Review these parameters in "costmap_common_params.yaml"

You can refine some parameters considering the recommendations in: https://emanual.robotis.com/docs/en/platform/turtlebot3/navigation/#tuning-guide

>Careful!!
>
>global & local_costmap_params.yaml: specify the robot_base_frame as "base_footprint" link according to the URDF file. This link is the first one and has TF connection to the world

To navigate within the map we need first to:
- Bringup rUBot_mecanum (if you have closed it before)
```shell
roslaunch rubot_slam rubot_slam_bringup_sw.launch
```
- Launch the rubot_navigation file
```shell
roslaunch rubot_slam rubot_navigation.launch
```
>Take care in launch file to read the correct robot model name and the correct map file in "maps" folder
- Set up an initial pose by using the 2D Pose Estimate tool (which published that pose to the /initialpose topic).
- To obtain a proper localisation of your robot, move it right and left using the key_teleop.
```shell
rosrun teleop_twist_keyboard teleop_twist_keyboard.py
```
- Select the target goal to navigate with the tool "2D-Nav-Goal"

- Using "omni" drive performances, the robot is able to move also in y direction and the mobility is much better.

- You need to modify the "dwa_local_planner_params_burger.yaml parameters. An exemple of possible parameters set is:
```xml
DWAPlannerROS:

  holonomic_robot: true           #false
# Robot Configuration Parameters  Defaults
  max_vel_x: 1                    #0.22
  min_vel_x: -1                   #-0.22

  max_vel_y: 0.5                    #0.0
  min_vel_y: -0.5                   #0.0

# The velocity when robot is moving in a straight line
  max_vel_trans:  1               #0.22
  min_vel_trans:  0.2             #0.11

  max_vel_theta: 1              #2.75
  min_vel_theta: -1              #1.37

  acc_lim_x: 2.5
  acc_lim_y: 0.5                  #0.0
  acc_lim_theta: 3.0              #3.2 

# Goal Tolerance Parametes
  xy_goal_tolerance: 0.1          #0.05
  yaw_goal_tolerance: 0.1         #0.17
  latch_xy_goal_tolerance: false

# Forward Simulation Parameters
  sim_time: 1.5
  vx_samples: 20
  vy_samples: 20                  #0
  vth_samples: 40
  controller_frequency: 5.0      #10.0

# Trajectory Scoring Parameters
  path_distance_bias: 32.0
  goal_distance_bias: 20.0
  occdist_scale: 0.02
  forward_point_distance: 0.325
  stop_time_buffer: 0.2
  scaling_speed: 0.25
  max_scaling_factor: 0.2

# Oscillation Prevention Parameters
  oscillation_reset_dist: 0.05

# Debugging
  publish_traj_pc : true
  publish_cost_grid_pc: true
```

You can see the optimized trajectory. The gopigo starts to follow this trajectory and if an obstacle appears, the robot avoid this obstacle and find in realtime the new optimal trajectory to the target point. 

