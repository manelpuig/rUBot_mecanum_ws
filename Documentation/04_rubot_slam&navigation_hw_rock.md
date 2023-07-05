# **SLAM & Navigation rUBot mecanum with Rock5b**
Autonomous navigation refers that the robot is able to move autonomously around the environment avoiding any obstacle.

In a hospital, a delivery robot carries samples or food from one room to another. 

The main objectives are:
- use SLAM (Simultaneous Localization and Mapping) techniques to generate and store a map of the hospital flor
- use Navigation techniques to find an optimal trajectory to reach a speciffic hospital target position

let's see how to fulfill these objectives


## **Using Odometry for AMCL localization**

To perform SLAM & Navigation, we need to create a specific "rubot_slam" package.

This package is already created in the simulation ws. Take the same "rubot_slam" package and make few modifications to use it to:
- generate the map of your maze
- navigate to speciffic target points within the map

### **1. Generate the map**

To generate the map we need first to:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_mecanum_description rubot_bringup_hw_rock.launch
```
- Launch the rubot_slam file
```shell
roslaunch rubot_slam rubot_slam.launch
```
>If we use new values of "gmapping.launch" parameters:
>- delta: 0.01m 
>- map_update_interval: 1s
>
>we obtain a more accurate movement and map

- use the navigation program you have designed to follow the walls for exemple to properly generate the map.
```shell
roslaunch rubot_control rubot_wall_follower_gm.launch
```

- Once you have finish the map, you need to launch the map_saver program from map_server package:
```shell
rosrun map_server map_saver -f hospital1map
```
The map files will be saved in local directory

You will get two files in the specified folder of your workspace: hospital1map.pgm and hospital1map.yaml.

Provided with the map, we are ready to perform robot navigation with the rUBot_mecanum robot.

### **2. Navigate to speciffic target points within the map**

To navigate within the map we need first to:
- Bringup rUBot_mecanum
```shell
roslaunch rubot_mecanum_description rubot_bringup_hw_rock.launch
```
- Launch the rubot_navigation file
```shell
roslaunch rubot_slam rubot_navigation.launch
```

- Set up an initial pose by using the 2D Pose Estimate tool (which published that pose to the /initialpose topic).
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
