# **rUBot mecanum challenging projects**

The projects proposed will be based on:

- Video and picture caption
- rUBot navigation with Image processing

The different projects will be:

- 1. rUBot takes photo
- 2. Go to specific point in the map and take a photo
- 4. Line follower

References OpenCV:

- http://wiki.ros.org/cv_bridge/Tutorials/ConvertingBetweenROSImagesAndOpenCVImagesPython
- https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html
- https://github.com/Akshay594/OpenCV/tree/master/tutorials

References InterRealSense:

- https://dev.intelrealsense.com/docs/ros-wrapper
- https://www.youtube.com/watch?v=GhHvuAoFC6I
- https://intel.github.io/robot_devkit_doc/pages/rs_slam.html

References for webcam:

- https://automaticaddison.com/working-with-ros-and-opencv-in-ros-noetic/

References:

- https://learn.turtlebot.com/
- https://learn.turtlebot.com/2015/02/04/1/
- https://learn.turtlebot.com/2015/02/04/2/
- https://learn.turtlebot.com/2015/02/04/3/
- https://github.com/markwsilliman/turtlebot
- http://wiki.ros.org/Camera%2BDynamixelRobotSample/CameraPictureServer
- https://industrial-training-master.readthedocs.io/en/melodic/_source/session5/OpenCV-in-Python.html

The first step is to create a new package "rubot_projects" with dependencies:

- rospy
- sensor_msgs
- std_msgs
- cv_bridge

```shell
catkin_create_pkg rubot_projects rospy std_msgs sensor_msgs cv_bridge
```

This package is already created and ready to use it!. You have not to create it.

We will perform some specific projects related to rUBot vision capabilities in a navigation process.

## **Project 1: rUBot takes photo**

The objective is to program a python code to take a photo using usb-cam in robot prototype.

Important information is taken from: https://learn.turtlebot.com/2015/02/04/3/

Follow the procedure:

- Identify the topic name where raspicam publishes the photo as a mesage of type sensor_msgs:

```shell
roslaunch rubot_slam rubot_slam_bringup_sw.launch 
rostopic list
```

- Then modify the "take_photo.py" python file with:
  - the proper topic name /rubot/camera1/image_raw
  - the proper photo filename in folder path: ./src/rubot_projects/photos/photo_sim.jpg
- run the "take_photo.py" python file to take a photo

```shell
rosrun rubot_projects take_photo.py
```

- Open the "photos" folder and you will see the photo1.jpg created

![](./Images/05_rubot_projects/1_photo1.png)


## **Project 2: Navigate to a sequence of goals in the map and take a photo**

The objective is to follow the route and take pictures.

We will combine the two programs:

- Send a sequence of goals to navigation stack
- Take a Photo 

We will take a "goals_foto.yaml" file to specify the POSE goal and Photo path-name.

Proceed with the following steps:

- Launch bringup in Gazebo virtual environment:

  ```shell
  roslaunch rubot_projects rubot_projects_bringup_sw.launch
  ```
- Run the navigation demo:

  ```shell
  roslaunch rubot_slam rubot_navigation.launch
  ```
- Launch the "rubot_nav_picture.launch" program:

  ```shell
  roslaunch rubot_projects rubot_nav_picture.launch
  ```

> Careful!:
> Be sure to execute the rosrun instruction inside the "rubot_mecanum_ws" folder. Review the the absolute path or relative path to the yaml file and the picture path destination.

![](./Images/05_rubot_projects/project2_nav_photo.png)

**Exercise:** Note that the picture is not saved!. Solve this issue.

## **Project 3: Line follower**

Important information can be obtained here:

- https://www.theconstructsim.com/morpheus-chair-create-a-linefollower-with-rgb-camera-and-ros-episode-5/
- https://www.youtube.com/watch?v=9C7Q8bRERgM
- https://github.com/noshluk2/ROS2-Self-Driving-Car-AI-using-OpenCV

Related to the links:

- http://www.rosject.io/l/8292943/
- https://en.wikipedia.org/wiki/Differential_wheeled_robot

And with the code:

- https://bitbucket.org/theconstructcore/morpheus_chair/src/master/

The nexts steps will be:

- Define a proper simulated world in Gazebo
- spawn the robot model in Gazebo world
- setup the robot with camera
- start the line follower node

### **World setup**

We have created different models to include in gazebo world:

- Trafic signs

- road

We will construct first these models in a specific folder:

- rubot_mecanum_ws/src/rubot_mecanum_description/models

We have to add this folder to GAZEBO_MODEL_PATH tenvironment variable. This is done either:

- in ~/.bashrc file adding this line:

```xml
export GAZEBO_MODEL_PATH=$HOME/rUBot_mecanum_ws/src/rubot_mecanum_description/models:$GAZEBO_MODEL_PATH
```

> If you want to delete any model path from gazebo, load the "gui.ini" file from .gazebo folder. There is a list of model paths and you can delete the one you do not want

- or copy the models folder in ~/.gazebo/models/
- you can always add a folder in "insert" tag of Gazebo

#### **a) Traffic sign**

Let's create a "sign board 30" model:

- Open Gazebo as superuser (sudo gazebo)
- select edit --> Model Editor
- add the meshes (obj files or standard objects) needed to create the sign model
- adjust the size and place the objects in the correct positions to be assembled
- select joint in gazebo (fixed), define the parent and child links, adjust the relative pose, change the joint name if you want and create the model
- save the model as "sign_stand"
- You will see the folder created for this model with 2 files (model.config and model.sdf)
- in model.sdf you can:
  - reduce the mass of the upper links for inertial stability.
  - change the defauld color (Gazebo/Grey)
  - change the dimensions and pose of different links if necessary
- open gazebo and add the generated model to verify the size and mecanical stability.

This model will be used to create all the other traffic signs, for exemple the turn traffic sign:

- Make a copy of this folder with the name "turn_left"
- in model.config file change the name to "turn_left"
- add materials and meshes folders inside "turn_left"
- In materials folder add scripts and textures folder
- In textures folder add the png file with the sign picture (turn_left.png)
- in scripts add a turn_left.material file with this contents (specify the turn_left.png file):

```xml
material turn_left/Diffuse
{
  technique
  {
    pass
    {
      texture_unit
      {
        texture turn_left.png
        filtering anistropic
        max_anisotropy 16
      }
    }
  }
}
```

- Open the model.sdf and change the material properties of link01 where we want to place the turn left texture. Replace the text:

```xml
        <material>
          <lighting>1</lighting>
          <script>
            <uri>file://media/materials/scripts/gazebo.material</uri>
            <name>Gazebo/White</name>
          </script>
          <shader type='pixel'/>
          <emissive>0 0 0 1</emissive>
        </material>
```

by this text:

```xml
        <material>
          <script>
            <uri>model://turn_left/materials/scripts</uri>
            <uri>model://turn_left/materials/textures</uri>
            <name>turn_left/Diffuse</name>
          </script>
        </material>
```

- you have now the turn traffic sign ready!

To add models in our world add each model in the last part of your world file (here starts with empy.world):

```xml
<sdf version="1.5">
  <world name="default">
    <!-- A global light source -->
    <include>
      <uri>model://sun</uri>
    </include>
    <!-- A ground plane -->
    <include>
      <uri>model://ground_plane</uri>
    </include>
    <!-- A traffic sign -->
    <include>
      <uri>model://sign_left_turn</uri>
      <pose>0 0 0.5 0 0 0</pose>
    </include>
    <!-- Line Test 1 -->
    <include>
      <uri>model://road_base</uri>
      <name>road_base</name>
      <pose>0 0 0 0 0 0</pose>
    </include> 
  </world>
</sdf>
```

We spawn our robot into gazebo world:

```shell
roslaunch rubot_mecanum_description rubot_bringup_sw.launch
```

To see the camera image, type:

```shell
rosrun rqt_image_view rqt_image_view
```

Open line_follower.py and:

- change the camera_topic="/rubot/camera1/image_raw", cmd_vel_topic="/cmd_vel"
- be sure to have the "rgb_hsv.py" file in src folder
  Start the node line_follower

```shell
roslaunch rubot_projects line_follower_start.launch
```

![](./Images/5_line_follower1.png)

see: https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_imgproc/py_colorspaces/py_colorspaces.html
