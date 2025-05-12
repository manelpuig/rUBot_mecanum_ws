# Project based on Object recognition

We will describe the Computer Vision based method to identify the Traffic Sign

## ROS packages installation

The needed packages installation instructions:
````shell
sudo apt-get update
sudo apt-get install python3-opencv
sudo apt install python3-pip
sudo pip install numpy matplotlib
sudo pip install keras
sudo pip install tensorflow
````
We will use Keras that is a high-level API that runs on top of TensorFlow. By using both TensorFlow and Keras, you get the best of both worlds: the ease of use and simplicity of Keras, combined with the power and flexibility of TensorFlow. 

## Getting started

1. Bringup the rUBot:
   ```bash
   roslaunch rubot_mecanum_description rubot_bringup_HW_arduino.launch
   ````
   > Remember this is already done if you are using the bringup service
2. Place the rUBot in front of the Traffic Signal:
   
   You will create a set of around 100 photos for each traffic signal and you will copy the photos for each traffic signal in a local PC folder with a custom name (Stop, Turn_Right, Turn_Left, forbidden direction, give way, etc)

3. Open RVIZ to see the picture frame
    ````bash
    rviz
    ````
    > Add the topic /usb_cam/image_raw/Image to see the camera view before starting the 'take_photo' node

4. Take photos from USB camera
   
   - For a first time model you can use:
      ````bash
      roslaunch rubot_projects rubot_take_photo.launch
      ````
      > You can change the rate that is 1Hz by default
   
5. Open the "teachablemachine" app to create a model for "Traffic Signs". Go to https://teachablemachine.withgoogle.com/ and create an image project. Select "Standard Image model" and create a Class name for each Traffic sign with the exact name you have choosen for each sign (Stop, Turn_Right, Turn_Left, forbidden_direction, give_way, etc). 
   - Copy the photos you have made with the robot from the local PC folder for each sign and upload them to each class of the project.  
   - Train the model.  
   - Open "Export model" and select "Tensorflow" option and choose "Download model"
   - A zip file is created in PC Download folder. It contains a folder with "keras_model.h5" and "labels.txt". Change the names if needed. The model can be created with some few photos, but this would be improved with some more photos in a later step.  
   - These two files model will be uploaded in "models" folder project 

6. Launch classification node to verify the performance of your generated model:

   Verify the topic name: /usb_cam/image_raw to place it in "keras_detect_signs.py" node and the names of the model and the Classes on labels.txt
   
   ```bash
   roslaunch rubot_projects keras_detect_signs.launch
   ````
7. Refine the model:

   When you already have a first model and you want to refine this model with more and better photos, you can use a more performand program to take photos continuously, make an identification with that actual model and store the photos to the corresponding folder with the name of the class. These new photos will help you to improve the performances of your model.
   - Launch "keras_takePictures_detect_signs.launch" file created for this purpose:
      ```bash
      roslaunch rubot_projects keras_takePictures_detect_signs.launch
      ````
      - Stop doing photos and only identify:
         ````bash
         rostopic pub /capture_toggle std_msgs/Bool "data: false"
         ````
      - If you want to do photos again set the argument to true 
         ```bash
         rostopic pub /capture_toggle std_msgs/Bool "data: true"
         ````
         >This is the default setting
   - Update the model

      You have to select "Tensorflow.js" and upload the model to the Cloud. A link appears corresponding of the model in the cloud.

      Open the link in a new google tip with this link. The model appears with the previous photos. You can add there the new ones.

8. Perform a trajectory in your world from Initial POSE to final POSE takin care of the different traffic signs 

   This is the core of your project. Different traffic signals will be placed on your world and the robot will be able to identify them and generate a proper tarjectory to reach the Final POSE takin care the traffic signals.
      - Contruct the desired world with the available wooden parts
      - Launch the slam gmapping node to generate the map
      - Launch the navigation node to:
         - Localize the initial POSE of the robot in the map
         - Localize the traffic signal POSEs (you will use later)
         - define a destination POSE
         - Navigation node generates an optimal trajectory and starts the movement
      -  Launch the node "keras_takePictures_detect_signs_move" who is able to identify the traffic signal and execute the corresponding movemment. 
      ````shell
      roslaunch rubot_projects keras_takePictures_detect_signs_move.launch
      ````
      > You will have to modify this node to subscribe to the Odometry and start to execute the corresponding movement when the robot is close enough (i.e. 40cm) of the traffic signal. A first version of this node is implemented in: keras_takePictures_detect_signs_move.py

### Final Project:

* Load the room map 
* Start the navigation stack. 
* Get the coordinates of all the traffic sign using Rviz.
* Write a Python node with the folowing behaviour:
	* Localize the robot in the starting POSE
   - Define a goal and start the movement to the defined goal
   - If you find a traffic sign execute the corresponding movement when you are close (i.e. aroud 30cm) to the sign. This movement has to be:
      - Stop: publish a Twist message (0,0,0,0,0,0)
      - Give_Way: publish a Twist message (0,0,0,0,0,0) during 5 seconds
      - Turn_Right: publish a Twist message (0,0,0,0,0,-wz)
      - Turn_Left: publish a Twist message (0,0,0,0,0,+wz)
	* continue to the target pose according to a new path obtained by Navigation stack during the previous movement
* Test a more complex escenario when there are 2 or 3 traffic signs involved in the path from init pose to target pose.

**Documentation:**

* [Sending a sequence of Goals to ROS NavStack with Python](https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/)
