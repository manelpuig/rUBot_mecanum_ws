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

6. Launch classification node (theConstruct):

   Verify the topic name: /usb_cam/image_raw to place it in "keras_detect_signs.py" node and the names of the model
   
   ```bash
   roslaunch rubot_projects keras_detect_signs.launch
   ````
7. Launch classification node and take photos:

   When you already have a first model and you want to refine this model with more and better photos, you can use a more performand program to take photos continuously, make an identification with tha actual model and the photo to the speciffic folder with the name of the class.
   ```bash
   roslaunch rubot_projects keras_takePictures_detect_signs.launch
   ````
      - Stop doing photos (theConstruct) (in a new Terminal):
         ````bash
         rostopic pub /capture_toggle std_msgs/Bool "data: false"
         ````
      - If you want to do photos again set false value to true 
         ```bash
         rostopic pub /capture_toggle std_msgs/Bool "data: true"
         ````
         >This is the default setting
   8. To update the model

      You have to select "Tensorflow.js" and upload the model to the Cloud. A link appears corresponding of the model in the cloud.

      Open the link in a new google tip with this link. The model appears with the previous photos. You can add there the new ones.

### Final Project:

* Load the room map 
* Start the navigation stack. 
* Get the coordinates of all the traffic sign using Rviz.
* Write a Python node with the folowing behaviour:
	* Lacalize the robot in the starting POSE
   - Define a goal and start the movement to the defined goal
   - If you find a traffic sign execute the corresponding movement when you are close (aroud 30cm) to the sign. This movement has to be:
      - Stop: publish a Twist message (0,0,0,0,0,0)
      - Turn_Right: publish a Twist message (0,0,0,0,0,-wz)
      - Turn_Left: publish a Twist message (0,0,0,0,0,+wz)
	* continue to the target pose according to a new path obtained by Navigation stack during the previous movement
* Test a more complex escenario when there are 2 or 3 traffic signs involved in the path from init pose to target pose.

**Documentation:**

* [Sending a sequence of Goals to ROS NavStack with Python](https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/)
