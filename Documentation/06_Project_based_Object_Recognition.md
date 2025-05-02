# Project based on Object recognition

We will describe the Computer Vision based method to identify the Traffic Sign

## ROS packages installation

The needed packages installation instructions:
````shell
sudo apt-get update
sudo apt-get upgrade
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

3. Take photos from USB camera
   
   - For a first time model you can use:
      ````bash
      rosrun rubot_projects rubot_take_photo.py
      ````
      > You can change the rate that is 1Hz by default
   - When you already have a first model and you want to refine this model with more and better photos, you can use a more performand program to take photos continuously, add data in picture name for model generation and also to detect traffic signs using the previous model and copy the photo to the corresponding folder with the name of the detected traffic signal.
      ````bash
      rosrun rubot_projects keras_takePictures_detect_signs.py
      ````
4. Open RVIZ to see the picture frame
    ````bash
    rviz
    ````
    > Add the topic /usb_cam/image_raw/Image to see the camera view before starting the 'take_photo' node

5. Open the "teachablemachine" app to create a model for "Traffic Signs". Go to https://teachablemachine.withgoogle.com/ and create an image project. Select "Standard Image model" and create a Class name for each Traffic sign with the exact name you have choosen for each sign (Stop, Turn_Right, Turn_Left, forbidden direction, give way, etc). 
   - Copy the photos you have made with the robot from the local PC folder for each sign and upload them to each class of the project.  
   - Train the model.  
   - Export the model as a "keras .h5 model". The model can be created with some few photos, but this would be improved with some more photos in a later step.  
   - The downloaded model will be uploaded in "models" folder project 

6. Verify topics (theConstruct):
   ```bash
   rostopic list
   ````
   >Verify the topic name: /usb_cam/image_raw to place it in "keras_detect_signs.py" node
   
7. Launch classification node (theConstruct):
   ```bash
   roslaunch rubot_projects detect_signs.launch
   ````
8. Launch classification node and take photos (v2) (theConstruct):
   ```bash
   roslaunch rubot_projects detect_signs_take_pictures.launch
   ````
      - Stop doing photos (theConstruct) (in a new Terminal):
         ````bash
         rostopic pub /capture_toggle std_msgs/Bool "data: false"
         ````
      - If you want to do photos again set false value to true 
         ```bash
         rostopic pub /capture_toggle std_msgs/Bool "data: true"
         ````
### How to send a sequence of goals to ROS NavStack

* [Sending a sequence of Goals to ROS NavStack with Python](https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/)

### Final Project:
#### First scenario: Image processing

* Load the room map 
* Start the navigation stack. 
* Get the coordinates of all the traffic sign using Rviz.
* Write a Python node with the folowing behaviour:
	* Go to the first sign.
	* When the robot stops:
		* take an image. 
		* recognize the object using image processing by coulour and shape. 
		* go to next sign according the signs:
			* Left: go to next sign on the left
			* Right: go to next sign on the right
			* STOP: Shutdown node.
			* Forbidden: continues to the next sign  	 

#### Second scenario: Computer Vision

* Load the room map 
* Start the navigation stack. 
* Get the coordinates of all the traffic sign using Rviz.
* Write a Python node with the folowing behaviour:
	* Go to the first sign.
	* When the robot stops:
		* take an image. 
        * recognize the object using TensorFlow model and do inference on each image
		* go to next sign according the signs:
			* Left: go to next sign on the left
			* Right: go to next sign on the right
			* STOP: Shutdown node.
			* Forbidden: continues to the next sign  

### How to train and use a TensorFlow model for object recognition:
* Go to https://teachablemachine.withgoogle.com/ and create an image project.
* Collect images with your mobile for each sign and upload them to the project.
* Train the model.
* Export the model as a keras .h5 model: 
* Create a ROS node that uses the model for inference:
  

Here's how you can adapt your code to create a ROS node that subscribes to the USB camera topic and identifies traffic signs in real time:

- Set Up ROS Environment: Ensure ROS Noetic is installed and configured on your Raspberry Pi 4.

- Create ROS Node: Develop a ROS node that subscribes to the USB camera topic, processes the images using the Keras model, and publishes the detected traffic signs.

