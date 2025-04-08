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
2. Place the rUBot in front of the Traffic Signal

3. Take photos from USB camera
    ````bash
    rosrun rubot_projects rubot_take_photo.py
    ````
    You can use a more performand program to take photos continuously, add data in picture name for model generation and also to detect traffic signs when model is created.
    ````bash
    rosrun rubot_projects keras_takePictures_detect_signs.py
    ````
4. Open RVIZ to see the picture frame
    ````bash
    rviz
    ````
- Open the "teachablemachine" app to create a model for "Traffic Signs". Go to https://teachablemachine.withgoogle.com/ and create an image project.  
- Collect images with rUBot USB_CAM for each sign and upload them to the project.  
- Train the model.  
- Export the model as a keras .h5 model. The model can be created with some pictures, but this would be improved with some more pictures.  
- The models will be uploaded in "models" folder  

8. Verify topics (theConstruct):
   ```bash
   rostopic list
   ````
   Verify the topic name: /usb_cam/image_raw to place it in "keras_detect_signs.py" node
   
9. Launch classification node (theConstruct):
   ```bash
   roslaunch rubot_projects detect_signs.launch
   ````
10. Launch classification node and take photos (v2) (theConstruct):
   ```bash
   roslaunch rubot_projects detect_signs_take_pictures.launch
   ````
10.1. Stop doing photos (theConstruct) (in a new Terminal):
   ```bash
   rostopic pub /capture_toggle std_msgs/Bool "data: false"
   ````
   If you want to do photos again set false value to true 
### How to send a sequence of goals to ROS NavStack

* [Sending a sequence of Goals to ROS NavStack with Python](https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/)



### Final Exercise:
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

