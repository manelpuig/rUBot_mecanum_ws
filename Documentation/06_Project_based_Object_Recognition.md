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
sudo pip install tflite-runtime
````
We will use Keras that is a high-level API that runs on top of TensorFlow. By using both TensorFlow and Keras, you get the best of both worlds: the ease of use and simplicity of Keras, combined with the power and flexibility of TensorFlow. ()

## Using the USB_Cam

A simple program to take Images:
```python
#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
```
**Capturing an image from the camera**

We gonna create a class for encapsuling the attributes and callback function:

```python
class TakeAShot(object):

 def __init__(self):
        self.image_sub = rospy.Subscriber("/camera/rgb/image_raw", Image, self.camera_callback)
        self.bridge_object = CvBridge()
```
The callback function will recieve the Image object from the camera in a missage and then it will save the Image as a png file in your 

```python
 def camera_callback(self,data):
      try:
           # We select bgr8 because its the OpenCV encoding by default
           cv_image = self.bridge_object.imgmsg_to_cv2(data, desired_encoding="bgr8")
           cv2.imwrite("image.png", cv_image)
      except CvBridgeError as e:
           print(e)        
```     		
The main is as usual:

```python
def main():

    rospy.init_node('take_a_shot', anonymous=True)
    take_a_shot = TakeAShot()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")

if __name__ == '__main__':
    main()
```

### How to send a sequence of goals to ROS NavStack

* [Sending a sequence of Goals to ROS NavStack with Python](https://hotblackrobotics.github.io/en/blog/2018/01/29/seq-goals-py/)

### Basic Tutorials for image analysis using OpenCV
* [Shape Detection Using findContourns and approxPolyDP ](https://www.pyimagesearch.com/2016/02/08/opencv-shape-detection/)
* [Arrow Interpretation using findContourns and approxPolyDP](https://programs.wiki/wiki/use-opencv-to-judge-the-arrow-direction.html)
* [Detecting lines using Canny Edges and Hough Lines](https://docs.opencv.org/3.4/d9/db0/tutorial_hough_lines.html) --> [Example code ](https://github.com/michael-pacheco/opencv-arrow-detection)
* [Color Detection using HSV space](https://www.pyimagesearch.com/2015/09/14/ball-tracking-with-opencv/)
* [HSV color picker (in opencv H value goes from 0 to 179)](https://alloyui.com/examples/color-picker/hsv.html)


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
* Install Keras and TensorFlow in the Robot: 
    ````python
    sudo pip install keras
    sudo pip install tensorflow
    ````
* Create a ROS node that uses the model for inference:
  
```python

from keras.models import load_model  # TensorFlow is required for Keras to work
import cv2  # Install opencv-python
import numpy as np

# Disable scientific notation for clarity
np.set_printoptions(suppress=True)

# Load the model
model = load_model("keras_Model.h5", compile=False)

# Load the labels
class_names = open("labels.txt", "r").readlines()

# CAMERA can be 0 or 1 based on default camera of your computer
camera = cv2.VideoCapture(0)

while True:
    # Grab the webcamera's image.
    ret, image = camera.read()

    # Resize the raw image into (224-height,224-width) pixels
    image = cv2.resize(image, (224, 224), interpolation=cv2.INTER_AREA)

    # Show the image in a window
    cv2.imshow("Webcam Image", image)

    # Make the image a numpy array and reshape it to the models input shape.
    image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)

    # Normalize the image array
    image = (image / 127.5) - 1

    # Predicts the model
    prediction = model.predict(image)
    index = np.argmax(prediction)
    class_name = class_names[index]
    confidence_score = prediction[0][index]

    # Print prediction and confidence score
    print("Class:", class_name[2:], end="")
    print("Confidence Score:", str(np.round(confidence_score * 100))[:-2], "%")

    # Listen to the keyboard for presses.
    keyboard_input = cv2.waitKey(1)

    # 27 is the ASCII for the esc key on your keyboard.
    if keyboard_input == 27:
        break

camera.release()
cv2.destroyAllWindows()
```

Here's how you can adapt your code to create a ROS node that subscribes to the USB camera topic and identifies traffic signs in real time:

- Set Up ROS Environment: Ensure ROS Noetic is installed and configured on your Raspberry Pi 4.

- Create ROS Node: Develop a ROS node that subscribes to the USB camera topic, processes the images using the Keras model, and publishes the detected traffic signs.

Code Snippet:
````python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from keras.models import load_model

class TrafficSignRecognizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.model = load_model("keras_Model.h5", compile=False)
        self.class_names = open("labels.txt", "r").readlines()
        rospy.init_node('traffic_sign_recognizer', anonymous=True)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("/traffic_sign_detection/image", Image, queue_size=10)

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.resize(frame, (224, 224), interpolation=cv2.INTER_AREA)
        image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
        image = (image / 127.5) - 1
        prediction = self.model.predict(image)
        index = np.argmax(prediction)
        class_name = self.class_names[index]
        confidence_score = prediction[0][index]
        if confidence_score > 0.5:
            cv2.putText(frame, f"Sign: {class_name[2:].strip()} ({confidence_score:.2f})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

if __name__ == '__main__':
    try:
        TrafficSignRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
````
With tflite-runtime:
````python
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import tflite_runtime.interpreter as tflite

class TrafficSignRecognizer:
    def __init__(self):
        self.bridge = CvBridge()
        self.interpreter = tflite.Interpreter(model_path="traffic_sign_model.tflite")
        self.interpreter.allocate_tensors()
        self.input_details = self.interpreter.get_input_details()
        self.output_details = self.interpreter.get_output_details()
        rospy.init_node('traffic_sign_recognizer', anonymous=True)
        self.image_sub = rospy.Subscriber("/usb_cam/image_raw", Image, self.callback)
        self.image_pub = rospy.Publisher("/traffic_sign_detection/image", Image, queue_size=10)

    def callback(self, data):
        frame = self.bridge.imgmsg_to_cv2(data, "bgr8")
        image = cv2.resize(frame, (224, 224), interpolation=cv2.INTER_AREA)
        image = np.asarray(image, dtype=np.float32).reshape(1, 224, 224, 3)
        image = (image / 127.5) - 1
        self.interpreter.set_tensor(self.input_details[0]['index'], image)
        self.interpreter.invoke()
        output_data = self.interpreter.get_tensor(self.output_details[0]['index'])
        index = np.argmax(output_data)
        class_name = self.class_names[index]
        confidence_score = output_data[0][index]
        if confidence_score > 0.5:
            cv2.putText(frame, f"Sign: {class_name[2:].strip()} ({confidence_score:.2f})", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        self.image_pub.publish(self.bridge.cv2_to_imgmsg(frame, "bgr8"))

if __name__ == '__main__':
    try:
        TrafficSignRecognizer()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
````
