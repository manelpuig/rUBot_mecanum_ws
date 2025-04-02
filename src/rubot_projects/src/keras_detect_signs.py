#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import cv2
import numpy as np
from tensorflow.keras.models import load_model
import os

class KerasImageClassifier:
    def __init__(self):
        # Paths
        script_dir = os.path.dirname(os.path.realpath(__file__))
        model_path = rospy.get_param("~model_path", os.path.join(script_dir, "../models/keras_model.h5"))
        labels_path = rospy.get_param("~labels_path", os.path.join(script_dir, "../models/labels.txt"))

        # Load model and labels
        self.model = load_model(model_path)
        self.labels = self.load_labels(labels_path)

        # Get input size
        self.input_shape = self.model.input_shape[1:3]  # e.g. (224, 224)
        rospy.loginfo(f"Expecting image size: {self.input_shape}")

        self.bridge = CvBridge()

        # ROS communication
        self.sub = rospy.Subscriber("/camera/image_raw", Image, self.image_callback, queue_size=1)
        self.pub = rospy.Publisher("/predicted_class", String, queue_size=1)

    def load_labels(self, path):
        with open(path, 'r') as f:
            lines = f.readlines()
            return [line.strip().split(' ', 1)[1] for line in lines]

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = cv2.resize(cv_image, self.input_shape)
            img = img.astype(np.float32) / 255.0
            img = np.expand_dims(img, axis=0)

            predictions = self.model.predict(img)
            class_index = np.argmax(predictions)
            class_name = self.labels[class_index]

            rospy.loginfo(f"Predicted: {class_name}")
            self.pub.publish(class_name)

        except Exception as e:
            rospy.logerr(f"Error processing image: {e}")

if __name__ == "__main__":
    rospy.init_node("keras_detect_signs")
    KerasImageClassifier()
    rospy.spin()
