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
        