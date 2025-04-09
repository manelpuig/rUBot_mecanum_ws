#!/usr/bin/env python3
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import os

class TakePhoto:
    def __init__(self, img_topic, image_dir):
        self.bridge = CvBridge()
        self.image_received = False
        self.cv_image = None
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)
        self.image_dir = image_dir
        self.image_count = 1

        if not os.path.exists(self.image_dir):
            os.makedirs(self.image_dir)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv.putText(self.cv_image, f"Image {self.image_count}", (100, 290),
                       cv.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 255), 1)
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(e)

    def save_picture(self):
        if self.image_received:
            img_title = os.path.join(self.image_dir, f"Foto_{self.image_count}.jpg")
            cv.imwrite(img_title, self.cv_image)
            rospy.loginfo("Saved image " + img_title)
            self.image_count += 1
        else:
            rospy.loginfo("No images received")

if __name__ == '__main__':
    rospy.init_node('take_photo', anonymous=False)

    img_topic = rospy.get_param('~image_topic', '/usb_cam/image_raw')
    image_dir = rospy.get_param('~image_dir', '/home/user/rUBot_mecanum_ws/src/rubot_projects/photos')

    camera = TakePhoto(img_topic, image_dir)

    rate = rospy.Rate(1)  # 1 Hz = una foto por segundo
    while not rospy.is_shutdown():
        camera.save_picture()
        rate.sleep()
