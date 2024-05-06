#!/usr/bin/env python3
import rospy
import cv2 as cv
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class TakePhoto:
    def __init__(self, img_topic, image_title):
        self.bridge = CvBridge()
        self.image_received = False
        self.cv_image = None

        # Connect image topic
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

    def callback(self, data):
        try:
            self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv.putText(self.cv_image, "Image 1", (100, 290), cv.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 255), 1)
            self.image_received = True
        except CvBridgeError as e:
            rospy.logerr(e)

    def save_picture(self, img_title):
        if self.image_received:
            cv.imwrite(img_title, self.cv_image)
            rospy.loginfo("Saved image " + img_title)
        else:
            rospy.loginfo("No images received")

if __name__ == '__main__':
    # Initialize
    rospy.init_node('take_photo', anonymous=False)

    # Get parameters from launch file
    img_topic = rospy.get_param('~image_topic', '/rubot/camera1/image_raw')
    img_title = rospy.get_param('~image_title', './src/rubot_projects/photos/photo3_sw.jpg')

    # Create TakePhoto instance
    camera = TakePhoto(img_topic, img_title)
    # Allow up to one second for connection
    rospy.sleep(1)
    camera.save_picture(img_title)
    # Keep the node running
    rospy.spin()

