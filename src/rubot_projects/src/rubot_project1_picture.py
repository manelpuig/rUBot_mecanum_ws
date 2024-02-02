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

        # Connect image topic
        self.image_sub = rospy.Subscriber(img_topic, Image, self.callback)

        # Allow up to one second for connection
        rospy.sleep(1)

        # Take a photo
        if self.take_picture(image_title):
            rospy.loginfo("Saved image " + image_title)
        else:
            rospy.loginfo("No images received")

    def callback(self, data):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
            cv.putText(cv_image, "Image 1", (100, 290), cv.FONT_HERSHEY_TRIPLEX, 1, (0, 255, 255), 1)
        except CvBridgeError as e:
            print(e)

        self.image_received = True
        self.image = cv_image

    def take_picture(self, img_title):
        if self.image_received:
            cv.imwrite(img_title, self.image)
            return True
        else:
            return False

if __name__ == '__main__':
    # Initialize
    rospy.init_node('take_photo', anonymous=False)

    # Get parameters from launch file
    img_topic = rospy.get_param('~image_topic', '/rubot/camera1/image_raw')
    img_title = rospy.get_param('~image_title', './src/rubot_projects/photos/photo3_sw.jpg')

    # Create TakePhoto instance
    camera = TakePhoto(img_topic, img_title)

   # Keep the node running
    rospy.spin()
