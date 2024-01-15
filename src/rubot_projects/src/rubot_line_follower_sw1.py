#!/usr/bin/env python3
import rospy
import cv2
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge

class LineFollower:

    def __init__(self):
        self.bridge = CvBridge()
        self.image_sub = rospy.Subscriber('/rubot/camera1/image_raw',
                                          Image, self.image_callback)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel',
                                          Twist, queue_size=1)
        self.vel_msg = Twist()

        # Constants
        self.MID_POINT_ROBOT = [135, 172]
        self.CANNY_THRESHOLD_LOW = 10
        self.CANNY_THRESHOLD_HIGH = 100

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        frame_roi = frame[290:479, 130:400]  # Define Region of Interest

        edged = cv2.Canny(frame_roi, self.CANNY_THRESHOLD_LOW, self.CANNY_THRESHOLD_HIGH)

        cv2.imshow('Original Frame', frame)
        #cv2.imshow('Canny Output', edged)
        cv2.waitKey(1)

        if edged is None:
            rospy.logwarn("Failed to perform Canny edge detection.")
            return

        white_index = [index for index, value in enumerate(edged[172]) if value == 255]
        print(white_index)

        mid_point_line = self.calculate_mid_point(white_index)
        error = self.calculate_error(mid_point_line)
        print("Error ->", error)

        self.publish_velocity(error)

        cv2.circle(img=edged, center=(self.MID_POINT_ROBOT[0], self.MID_POINT_ROBOT[1]),
                   radius=5, color=(255, 0, 0), thickness=2)

        cv2.imshow('Frame', frame)
        cv2.imshow('Canny Output', edged)
        cv2.waitKey(1)

    def calculate_mid_point(self, white_index):
        if len(white_index) == 2:
            mid_point_line = int(sum(white_index) / len(white_index))
            cv2.circle(img=edged, center=(white_index[0], 172), radius=2, color=(255, 0, 0), thickness=1)
            cv2.circle(img=edged, center=(white_index[1], 172), radius=2, color=(255, 0, 0), thickness=1)
            cv2.circle(img=edged, center=(mid_point_line, 172), radius=3, color=(255, 0, 0), thickness=2)
            return mid_point_line
        else:
            return 0  # Handle the case when white_index does not have 2 elements

    def calculate_error(self, mid_point_line):
        return self.MID_POINT_ROBOT[0] - mid_point_line if mid_point_line != 0 else 0

    def publish_velocity(self, error):
        self.vel_msg.angular.z = -0.5 if error < 0 else 0.5
        self.vel_msg.linear.x = 0.4
        self.cmd_vel_pub.publish(self.vel_msg)

if __name__ == '__main__':
    rospy.init_node('line_follower')
    follower = LineFollower()
    rospy.spin()
