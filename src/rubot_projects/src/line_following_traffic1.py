#!/usr/bin/env python3
import rospy
import sys
import time
import numpy as np
from sensor_msgs.msg import Image,LaserScan
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2

class traffic_line:

    def __init__(self):
        rospy.init_node('following_traffic_line', anonymous=True)
        self.camera_sub = rospy.Subscriber('/rubot/camera1/image_raw',Image, self.camera_cb)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.on_shutdown(self.shutdown)
        self.vel_msg=Twist()
        self.bridge=CvBridge()
        self._r = rospy.Rate(5)

    def camera_cb(self, data):
        frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        #frame = frame[290:479,130:400] # First is Y and sencond is X
        edged = cv2.Canny(frame ,60,100 )
        
        white_index=[]
        mid_point_line = 0
        for index,values in enumerate(edged[:][200]):# [172] index 0 on top
            if(values == 255):
                white_index.append(index)
                
        print("White: ",white_index)

        if(len(white_index) >= 2 ):#== some times more than 2 white points 
            cv2.circle(img=edged, center = (white_index[0],200), radius = 2 , color = (255,0,0), thickness=1)
            cv2.circle(img=edged, center = (white_index[1],200), radius = 2 , color = (255,0,0), thickness=1)
            mid_point_line = int ( (white_index[0] + white_index[1]) /2 )
            cv2.circle(img=edged, center = (mid_point_line,200), radius = 3 , color = (255,0,0), thickness=2)

        mid_point_robot = [160,200]#[135,172]
        cv2.circle(img=edged, center = (mid_point_robot[0],mid_point_robot[1]), radius = 5 , color = (255,0,0), thickness=2)
        error = mid_point_robot[0] - mid_point_line
        print("Error -> " , error)

        if ( error < 0):
            self.vel_msg.angular.z = -0.2#0.4
        else:
            self.vel_msg.angular.z = 0.2

        self.vel_msg.linear.x = 0.1#0.4

        self.cmd_vel_pub.publish(self.vel_msg)


        cv2.imshow('Frame',frame)
        cv2.imshow('Canny Output',edged)
        cv2.waitKey(1)

    def lidar_cb(self, scan):
        closestDistance, elementIndex = min((val, idx) for (idx, val) in enumerate(scan.ranges) if scan.range_min < val < scan.range_max)
        angleClosestDistance = (elementIndex / 2)
        if closestDistance < 0.2 and 90 < angleClosestDistance < 180:# front right
            self.vel_msg.linear.x = 0.1
            self.vel_msg.angular.z = -0.2

    def start(self):
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.vel_msg)
            self._r.sleep()
        rospy.spin()

    def shutdown(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.vel_msg)
        rospy.loginfo("Stop RVIZ")

    
if __name__ == '__main__':
    try:
        traffic_line_following = traffic_line()
        traffic_line_following.start()

    except rospy.ROSInterruptException:
        traffic_line_following.shutdown()
        print ("Ending Traffic line Following")