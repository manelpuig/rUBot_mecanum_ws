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
        rospy.init_node('line_following_traffic', anonymous=True)
        self.camera_sub = rospy.Subscriber('/rubot/camera1/image_raw',Image, self.camera_cb)
        self.lidar_sub = rospy.Subscriber("/scan", LaserScan, self.lidar_cb)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        rospy.on_shutdown(self.shutdown)
        self.vel_msg=Twist()
        self.bridge=CvBridge()
        self._r = rospy.Rate(5)
        self.traffic_sign = False
        self.turn_time = 0

    def camera_cb(self, data):
        frame = self.bridge.imgmsg_to_cv2(data,desired_encoding="bgr8")
        #print(frame.shape)# 240*320
        sign_dim = (60,40)# first is X and second Y
        frame_sign = frame[30:70,260:320] # First is Y and sencond is X

        left1 = cv2.imread('/home/ubuntu/rUBot_mecanum_ws/src/rubot_projects/photos/Traffic_Signs/left.png')
        left2 = cv2.resize(left1,sign_dim)
        right1 = cv2.imread('/home/ubuntu/rUBot_mecanum_ws/src/rubot_projects/photos/Traffic_Signs/right.png')
        right2 = cv2.resize(right1,sign_dim)
        stop1 = cv2.imread('/home/ubuntu/rUBot_mecanum_ws/src/rubot_projects/photos/Traffic_Signs/stop.png')
        stop2 = cv2.resize(stop1,sign_dim)

        edged = cv2.Canny(frame ,60,100 )
        
        white_index=[]
        mid_point_line = 0
        for index,values in enumerate(edged[:][200]):# [172] index 0 on top
            if(values == 255):
                white_index.append(index)
                
        #print("White: ",white_index)

        if(len(white_index) >= 2 ):#== some times more than 2 white points 
            cv2.circle(img=edged, center = (white_index[0],200), radius = 2 , color = (255,0,0), thickness=1)
            cv2.circle(img=edged, center = (white_index[1],200), radius = 2 , color = (255,0,0), thickness=1)
            mid_point_line = int ( (white_index[0] + white_index[1]) /2 )
            cv2.circle(img=edged, center = (mid_point_line,200), radius = 3 , color = (255,0,0), thickness=2)

        mid_point_robot = [160,200]#[135,172]
        cv2.circle(img=edged, center = (mid_point_robot[0],mid_point_robot[1]), radius = 5 , color = (255,0,0), thickness=2)
        error = mid_point_robot[0] - mid_point_line
        #print("Error -> " , error)
        
        if (self.traffic_sign == False):
            print("Follow right line")
            self.turn_time=0
            if (error < 0):
                self.vel_msg.angular.z = -0.2#0.4
                self.vel_msg.linear.x = 0.1#0.4
            else:
                self.vel_msg.angular.z = 0.2
                self.vel_msg.linear.x = 0.1#0.4
        else:
            print("Reading Traffic sign")
            
            match_right = traffic_line_following.mse(right2,frame_sign)
            match_left = traffic_line_following.mse(left2,frame_sign)
            match_stop = traffic_line_following.mse(stop2,frame_sign)
            matching = np.array([match_right,match_left,match_stop])
            min_match = matching.min()
            #arg_min_match = matching.argmin()
            arg_min_match=1#turn left
            print("Traffic sign detected: "+str(matching))
            
            if arg_min_match==0:
                print("Traffic sign detected: Turn right, with error: "+str(min_match))
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = -0.0
                self.turn_time=0
            elif arg_min_match==1:
                print("Traffic sign detected: Turn left, with error: "+str(min_match))
                self.vel_msg.linear.x = 0.1
                self.vel_msg.angular.z = 0.3
                self.turn_time=2#wait 2 secionds
            elif arg_min_match==2:
                print("Traffic sign detected: Stop, with error: "+str(min_match))
                self.vel_msg.linear.x = 0.0
                self.vel_msg.angular.z = -0.0
                self.turn_time=0

       
        cv2.imshow('Frame',frame)
        cv2.imshow('Traffic Sign',frame_sign)
        #cv2.imshow('left1',left1)
        #cv2.imshow('left2',left2)
        cv2.imshow('Canny Output',edged)
        cv2.waitKey(1)

    def lidar_cb(self, scan):
        closestDistance, elementIndex = min((val, idx) for (idx, val) in enumerate(scan.ranges))
        angleClosestDistance = (elementIndex / 2)
        rospy.loginfo("Lidar readings...")
        print("Distance: "+str(closestDistance)+" Angle: "+str(angleClosestDistance))
        if closestDistance < 0.51 and 90 < angleClosestDistance < 180:# front right
            rospy.loginfo("Traffic sign detected!")
            self.traffic_sign=True
        else:
        	self.traffic_sign=False

    def mse(self,img1, img2):
        h, w, c = img1.shape
        diff = cv2.subtract(img1, img2)
        err = np.sum(diff**2)
        mse = err/(float(h*w))
        return mse
     
    def start(self):
        while not rospy.is_shutdown():
            self.cmd_vel_pub.publish(self.vel_msg)
            self._r.sleep()
            time.sleep(self.turn_time)#time to turn
        rospy.spin()

    def shutdown(self):
        self.vel_msg.linear.x = 0
        self.vel_msg.linear.y = 0
        self.vel_msg.angular.z = 0
        self.cmd_vel_pub.publish(self.vel_msg)
        rospy.loginfo("Stopping")

    
if __name__ == '__main__':
    try:
        traffic_line_following = traffic_line()
        traffic_line_following.start()

    except rospy.ROSInterruptException:
        traffic_line_following.shutdown()
        print ("Ending Traffic line Following")
