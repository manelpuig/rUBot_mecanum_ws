#!/usr/bin/env python3

import cv2
import numpy as np

# terminal in the png folder
# yelow line detection RGB=(255,255,0) or BGR=(0,255,255)
frame = cv2.imread("road_view1.png", cv2.IMREAD_COLOR)
cv2.imshow("Road init frame", frame)
height, width, channels = frame.shape
print("shape frame: width {1} height {0}".format(height,width))
frame2 = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)#half resolution
height, width, channels = frame2.shape
print("shape frame2: width {1:.0f} height {0:.0f}".format(height,width))
# Convert BGR to HSV. Yelow HSV = [30, 255, 255]
hsv = cv2.cvtColor(frame2, cv2.COLOR_BGR2HSV)
cv2.imshow("hsv", hsv)
# Define range of yelow color in HSV
# Take red H range: fom 27 to 33 
# Take S range: from 100 to 255 (for white from 0)
# Tahe V range: from 20 to 255 (for white from 0)
lower_color = np.array([27,100,20])
upper_color = np.array([33,255,255])
# Threshold the HSV image to get only yelow color zone in B&W image
mask = cv2.inRange(hsv, lower_color, upper_color)
# Bitwise-AND mask and original image to obtain the image with only yelow regions
res = cv2.bitwise_and(frame2,frame2, mask= mask)
cv2.imshow('Road low resolution',frame2)
cv2.imshow('mask',mask)
cv2.imshow('res',res)
# Find Contours
(contours, _) = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
print("Number of centroids==>" + str(len(contours)))
cv2.drawContours(frame2,contours,-1,(0,255,0),3)
cv2.imshow('Contour',frame2)
# Find Centroids
M = cv2.moments(contours[0])
cx = int(M['m10']/M['m00'])
cy = int(M['m01']/M['m00'])
cv2.circle(res, (int(cx), int(cy)), 5, (0, 255, 0), -1)
cv2.imshow("Centroid", res)
print("Centroid: ({0},{1})".format(cx,cy))         
# Wait until x miliseconds or until you close all windows (0)
cv2.waitKey(0)
#cv2.destroyAllWindows()
