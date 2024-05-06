#!/usr/bin/env python3
import rospy
import cv2
import numpy as np


def signal_detected(photo):
    img = cv2.imread(photo)
    # Convert the image to grayscale
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise and help with contour detection
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny edge detection
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours and hierarchy
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    #print("Contours:\n"+str(contours))

    # Initialize variables to keep track of the contour with the highest hierarchy_info[3] value
    max_parent_index = -1
    max_hierarchy_value = -1


    # Iterate through contours and hierarchy
    for i in range(len(contours)):
        #print (f"Contour: {i}")
        hierarchy_info = hierarchy[0][i]
        cv2.drawContours(img, contours[i], -1, (0,255,0), 1)
        cv2.imshow('imagen',img)
        #cv2.waitKey(0)
        
        # Check if the contour has a parent
        if hierarchy_info[3] != -1:
            # Update the variables if the current contour has a higher hierarchy_info[3] value
            if hierarchy_info[3] > max_hierarchy_value:
                max_hierarchy_value = hierarchy_info[3]
                max_parent_index = i#index starts with 0

    #print ('hierarchy=\n',hierarchy)
    #print (f"Index: {max_parent_index} Value: {max_hierarchy_value}")
    #cv2.waitKey(0)

    # Access the first child contour using hierarchy_info[max_hierarchy_value]
    child_contour = contours[max_parent_index]
    parent_contour = contours[max_parent_index-2]# there are 2 contours in arrow (inner and outer)
    # Calculate the moments for the child contour
    moments = cv2.moments(child_contour)


    # Calculate the centroid coordinates
    if moments['m00'] != 0:
        cx = int(moments['m10'] / moments['m00'])
        cy = int(moments['m01'] / moments['m00'])

        # Draw the original contour in Blue
        img_child=cv2.drawContours(img, [child_contour], -1, (255, 0, 0), 1)
        # Draw the centroid as a blue dot
        cv2.circle(img_child, (cx, cy), 2, (255, 0, 0), -1)
        #print(f"Child Contour: ({cx} , {cy})")
        # Print the area of the first child contour
        area = cv2.contourArea(child_contour)
        #print(f"Area of child Contour {i+1}: {area}")
        cv2.imshow('Child', img_child)
        #cv2.waitKey(0)
        
    # Calculate the moments for the Parent contour
    moments = cv2.moments(parent_contour)

    # Calculate the centroid coordinates
    if moments['m00'] != 0:
        px = int(moments['m10'] / moments['m00'])
        py = int(moments['m01'] / moments['m00'])

        # Draw the first parent contour in red
        img_parent=cv2.drawContours(img, [parent_contour], -1, (0, 0, 255), 1)
        # Draw the centroid as a red dot
        cv2.circle(img_parent, (px, py), 2, (0, 0, 255), -1)
        #print(f"Parent Contour: ({px} , {py})")
        cv2.imshow('Parent', img_parent)
        #cv2.waitKey(0)


    # Print the detection
    dif=cx-px
           
    # Display the image with contours and centroids
    cv2.imshow('Contours with Centroids', cv2.resize(img, (800, 600)))
    #cv2.waitKey(0)
    #cv2.destroyAllWindows()
    rospy.loginfo("Difference " + str(dif))
    
    if dif > 0:
        signal = "right"
    else:
        signal = "left"
    rospy.loginfo("Centroids " + str(dif))
    return signal

if __name__ == '__main__':
    # Read signal
    #image = cv2.imread('left.png')
    photo = "left.png"
    signal = signal_detected(photo)
    print("Signal detected: ", signal)
    #cv2.imshow('Signal',image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
