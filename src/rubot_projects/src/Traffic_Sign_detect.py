import time
import numpy as np
import cv2

class Dettection:
    def __init__(self):
        self.traffic_sign = False
        
    def mse(self,img1, img2):
        h, w, c = img1.shape
        diff = cv2.subtract(img1, img2)
        err = np.sum(diff**2)
        mse = err/(float(h*w))
        return mse
     
    def start(self):
        sign_dim = (187,197)# usually (320,240) first is X and second Y
        path1 = "C:/Users/puigm/Desktop/ROS_github/rUBot_mecanum_ws/src/rubot_projects/photos/Traffic_Signs/"
        path2 = "C:/Users/HP/OneDrive/Escritorio/ROS/rUBot_mecanum_ws/src/rubot_projects/photos/Traffic_Signs/"
        path3 = "C:/Users/HP/OneDrive/Escritorio/ROS/rUBot_mecanum_ws/src/rubot_projects/photos/"
        path4 = "/home/ubuntu/rUBot_mecanum_ws/src/rubot_projects/photos/Traffic_Signs/"
        left1 = cv2.imread(path2+'left.png')
        left2 = cv2.resize(left1,sign_dim)
        #print(left1.shape) # (Y,X,channels)
        right1 = cv2.imread(path2+'right.png')
        right2 = cv2.resize(right1,sign_dim)
        stop1 = cv2.imread(path2+'stop.png')
        stop2 = cv2.resize(stop1,sign_dim)
        s90_1 = cv2.imread(path2+'90.png')
        s90_2 = cv2.resize(s90_1,sign_dim)
        sign1 = cv2.imread(path2+'signL1.png')
        sign2 = cv2.resize(sign1,sign_dim)
        
        #frame_sign = sign2
        frame_sign = cv2.imread(path3+'image5_55.png')
        print(frame_sign.shape)
        frame_sign2 = frame_sign[20:90,220:300] # First is Y and sencond is X
        print(frame_sign2.shape)
        frame_sign = cv2.resize(frame_sign2,sign_dim)

        """ cv2.imshow('Traffic Sign',frame_sign)
        cv2.imshow('Traffic Sign2',frame_sign2)
        cv2.imshow('Traffic Sign3',frame_sign3)
        cv2.waitKey(0) # waits until a key is pressed
        cv2.destroyAllWindows() # destroys the window showing image """
        
        match_right = traffic_sign_dettection.mse(right2,frame_sign)
        match_left = traffic_sign_dettection.mse(left2,frame_sign)
        match_stop = traffic_sign_dettection.mse(stop2,frame_sign)
        matching = np.array([match_right,match_left,match_stop])
        min_match = matching.min()
        arg_min_match = matching.argmin()
        print("Traffic sign detected: "+str(matching))
        
        if arg_min_match==0:
            print("Traffic sign detected: Turn right, with error: "+str(min_match))
            
        elif arg_min_match==1:
            print("Traffic sign detected: Turn left, with error: "+str(min_match))
            
        elif arg_min_match==2:
            print("Traffic sign detected: Stop, with error: "+str(min_match))
            
        cv2.imshow('Traffic Sign',frame_sign)
        cv2.imshow('Traffic Sign3',frame_sign2)
        cv2.imshow('Sign',left1)
        cv2.waitKey(0) # waits until a key is pressed
        cv2.destroyAllWindows() # destroys the window showing image
        
if __name__ == '__main__':
    try:
        traffic_sign_dettection = Dettection()
        traffic_sign_dettection.start()

    except:
        print("Closed")