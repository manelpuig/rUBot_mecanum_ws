#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations
from std_msgs.msg import Bool

import math

pub_ = None
regions_ = {
    'right': 0,
    'fright': 0,
    'front': 0,
    'fleft': 0,
    'left': 0,
    'bright':0,
}
state_ = 0
find=0 
distance=0
s_factor=0
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'follow corner',
    4: 'left_correction',
    5: 'right_correction',
    6: 'stop',
}

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 1



def clbk_laser(msg):
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
            scanRangesLengthCorrectionFactor = int(len(msg.ranges) / 360)
            isScanRangesLengthCorrectionFactorCalculated = True
            
    #rospy.loginfo("Scan Ranges correction %5.2f we have,%5.2f points.",scanRangesLengthCorrectionFactor, len(msg.ranges))
        
            
    bright_min = 60 * scanRangesLengthCorrectionFactor
    bright_max = 88 * scanRangesLengthCorrectionFactor
    right_min = 88 * scanRangesLengthCorrectionFactor
    right_max = 92 * scanRangesLengthCorrectionFactor
    fright_min = 92 * scanRangesLengthCorrectionFactor
    fright_max = 120 * scanRangesLengthCorrectionFactor
    front_min= 150 * scanRangesLengthCorrectionFactor
    front_max = 210 * scanRangesLengthCorrectionFactor
            
    global regions_
    regions_ = {
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
        'fright':  min(min(msg.ranges[fright_min:fright_max]), 3),
        'right':   min(min(msg.ranges[right_min:right_max]), 3),
        'bright':   min(min(msg.ranges[bright_min:bright_max]), 3),
    }
    print ("front distance: "+ str(regions_["front"]))
    #print ("front-right distance: "+ str(regions_["fright"]))
    print ("right distance: "+ str(regions_["right"]))
    #print ("back-right distance: "+ str(regions_["bright"]))

    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s  \n \n' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    global find
    global distance
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''
    d=distance
   	
    #d = 0.6
	# Buscando la pared
    if regions['front'] > d and regions['fright'] > (d) and regions['bright'] > d and find==0:
        state_description = 'case 0 - find wall'
        change_state(0)
        
	# Pared encontrada
    elif regions['front'] < d and find==0:
        state_description = 'case 1 - wall found'
        find=1
        change_state(1)
        
	# Girando    	
    elif regions['right'] > (d) and regions['fright'] > (d-0.2) and find ==1:
        state_description = 'case 2 - turn left '
        change_state(1)
    elif regions['right'] < (d) and find==1:
        state_description = 'case 3 - start following'
        find=2
        change_state(2)
        
	# Siguiendo la pared
    elif regions['front'] < (d) and find==2:
        state_description = 'case 4 - following-turn left objeto en front'
        change_state(1)    
    elif regions['fright'] < (d/2) and find==2:
        state_description = 'case 5 - following-correccion turn left'
        change_state(4)
    elif regions['right'] > (d/1.1) and find==2:
        state_description = 'case 6 - following- correccion turn right'
        change_state(5) 
           
    elif regions['right'] < (d) and find==2:
        state_description = 'case 7 - following- straing on '
        change_state(2)
          
    else:
        state_description = 'stop case'
        change_state(6)
        rospy.loginfo('stop case')


def find_wall():

    global s_factor
    msg = Twist()
    msg.linear.x = (0.2)
    msg.angular.z = 0
    return msg

def turn_left():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0.3
    return msg

def left_correction():
    msg = Twist()    
    msg.linear.x = 0.2
    msg.angular.z = 0.4  #0.2
    return msg
    
def right_correction():
    msg = Twist()    
    msg.linear.x = 0.2
    msg.angular.z = -0.4  #-0.2
    return msg	

def follow_the_wall():
    global regions_
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0
    return msg

def stop():
    msg = Twist()
    msg.linear.x = 0
    msg.angular.z = 0
    return msg

def main():
    global pub_
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    global distance
    global s_factor
    #rostopic pub /rest_odom std_msgs/Bool "data: True"
    

    
    rospy.init_node('wall_follower')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    z=Bool()
    reset=rospy.Publisher('/rest_odom', Bool, queue_size=1)
    z.data=True
    reset.publish(z)
        
    distance= rospy.get_param("~distance_laser")
    s_factor = rospy.get_param("~speed_factor")
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
    
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        elif state_== 4:
            msg = left_correction()
        elif state_== 5:
            msg = right_correction()            
        elif state_== 6:
            msg = stop()
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)
        rate.sleep()
        

    change_state(6)
    pub_.publish(msg) 
    rate.sleep()    
            


if __name__ == '__main__':
    main()
