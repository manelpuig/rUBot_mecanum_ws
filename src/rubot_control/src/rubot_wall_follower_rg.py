#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

import math


pub_ = None
regions_ = {
    'front': 0,
    'fright': 0,    
    'right': 0,
    'bright':0,
}
state_ = 0
state_dict_ = {
    0: 'find the wall',
    1: 'front wall',
    2: 'fright wall',
    3: 'follow wall',
    4: 'bright corner',
}

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 1
angleClosestDistance = 0
ClosestDistance = 0
d = 0.3

def clbk_laser(msg):
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    global angleClosestDistance
    global ClosestDistance
    
    if not isScanRangesLengthCorrectionFactorCalculated:
            scanRangesLengthCorrectionFactor = int(len(msg.ranges) / 360)
            isScanRangesLengthCorrectionFactorCalculated = True
            
    bright_min = 60 * scanRangesLengthCorrectionFactor
    bright_max = 80 * scanRangesLengthCorrectionFactor
    right_min = 80 * scanRangesLengthCorrectionFactor
    right_max = 120 * scanRangesLengthCorrectionFactor
    fright_min = 120 * scanRangesLengthCorrectionFactor
    fright_max = 150 * scanRangesLengthCorrectionFactor
    front_min= 150 * scanRangesLengthCorrectionFactor
    front_max = 210 * scanRangesLengthCorrectionFactor
    
    closestDistance, elementIndex = min((val, idx) for (idx, val) in enumerate(msg.ranges) if msg.range_min < val < msg.range_max)
    angleClosestDistance = (elementIndex / scanRangesLengthCorrectionFactor)
            
    global regions_
    regions_ = {
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
        'fright':  min(min(msg.ranges[fright_min:fright_max]), 3),
        'right':   min(min(msg.ranges[right_min:right_max]), 3),
        'bright':   min(min(msg.ranges[bright_min:bright_max]), 3),
    }
    #print ("front distance: "+ str(regions_["front"]))
    #print ("front-right distance: "+ str(regions_["fright"]))
    #print ("right distance: "+ str(regions_["right"]))
    #print ("back-right distance: "+ str(regions_["bright"]))

    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - new state [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    global d
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    #d = 0.3

    if regions['front'] > d and regions['fright'] > d and regions['right'] > (d+0.4) and regions['bright'] > d:
        state_description = 'case 0 - nothing'
        change_state(0)
    elif regions['front'] < d:
        state_description = 'case 1 - front'
        change_state(1)
    elif regions['fright'] < d and regions['front'] > d:
        state_description = 'case 2 - fright'
        change_state(2)
    elif regions['right'] < d and regions['fright'] > d and regions['front'] > d:
        state_description = 'case 3 - right'
        change_state(3)
    elif regions['bright'] < (d+0.0) and regions['right'] > d and regions['fright'] > d and regions['front'] > d:
        state_description = 'case 4 - bright'
        change_state(4)
    else:
        state_description = 'unknown case'
        rospy.loginfo('unknown case')


def find_wall():# 'case 0 - nothing'
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.05
    return msg

def front_wall():# 'case 1 - front'
    msg = Twist()
    msg.angular.z = 0.2
    return msg
    
def fright_wall():# 'case 2 - fright'
    msg = Twist()
    msg.angular.z = 0.2
    return msg

def follow_wall():# 'case 3 - right'
    global regions_
    global d
    global angleClosestDistance
    global ClosestDistance
    msg = Twist()
    msg.linear.x = 0.1
    msg.angular.z = 0.3 * ((d-0.05) - regions_["right"])
    #msg.angular.z = 0.3 * ((d-0.05) - ClosestDistance)
    print ("Distance: " + str(regions_["right"]))
    #msg.angular.z = 0.05 * (angleClosestDistance -90)
    #print ("angle minim: " + str(angleClosestDistance))
    return msg

def bright_corner():# 'case 4 - bright'
    global d
    msg = Twist()
    msg.linear.x = 0.5*0.3
    msg.angular.z = -1
    return msg

def main():
    global pub_
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor

    rospy.init_node('wall_follower')
    pub_ = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    
    rate = rospy.Rate(20)
    while not rospy.is_shutdown():
        msg = Twist()
        if state_ == 0:
            msg = find_wall()
        elif state_ == 1:
            msg = front_wall()
        elif state_ == 2:
            msg = fright_wall()
        elif state_ == 3:
            msg = follow_wall()
        elif state_ == 4:
            msg = bright_corner()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
