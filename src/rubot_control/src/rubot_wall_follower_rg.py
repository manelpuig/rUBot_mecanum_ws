#! /usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf import transformations

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
state_dict_ = {
    0: 'find the wall',
    1: 'turn left',
    2: 'follow the wall',
    3: 'follow corner',
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
            
    bright_min = 60 * scanRangesLengthCorrectionFactor
    bright_max = 88 * scanRangesLengthCorrectionFactor
    right_min = 88 * scanRangesLengthCorrectionFactor
    right_max = 92 * scanRangesLengthCorrectionFactor
    fright_min = 92 * scanRangesLengthCorrectionFactor
    fright_max = 120 * scanRangesLengthCorrectionFactor
    front_min= 120 * scanRangesLengthCorrectionFactor
    front_max = 240 * scanRangesLengthCorrectionFactor
            
    global regions_
    regions_ = {
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
        'fright':  min(min(msg.ranges[fright_min:fright_max]), 3),
        'right':   min(min(msg.ranges[right_min:right_max]), 3),
        'bright':   min(min(msg.ranges[bright_min:bright_max]), 3),
    }
    print ("front distance: "+ str(regions_["front"]))
    print ("front-right distance: "+ str(regions_["fright"]))
    print ("right distance: "+ str(regions_["right"]))
    print ("back-right distance: "+ str(regions_["bright"]))

    take_action()


def change_state(state):
    global state_, state_dict_
    if state is not state_:
        print ('Wall follower - [%s] - %s' % (state, state_dict_[state]))
        state_ = state


def take_action():
    global regions_
    regions = regions_
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    d = 0.3

    if regions['front'] > d and regions['fright'] > (d+0.6) and regions['bright'] > d:
        state_description = 'case 1 - nothing'
        change_state(0)
    elif regions['front'] < d and regions['fright'] > d:
        state_description = 'case 2 - front'
        change_state(1)
    elif regions['front'] < d and regions['fright'] < d:
        state_description = 'case 2 - front & fright'
        change_state(1)
    elif regions['front'] > d and regions['fright'] < d:
        state_description = 'case 3 - fright'
        change_state(2)
    elif regions['front'] > d and regions['right'] < d:
        state_description = 'case 4 - right'
        change_state(2)
    elif regions['front'] > d and regions['bright'] < d:
        state_description = 'case 5 - bright'
        change_state(3)
    elif regions['front'] > d and regions['right'] > d:
        state_description = 'case 5 - right too far'
        change_state(3)    
    elif regions['front'] > d and regions['bright'] > d and regions['fright'] < (d+0.6):
        state_description = 'case 5 - close fright'
        change_state(3)
    else:
        state_description = 'unknown case'
        rospy.loginfo('unknown case')


def find_wall():
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.01
    return msg


def turn_left():
    msg = Twist()
    msg.angular.z = 0.2
    return msg


def follow_the_wall():
    global regions_
    msg = Twist()
    msg.linear.x = 0.2
    msg.angular.z = 0.05
    return msg

def follow_corner():
    msg = Twist()
    msg.linear.x = 0.1
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
            msg = turn_left()
        elif state_ == 2:
            msg = follow_the_wall()
        elif state_ == 3:
            msg = follow_corner()
            pass
        else:
            rospy.logerr('Unknown state!')

        pub_.publish(msg)

        rate.sleep()


if __name__ == '__main__':
    main()
