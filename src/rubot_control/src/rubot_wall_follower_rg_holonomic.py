#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import time

pub = None
d = 0
vx = 0
wz = 0
vf = 0

isScanRangesLengthCorrectionFactorCalculated = False
scanRangesLengthCorrectionFactor = 2


def clbk_laser(msg):
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
            scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            isScanRangesLengthCorrectionFactorCalculated = True
    back_min = int(335 * scanRangesLengthCorrectionFactor)
    back_cut_min = int(360 * scanRangesLengthCorrectionFactor - 1)
    back_cut_max = int(0 * scanRangesLengthCorrectionFactor)
    back_max = int(25 * scanRangesLengthCorrectionFactor)
    bright_min = int(25 * scanRangesLengthCorrectionFactor)
    bright_max = int(65 * scanRangesLengthCorrectionFactor)
    right_min = int(65 * scanRangesLengthCorrectionFactor)
    right_max = int(115 * scanRangesLengthCorrectionFactor)
    fright_min = int(115 * scanRangesLengthCorrectionFactor)
    fright_max = int(155 * scanRangesLengthCorrectionFactor)
    front_min= int(155 * scanRangesLengthCorrectionFactor)
    front_max = int(205 * scanRangesLengthCorrectionFactor)
    fleft_min = int(205 * scanRangesLengthCorrectionFactor)
    fleft_max = int(245 * scanRangesLengthCorrectionFactor)
    left_min = int(245 * scanRangesLengthCorrectionFactor)
    left_max = int(295 * scanRangesLengthCorrectionFactor)
    bleft_min = int(295 * scanRangesLengthCorrectionFactor)
    bleft_max = int(335 * scanRangesLengthCorrectionFactor)

    regions = {
        'back': min(min(msg.ranges[back_min:back_cut_min]),min(msg.ranges[back_cut_max:back_max]), 3),
        'left':  min(min(msg.ranges[left_min:left_max]), 3),
        'bleft':  min(min(msg.ranges[bleft_min:bleft_max]), 3),
        'fleft':  min(min(msg.ranges[fleft_min:fleft_max]), 3),
        'right':  min(min(msg.ranges[right_min:right_max]), 3),
        'bright':  min(min(msg.ranges[bright_min:bright_max]), 3),
        'fright':  min(min(msg.ranges[fright_min:fright_max]), 3),
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
    }

    take_action(regions)


def take_action(regions):
    msg = Twist()
    linear_x = 0
    linear_y = 0
    angular_z = 0
    aux = 1.5

    state_description = ''

    if regions['front'] > d and regions['left'] > d and regions['bleft'] > d*aux and regions['fleft'] > d*aux and regions['right'] > d  and regions['bright'] > d*aux  and regions['fright'] > d*aux  and regions['back'] > d:
        state_description = 'case 1 - nothing'
        linear_x = vx
        linear_y = 0
        angular_z = 0
    elif regions['front'] < d and regions['left'] > d:
        state_description = 'case 2 - front'
        linear_x = 0
        linear_y = vx
        angular_z = 0

    elif regions['left'] < d and regions['back'] > d:
        state_description = 'case 3 - left'
        linear_x = -vx
        linear_y = 0
        angular_z = 0
        

    elif regions['back'] < d  and regions['right'] > d:
        state_description = 'case 4 - back'
        linear_x = 0
        linear_y = -vx
        angular_z = 0
    
    elif regions['right'] < d and regions['front'] > d:
        state_description = 'case 5 - right'
        linear_x = vx
        linear_y = 0
        angular_z = 0

    elif regions['fleft'] < d*aux:
        state_description = 'case 6 - fleft 2'
        linear_x = 0
        linear_y = vx*0.5
        angular_z = 0

    elif regions['bleft'] < d*aux:
        state_description = 'case 7 - bleft 2'
        linear_x = -vx*0.5
        linear_y = 0
        angular_z = 0

    elif regions['bright'] < d*aux:
        state_description = 'case 8 - brigth 2'
        linear_x = 0
        linear_y = -vx*0.5
        angular_z = 0

    elif regions['fright'] < d*aux:
        state_description = 'case 9 - rfigth 2'
        linear_x = vx*0.5
        linear_y = 0
        angular_z = 0

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
    msg.linear.y = linear_y
    msg.angular.z = angular_z
    pub.publish(msg)
    rate.sleep()

def shutdown():
    msg = Twist()
    msg.linear.x = 0
    msg.linear.y = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Stop rUBot")

def main():
    global pub
    global sub
    global rate
    global d
    global vx
    global wz
    global vf

    rospy.init_node('wall_follower')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    sub = rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rate = rospy.Rate(25)

    d= rospy.get_param("~distance_laser")
    vx= rospy.get_param("~forward_speed")
    wz= rospy.get_param("~rotation_speed")
    vf= rospy.get_param("~speed_factor")
    
    
if __name__ == '__main__':
    try:
        main()
        rospy.spin()
    except rospy.ROSInterruptException:
        shutdown()

    


if __name__ == '__main__':
    main()
