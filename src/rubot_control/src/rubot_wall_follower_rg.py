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

    bright_min = int(30 * scanRangesLengthCorrectionFactor)
    bright_max = int(90 * scanRangesLengthCorrectionFactor)
    right_min = int(90 * scanRangesLengthCorrectionFactor)
    right_max = int(120 * scanRangesLengthCorrectionFactor)
    fright_min = int(120 * scanRangesLengthCorrectionFactor)
    fright_max = int(170 * scanRangesLengthCorrectionFactor)
    front_min= int(170 * scanRangesLengthCorrectionFactor)
    front_max = int(190 * scanRangesLengthCorrectionFactor)

    regions = {
        'bright':  min(min(msg.ranges[bright_min:bright_max]), 3),
        'right':  min(min(msg.ranges[right_min:right_max]), 3),
        'fright': min(min(msg.ranges[fright_min:fright_max]), 3),
        'front':  min(min(msg.ranges[front_min:front_max]), 3),
    }

    take_action(regions)


def take_action(regions):
    msg = Twist()
    linear_x = 0
    angular_z = 0

    state_description = ''

    if regions['front'] > d and regions['fright'] > 2*d and regions['right'] > 2*d and regions['bright'] > 2*d:
        state_description = 'case 1 - nothing'
        linear_x = vx
        angular_z = 0
    elif regions['front'] < d:
        state_description = 'case 2 - front'
        linear_x = vx
        angular_z = 0
    elif regions['fright'] < d:
        state_description = 'case 3 - fright'
        linear_x = vx
        angular_z = 0
    elif regions['front'] > d and regions['right'] < d:
        state_description = 'case 4 - right'
        linear_x = vx
        angular_z = 0
    elif regions['bright'] < d:
        state_description = 'case 5 - bright'
        linear_x = vx
        angular_z = 0
    else:
        state_description = 'case 6 - Far'
        linear_x = vx
        angular_z = 0

    rospy.loginfo(state_description)
    msg.linear.x = linear_x
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
