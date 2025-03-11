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

class Regions:
 def __init__(self, msg):
    bright_min = int(30 * scanRangesLengthCorrectionFactor)
    bright_max = int(90 * scanRangesLengthCorrectionFactor)
    right_min = int(90 * scanRangesLengthCorrectionFactor)
    right_max = int(120 * scanRangesLengthCorrectionFactor)
    fright_min = int(120 * scanRangesLengthCorrectionFactor)
    fright_max = int(170 * scanRangesLengthCorrectionFactor)
    front_min= int(170 * scanRangesLengthCorrectionFactor)
    front_max = int(190 * scanRangesLengthCorrectionFactor)

    self.bright = min(min(msg.ranges[bright_min:bright_max]), 3)
    self.right =  min(min(msg.ranges[right_min:right_max]), 3)
    self.fright = min(min(msg.ranges[fright_min:fright_max]), 3)
    self.front =  min(min(msg.ranges[front_min:front_max]), 3)

class Movement:
    def __init__(self):
        self.linear_x = 0
        self.angular_z = 0
        self.state_description = ''
    
    def set_linear_x(self, vx):
        self.linear_x = vx
    
    def set_angular_z(self, vz):
        self.angular_z = vz
    
    def set_description(self, d):
        self.state_description = d

    

def clbk_laser(msg):
    # En la primera ejecucion, calculamos el factor de correcion
    global isScanRangesLengthCorrectionFactorCalculated
    global scanRangesLengthCorrectionFactor
    
    if not isScanRangesLengthCorrectionFactorCalculated:
            scanRangesLengthCorrectionFactor = len(msg.ranges) / 360
            isScanRangesLengthCorrectionFactorCalculated = True

    regions = Regions(msg)

    take_action(regions)


def take_action(regions):
    msg = Twist()
    movement = Movement()

    if regions.front > d and regions.fright > 2*d and regions.right > 2*d and regions.bright > 2*d:
        movement.set_description('case 1 - nothing')
        movement.set_linear_x(vx)  
        movement.set_angular_z(0)  
    elif regions.front < d:
        movement.set_description('case 2 - front')
        movement.set_linear_x(0)
        movement.set_angular_z(wz)
    elif regions.fright < d:
        movement.set_description('case 3 - fright')
        movement.set_linear_x(0)
        movement.set_angular_z(wz)
    elif regions.front > d and regions.right < d:
        movement.set_description('case 4 - right')
        movement.set_linear_x(vx)
        movement.set_angular_z(0)
    elif regions.bright < d:
        movement.set_description('case 5 - bright')
        movement.set_linear_x(0)
        movement.set_angular_z(-wz)
    else:
        movement.set_description('case 6 - Far')
        movement.set_linear_x(0)
        movement.set_angular_z(-wz)

    rospy.loginfo(movement.state_description)
    msg.linear.x = movement.linear_x
    msg.angular.z = movement.angular_z
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


