#!/usr/bin/env python3

import rospy
import sys
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class rUBot:

    def __init__(self):

        rospy.init_node("rubot_nav", anonymous=False)
        self._distanceLaser = rospy.get_param("~distance_laser")
        self._speedFactor = rospy.get_param("~speed_factor")
        self._forwardSpeed = rospy.get_param("~forward_speed")
        self._backwardSpeed = rospy.get_param("~backward_speed")
        self._rotationSpeed = rospy.get_param("~rotation_speed")

        self._msg = Twist()
        self._cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callbackLaser)
        rospy.on_shutdown(self.shutdown)

        self._r = rospy.Rate(5)

    def start(self):

        while not rospy.is_shutdown():
            self._cmdVel.publish(self._msg)
            self._r.sleep()

    def callbackLaser(self, scan):

        closestDistance, elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges) if scan.range_min < val < scan.range_max
        )
        angleClosestDistance = self.__wrapAngle(elementIndex / 2) # RPLidar zero angle in backside

        rospy.loginfo("Closest distance of %5.2f m at %5.1f degrees.",
                      closestDistance, angleClosestDistance)

        if closestDistance < self._distanceLaser and -90 < angleClosestDistance < 90:
            self._msg.linear.x = self._backwardSpeed * self._speedFactor
            self._msg.angular.z = -self.__sign(angleClosestDistance) * self._rotationSpeed * self._speedFactor
            rospy.logwarn("Within laser distance threshold. Rotating the robot (z=%4.1f)...", self._msg.angular.z)

        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.angular.z = 0

    def __sign(self, val):

        if val >= 0:
            return 1
        else:
            return -1

    def __wrapAngle(self, angle):
        if 0 <= angle <= 180:
            return angle
        else:
            return angle - 360

    def shutdown(self):
        self._msg.linear.x = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)

if __name__ == '__main__':
    try:
        rUBot1 = rUBot()
        rUBot1.start()
        rospy.spin()
    except rospy.ROSInterruptException: pass