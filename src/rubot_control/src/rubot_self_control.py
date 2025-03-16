#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from numpy import sign


class rUBot:
    def __init__(self):
        rospy.init_node("rubot_nav", anonymous=False)

        self._distanceLaser = rospy.get_param("~distance_laser", 0.5)
        self._speedFactor = rospy.get_param("~speed_factor", 1.0)
        self._forwardSpeed = rospy.get_param("~forward_speed", 0.2)
        self._backwardSpeed = rospy.get_param("~backward_speed", -0.1)
        self._rotationSpeed = rospy.get_param("~rotation_speed", 1.0)

        self._msg = Twist()
        self._cmdVel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self._scan_sub = rospy.Subscriber("/scan", LaserScan, self.scan_callback)
        rospy.on_shutdown(self.shutdown_callback)

        self._rate = rospy.Rate(25)

        self.__isScanRangesLengthCorrectionFactorCalculated = False
        self.__scanRangesLengthCorrectionFactor = 2

        self._shutting_down = False #flag to ensure a proper shutdown

    def scan_callback(self, scan):
        if self._shutting_down: #check flag
            return

        if not self.__isScanRangesLengthCorrectionFactorCalculated:
            self.__scanRangesLengthCorrectionFactor = int(len(scan.ranges) / 360)
            self.__isScanRangesLengthCorrectionFactorCalculated = True
            #rospy.loginfo(f"Scan Ranges correction factor: {self.__scanRangesLengthCorrectionFactor:.2f}, number of points: {len(scan.ranges)}")

        closestDistance, elementIndex = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges)
            if scan.range_min < val < scan.range_max
        )
        angleClosestDistance = elementIndex / self.__scanRangesLengthCorrectionFactor
        #rospy.loginfo(f"Angle Closest Distance (raw): {angleClosestDistance:.0f} degrees")
        # To take into account the Lidar Zero angle is on the back
        angleClosestDistance = angleClosestDistance - 180
        # To wrapp the angle to the ranege [+180, -180]
        angleClosestDistance = (angleClosestDistance + 180) % 360 - 180
        #rospy.loginfo(f"Angle Closest Distance (wrapped): {angleClosestDistance:.2f} degrees")
        rospy.loginfo(f"Closest distance: {closestDistance:.2f} meters and Angle: {angleClosestDistance:.1f} degrees")

        if closestDistance < self._distanceLaser and -80 < angleClosestDistance < 80:
            self._msg.linear.x = self._backwardSpeed * self._speedFactor
            self._msg.angular.z = -sign(angleClosestDistance) * self._rotationSpeed * self._speedFactor
        else:
            self._msg.linear.x = self._forwardSpeed * self._speedFactor
            self._msg.angular.z = 0.0

        #rospy.logwarn(f"Twist vx={self._msg.linear.x:.1f} m/s, wz={self._msg.angular.z:.1f} rad/s")
        self._cmdVel.publish(self._msg)
        self._rate.sleep()
        #Important!: With real rUBot, due to latency, increase the rate to >=25Hz and limite the loginfo to avoid delays in callback update

    def shutdown_callback(self):
        self._shutting_down = True #set flag
        self._scan_sub.unregister() #unsubscribe
        self._msg.linear.x = 0
        self._msg.linear.y = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)
        self._rate.sleep()
        rospy.loginfo("rUBot stopped.")

if __name__ == '__main__':
    rUBot1 = rUBot()
    rospy.spin()
