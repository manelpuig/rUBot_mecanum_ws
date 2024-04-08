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

        self._r = rospy.Rate(25)
        
        # Propiedades secundarias

        # Our Lidar has more than 720 laser beams and not all the Lidars have the same number
        # Se debe de calcular en la primera ejecucion de __callbackLaser(). Esta
        # variable sirve para asegurar que solo se ejecuta este calculo del
        # factor de correccion una sola vez.
        #self.__isScanRangesLengthCorrectionFactorCalculated = False
        #self.__scanRangesLengthCorrectionFactor = 2

    def start(self):

        while not rospy.is_shutdown():
            self._cmdVel.publish(self._msg)
            self._r.sleep()

    def callbackLaser(self, scan):
        """Funcion ejecutada cada vez que se recibe un mensaje en /scan."""
        # En la primera ejecucion, calculamos el factor de correcion del Lidar
                
        closestDistance, elementIndex = min((val, idx) for (idx, val) in enumerate(scan.ranges) if scan.range_min < val < scan.range_max)
        angleClosestDistance = (elementIndex / 2)  

        angleClosestDistance= self.__wrapAngle(angleClosestDistance)
        rospy.loginfo("Degree wraped %5.2f ",(angleClosestDistance))
        
        if angleClosestDistance > 0:
            angleClosestDistance=(angleClosestDistance-180)
        else:
            angleClosestDistance=(angleClosestDistance+180)
			
        rospy.loginfo("Closest distance of %5.2f m at %5.1f degrees.",closestDistance, angleClosestDistance)
                      

        if closestDistance < self._distanceLaser:
            if abs(angleClosestDistance) < 100:
                kp_f = 0.01
                kp_d = 0.4
                error_f = angleClosestDistance + 90
                error_d = self._distanceLaser - closestDistance
                self._msg.linear.x = self._forwardSpeed * self._speedFactor
                self._msg.linear.y = kp_d * error_d
                self._msg.angular.z = kp_f * error_f * self._rotationSpeed * self._speedFactor
                rospy.logwarn("Driving robot: vx=%1.1f vy=%1.2f w=%1.2f", self._msg.linear.x, self._msg.linear.y, self._msg.angular.z)

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
        self._msg.linear.y = 0
        self._msg.angular.z = 0
        self._cmdVel.publish(self._msg)
        rospy.loginfo("Stop rUBot")

if __name__ == '__main__':
    try:
        rUBot1 = rUBot()
        rUBot1.start()
        rospy.spin()
        rUBot1.shutdown()
    except rospy.ROSInterruptException: rUBot1.shutdown()#pass
