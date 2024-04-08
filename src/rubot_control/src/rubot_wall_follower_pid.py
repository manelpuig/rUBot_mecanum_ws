#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class rUBot:
    def __init__(self):
        rospy.init_node("rubot_nav", anonymous=False)

        # Parameters
        self._distance_laser = rospy.get_param("~distance_laser")
        self._speed_factor = rospy.get_param("~speed_factor")
        self._forward_speed = rospy.get_param("~forward_speed")
        self._rotation_speed = rospy.get_param("~rotation_speed")

        # Twist message
        self._msg = Twist()
        self._cmd_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber("/scan", LaserScan, self.callback_laser)

        # Rate
        self._rate = rospy.Rate(25)

        # PID parameters for angle control
        self._kp_angle = 0.02
        self._ki_angle = 0.01
        self._kd_angle = 0.00001
        self._integral_angle = 0.0
        self._prev_error_angle = 0.0

        # PID parameters for distance control
        self._kp_distance = 0.5
        self._ki_distance = 0.8
        self._kd_distance = 0.00001
        self._integral_distance = 0.0
        self._prev_error_distance = 0.0

    def start(self):
        while not rospy.is_shutdown():
            self._cmd_vel.publish(self._msg)
            self._rate.sleep()

    def callback_laser(self, scan):
        closest_distance, element_index = min(
            (val, idx) for (idx, val) in enumerate(scan.ranges)
            if scan.range_min < val < scan.range_max
        )
        angle_closest_distance = (element_index / 2)

        rospy.loginfo(
            "Closest distance of %5.2f m at %5.1f degrees.",
            closest_distance,
            angle_closest_distance,
        )

        # Angle error
        angle_error = angle_closest_distance - 90.0

        # Angle controller
        p_term_angle = self._kp_angle * angle_error
        self._integral_angle += angle_error
        i_term_angle = self._ki_angle * self._integral_angle
        d_term_angle = self._kd_angle * (angle_error - self._prev_error_angle)
        control_signal_angle = p_term_angle + i_term_angle + d_term_angle
        self._prev_error_angle = angle_error

        # Distance error
        distance_error = self._distance_laser - closest_distance

        # Distance controller
        p_term_distance = self._kp_distance * distance_error
        self._integral_distance += distance_error
        i_term_distance = self._ki_distance * self._integral_distance
        d_term_distance = self._kd_distance * (
            distance_error - self._prev_error_distance
        )
        control_signal_distance = p_term_distance + i_term_distance + d_term_distance
        self._prev_error_distance = distance_error

        # Update twist message
        self._msg.linear.x = self._forward_speed * self._speed_factor
        self._msg.linear.y = control_signal_distance *self._forward_speed * self._speed_factor
        self._msg.angular.z = control_signal_angle * self._rotation_speed * self._speed_factor

        # Log case
        if closest_distance < self._distance_laser and angle_closest_distance < 90.0:
            rospy.logwarn(
                "Case 1. Driving the robot (vy=%4.1f, w=%4.1f)...", self._msg.linear.y, self._msg.angular.z
            )
        elif (
            closest_distance < self._distance_laser
            and 90.0 <= angle_closest_distance < 180.0
        ):
            rospy.logwarn(
                "Case 2. Driving the robot (vy=%4.1f, w=%4.1f)...", self._msg.linear.y, self._msg.angular.z            )
        else:
            # Reset integrals
            self._integral_angle = 0.0
            self._integral_distance = 0.0
            # Log case
            rospy.logwarn("Case 3. robot (vx=%4.1f)...", self._msg.linear.x)

    def shutdown(self):
        self._msg.linear.x = 0
        self._msg.linear.y = 0
        self._msg.angular.z = 0
        self._cmd_vel.publish(self._msg)
        rospy.loginfo("Stop")


if __name__ == "__main__":
    try:
        rUBot1 = rUBot()
        rUBot1.start()
        rospy.spin()
        rUBot1.shutdown()
    except rospy.ROSInterruptException:
        rUBot1.shutdown()
