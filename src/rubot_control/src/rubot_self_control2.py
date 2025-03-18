import rospy
import numpy as np
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class SimpleNavigator:
    def __init__(self):
        rospy.init_node("simple_nav")

        # Parameters
        self.safe_distance = rospy.get_param("~safe_distance", 0.5)
        self.forward_speed = rospy.get_param("~forward_speed", 0.2)
        self.backward_speed = rospy.get_param("~backward_speed", -0.1)
        self.rotation_speed = rospy.get_param("~rotation_speed", 1.0)

        # Velocity publisher
        self.cmd_vel_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.laser_callback, queue_size=10)

        self.closest_distance = float("inf")
        self.rate = rospy.Rate(10)  # 10 Hz

    def laser_callback(self, scan):
        ranges = np.array(scan.ranges)
        ranges[np.isinf(ranges)] = 999  # Replace inf values with a large number
        self.closest_distance = np.min(ranges)

    def run(self):
        msg = Twist()
        while not rospy.is_shutdown():
            if self.closest_distance < self.safe_distance:
                msg.linear.x = self.backward_speed
                msg.angular.z = self.rotation_speed
            else:
                msg.linear.x = self.forward_speed
                msg.angular.z = 0

            self.cmd_vel_pub.publish(msg)
            self.rate.sleep()

if __name__ == "__main__":
    node = SimpleNavigator()
    node.run()