#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class Rubot:
    def __init__(self):
        # Initialize the ROS node
        rospy.init_node('rubot_control', anonymous=True)

        # Retrieve goal parameters
        self.x_goal = rospy.get_param("~x", 0.0)
        self.y_goal = rospy.get_param("~y", 0.0)
        f_goal_deg = rospy.get_param("~f", 0.0)
        self.f_goal = radians(f_goal_deg)
        self.q_goal = quaternion_from_euler(0, 0, self.f_goal)

        # Initialize current pose variables
        self.x_pose = 0.0
        self.y_pose = 0.0
        self.yaw = 0.0

        # Publisher for velocity commands
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscriber for odometry updates
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odom)

        self.odom = Odometry()
        self.rate = rospy.Rate(10)

        # Set up shutdown handler
        rospy.on_shutdown(self.shutdown_handler)

    def update_odom(self, data):
        """Callback function for updating the robot's current odometry."""
        self.odom = data
        self.x_pose = round(self.odom.pose.pose.position.x, 2)
        self.y_pose = round(self.odom.pose.pose.position.y, 2)
        orientation_q = self.odom.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        self.yaw = euler_from_quaternion(orientation_list)[2]

    def euclidean_distance(self, goal_odom):
        """Calculate the Euclidean distance between the current pose and the goal."""
        return sqrt(pow((goal_odom.pose.pose.position.x - self.x_pose), 2) +
                    pow((goal_odom.pose.pose.position.y - self.y_pose), 2))

    def linear_vel(self, goal_odom, constant=0.5):
        """Calculate the linear velocity towards the goal."""
        return constant * self.euclidean_distance(goal_odom)

    def steering_angle(self, goal_odom):
        """Calculate the steering angle towards the goal."""
        return atan2(goal_odom.pose.pose.position.y - self.y_pose, goal_odom.pose.pose.position.x - self.x_pose)

    def angular_vel(self, goal_odom, constant=5):
        """Calculate the angular velocity towards the goal."""
        return constant * (self.steering_angle(goal_odom) - self.yaw)

    def move_to_pose(self):
        """Move the robot towards the goal pose."""
        goal_odom = Odometry()
        goal_odom.pose.pose.position.x = self.x_goal
        goal_odom.pose.pose.position.y = self.y_goal

        distance_tolerance = 0.1
        angle_tolerance = 0.1

        vel_msg = Twist()

        while not rospy.is_shutdown() and self.euclidean_distance(goal_odom) >= distance_tolerance:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_odom)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_odom)

            # Publishing velocity message
            self.velocity_publisher.publish(vel_msg)
            rospy.loginfo("Distance to target: %.2f", self.euclidean_distance(goal_odom))

            # Maintain the loop rate
            self.rate.sleep()

        # Align with the final orientation
        while not rospy.is_shutdown() and abs(self.f_goal - self.yaw) >= angle_tolerance:
            vel_msg.linear.x = 0
            vel_msg.angular.z = (self.f_goal - self.yaw) * 0.5

            # Publishing velocity message
            self.velocity_publisher.publish(vel_msg)
            rospy.loginfo("Orientation error: %.2f", abs(degrees(self.f_goal - self.yaw)))

            # Maintain the loop rate
            self.rate.sleep()

        # Stop the robot
        self.stop_robot()

    def stop_robot(self):
        """Send a zero velocity command to stop the robot."""
        vel_msg = Twist()
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Robot stopped.")

    def shutdown_handler(self):
        """Handle the shutdown signal by stopping the robot."""
        self.stop_robot()
        rospy.loginfo("Shutdown initiated. Robot stopped.")

if __name__ == '__main__':
    try:
        rubot = Rubot()
        rubot.move_to_pose()
    except rospy.ROSInterruptException:
        pass
