#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist, Pose, Quaternion
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt, degrees, radians
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class rubot:

    def __init__(self):
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        rospy.init_node('rubot_control', anonymous=True)
        # Define goal odometry from parameters
        self.x_goal = rospy.get_param("~x")
        self.y_goal = rospy.get_param("~y")
        self.f_goal = radians(rospy.get_param("~f"))
        self.q_goal = quaternion_from_euler(0,0,self.f_goal)
        # Define initial values for actual odometry (read in callback function)
        self.x_pose=0
        self.y_pose=0
        self.yaw=0
        
        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # A subscriber to the topic '/odom'. self.update_pose is called
        # when a message of type Odometry is received.
        self.odom_subscriber = rospy.Subscriber('/odom', Odometry, self.update_odom)

        self.odom = Odometry()
        self.rate = rospy.Rate(10)

    def update_odom(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.odom = data
        self.x_pose = round(self.odom.pose.pose.position.x, 2)
        self.y_pose = round(self.odom.pose.pose.position.y, 2)
        self.z_pose = round(self.odom.pose.pose.position.z, 2)
        self.orientation_q = self.odom.pose.pose.orientation
        self.orientation_list = [self.orientation_q.x, self.orientation_q.y, self.orientation_q.z, self.orientation_q.w]
        self.rpw = euler_from_quaternion (self.orientation_list)
        self.yaw = self.rpw[2]
        
    def euclidean_distance(self, goal_odom):
        """Euclidean distance between current pose and the goal."""
        return sqrt(pow((goal_odom.pose.pose.position.x - self.x_pose), 2) +
                    pow((goal_odom.pose.pose.position.y - self.y_pose), 2))

    def linear_vel(self, goal_odom, constant=0.5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * self.euclidean_distance(goal_odom)

    def steering_angle(self, goal_odom):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_odom.pose.pose.position.y - self.y_pose, goal_odom.pose.pose.position.x - self.x_pose)

    def angular_vel(self, goal_odom, constant=5):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return constant * (self.steering_angle(goal_odom) - self.yaw)

    def move2pose(self):
        """Moves the turtle to the goal."""
        goal_odom = Odometry()

        # Get the input from the user.
        goal_odom.pose.pose.position.x = self.x_goal
        goal_odom.pose.pose.position.y = self.y_goal
                
        # Please, insert tolerances a number slightly greater than 0 (e.g. 0.01).
        distance_tolerance = 0.1
        angle_tolerance = 0.1

        vel_msg = Twist()

        while self.euclidean_distance(goal_odom) >= distance_tolerance:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(goal_odom)
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = self.angular_vel(goal_odom)
            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)
            rospy.loginfo("Distance to target: " + str(round(self.euclidean_distance(goal_odom), ndigits=2)))
            # Publish at the desired rate.
            self.rate.sleep()
        else:
            while abs(self.f_goal-self.yaw) >= angle_tolerance:
                # Linear velocity in the x-axis.
                vel_msg.linear.x = 0
                vel_msg.linear.y = 0
                vel_msg.linear.z = 0
                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0
                vel_msg.angular.y = 0
                vel_msg.angular.z = (self.f_goal-self.yaw)*0.5
                 # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)
                rospy.loginfo("Orientation error: " + str(round(degrees(self.f_goal-self.yaw), ndigits=2)))
                # Publish at the desired rate.
                self.rate.sleep()

        # Stopping our robot after the movement is over.
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        # Publishing our vel_msg
        self.velocity_publisher.publish(vel_msg)
        rospy.loginfo("Goal POSE reached!")

        # If we press control + C, the node will stop.
        rospy.spin()

if __name__ == '__main__':
    try:
        rubot1 = rubot()
        rubot1.move2pose()
    except rospy.ROSInterruptException:
        pass