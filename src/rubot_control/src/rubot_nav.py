#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler

def odom_callback(data):
    global robot_x
    global robot_y
    global robot_f
    robot_x=data.pose.pose.position.x
    robot_y=data.pose.pose.position.y
    q=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    (roll, pitch, yaw)=euler_from_quaternion(q)
    rospy.loginfo("Robot Odometry x= %f\n",robot_x)
    rospy.loginfo("Robot Odometry y= %f\n",robot_y)
    rospy.loginfo("Robot Odometry yaw= %.0f\n",math.degrees(yaw))
	
def move_rubot(lin_velx,lin_vely,ang_vel,distance):
    global robot_x
    global robot_y
    global robot_f
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom',Odometry, odom_callback)
    rate = rospy.Rate(10) # 10hz
    vel = Twist()
    while not rospy.is_shutdown():
        vel.linear.x = lin_velx
        vel.linear.y = lin_vely
        vel.linear.z = 0
        vel.angular.x = 0
        vel.angular.y = 0
        vel.angular.z = ang_vel
        rospy.loginfo("Linear Vel_x = %f: Linear Vel_y = %f: Angular Vel = %f",lin_velx,lin_vely,ang_vel)

        if(robot_x >= distance):
            rospy.loginfo("Robot Reached destination")
            rospy.logwarn("Stopping robot")
            vel.linear.x = 0
            vel.linear.y = 0
            vel.angular.z = 0
            pub.publish(vel)
            break
        else:
            pub.publish(vel)
            rate.sleep()

if __name__ == '__main__':
    try:
        rospy.init_node('rubot_nav', anonymous=False)
        vx= rospy.get_param("~vx")
        vy= rospy.get_param("~vy")
        w= rospy.get_param("~w")
        d= rospy.get_param("~d")
        robot_x = 0
        robot_y = 0
        robot_f = 0
        move_rubot(vx,vy,w,d)
    except rospy.ROSInterruptException:
        pass
