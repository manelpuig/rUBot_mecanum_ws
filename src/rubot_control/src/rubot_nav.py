#!/usr/bin/env python3
import math
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import sys
robot_x = 0
robot_y = 0
robot_f = 0

def odom_callback(data):
    global robot_x
    global robot_y
    global robot_f
    robot_x=data.pose.pose.position.x
    robot_y=data.pose.pose.position.y
    q=[data.pose.pose.orientation.x,data.pose.pose.orientation.y,
    data.pose.pose.orientation.z,data.pose.pose.orientation.w]
    (roll, pitch, yaw)=euler_from_quaternion(q)
    robot_f = math.degrees(yaw)
    #rospy.loginfo("Robot Odometry x= %f\n",robot_x)
    #rospy.loginfo("Robot Odometry y= %f\n",robot_y)
    #rospy.loginfo("Robot Odometry yaw= %.0f\n",math.degrees(yaw))

def move_rubot(lin_velx,lin_vely,ang_vel,time_duration):
    global robot_x
    global robot_y
    global robot_f
    time_begin = 0
    time_end = 0
    duration = 0
    duration_s = 0
    end_mov = False
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/odom',Odometry, odom_callback)
    rate = rospy.Rate(10) # 10hz
    vel = Twist()
    time_begin = rospy.Time.now()
    if time_begin == 0:# when using software time usually returns 0
        time_begin = rospy.Time.now()
        n+=1
        print(n)
    rospy.loginfo("Time_begin = " + str(time_begin))
    while not end_mov:
        if (duration_s <= time_duration):
            rospy.loginfo("Robot running")
            rospy.loginfo("Duration_s= "+str(duration_s))
            #rospy.loginfo("Linear Vel_x = %f: Linear Vel_y = %f: Angular Vel = %f",lin_velx,lin_vely,ang_vel)
            vel.linear.x = lin_velx
            vel.linear.y = lin_vely
            vel.angular.z = ang_vel
            pub.publish(vel)
            rate.sleep()
        else:
            rospy.logwarn("Stopping robot")
            vel.linear.x = 0.0
            vel.linear.y = 0.0
            vel.angular.z = 0.0
            pub.publish(vel)
            end_mov = True
            rate.sleep()
            
        time_end = rospy.Time.now()
        rospy.loginfo("Time_end = " + str(time_end))
        duration = time_end - time_begin
        duration_s = duration.to_sec()
        rospy.loginfo("Time_begin = " + str(time_begin))
        rospy.loginfo("Time duration " + str(duration_s) + " secs" + " from " + str(time_duration))


if __name__ == '__main__':
    try:
        rospy.init_node('rubot_nav', anonymous=False)
        vx= rospy.get_param("~vx")
        vy= rospy.get_param("~vy")
        w= rospy.get_param("~w")
        td= rospy.get_param("~td")
        move_rubot(vx,vy,w,td)
    except rospy.ROSInterruptException:
        pass
