#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, radians

def create_initpose(position_x, position_y, orientation_z):
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, orientation_z)
    pose_msg = PoseWithCovarianceStamped()
    pose_msg.header.frame_id = 'map'
    pose_msg.header.stamp = rospy.Time.now()
    pose_msg.pose.pose.position.x = position_x
    pose_msg.pose.pose.position.y = position_y
    pose_msg.pose.pose.position.z = 0.0
    pose_msg.pose.pose.orientation.x = q_x
    pose_msg.pose.pose.orientation.y = q_y
    pose_msg.pose.pose.orientation.z = q_z
    pose_msg.pose.pose.orientation.w = q_w
    return pose_msg

def main():
    rospy.init_node('pub_initpose_node', anonymous=True)
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    initial_pose = create_initpose(-0.8, -0.5, radians(180))
    rate = rospy.Rate(10) # 10hz
    while not rospy.is_shutdown():
        pub.publish(initial_pose)
        rate.sleep()
    
if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")