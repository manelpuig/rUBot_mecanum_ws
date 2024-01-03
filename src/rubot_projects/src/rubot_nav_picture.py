#!/usr/bin/env python3
import rospy
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, radians
import yaml
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal

def create_pose_stamped(position_x, position_y, rotation_z):
    goal = MoveBaseGoal()# Has to be created here
    q_x, q_y, q_z, q_w = quaternion_from_euler(0.0, 0.0, rotation_z)
    goal.target_pose.header.frame_id = "map"
    goal.target_pose.header.stamp = rospy.Time.now()
    goal.target_pose.pose.position.x = position_x
    goal.target_pose.pose.position.y = position_y
    goal.target_pose.pose.position.z = 0.0
    goal.target_pose.pose.orientation.x = q_x
    goal.target_pose.pose.orientation.y = q_y
    goal.target_pose.pose.orientation.z = q_z
    goal.target_pose.pose.orientation.w = q_w
    return goal

def init_pose():
    pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=10)
    initial_pose = create_initpose(0.5, -0.5, radians(-90))
    rate = rospy.Rate(1) # 1hz has to be low value
    pub.publish(initial_pose)
    rate.sleep()
    rospy.loginfo("Init Pose done!")
    
def nav2goals():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    client.wait_for_server()
    
    goal1 = rospy.get_param("~goal1")
    goal2 = rospy.get_param("~goal2")

    goal_pose1 = create_pose_stamped(goal1['x'], goal1['y'], radians(goal1['w']))
    goal_pose2 = create_pose_stamped(goal2['x'], goal2['y'], radians(goal2['w']))

    # --- Follow Waypoints ---
    waypoints = [goal_pose1, goal_pose2]
    for i in range(2):
        client.send_goal(waypoints[i])
        wait = client.wait_for_result(rospy.Duration(20))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal execution done!")   

if __name__ == '__main__':
    try:
        rospy.init_node('rubot_nav_picture')
        nav2goals()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")