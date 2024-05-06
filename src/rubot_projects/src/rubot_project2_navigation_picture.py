#!/usr/bin/env python3
import rospy
import rospkg
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from math import degrees, radians
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import cv2 as cv
from rubot_project1_picture import TakePhoto

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
    
def nav2goals():
    client = actionlib.SimpleActionClient('move_base',MoveBaseAction)
 
    client.wait_for_server()
    
    goal1 = rospy.get_param("~goal1")
    goal2 = rospy.get_param("~goal2")
    img_topic = rospy.get_param("~img_topic")

    goal_pose1 = create_pose_stamped(goal1['x'], goal1['y'], radians(goal1['w']))
    goal_pose2 = create_pose_stamped(goal2['x'], goal2['y'], radians(goal2['w']))
    
    name_photo1= photos_path + goal1['photo_name']
    name_photo2= photos_path + goal2['photo_name']


    # --- Follow Waypoints ---
    waypoints = [goal_pose1, goal_pose2]
    photos = [name_photo1, name_photo2]
    for i in range(2):
        client.send_goal(waypoints[i])
        wait = client.wait_for_result(rospy.Duration(40))
        if not wait:
            rospy.logerr("Action server not available!")
            rospy.signal_shutdown("Action server not available!")
        else:
            rospy.loginfo("Goal execution done!")
            camera = TakePhoto(img_topic, photos[i])
            # Important! Allow up to one second for connection
            rospy.sleep(1)
            camera.save_picture(photos[i])

if __name__ == '__main__':
    try:
        rospy.init_node('movebase_client_waypoints')
        # Initialize the ROS package manager
        rospack = rospkg.RosPack()
        # Get the path of the 'rubot_projects' package
        rubot_projects_path = rospack.get_path('rubot_projects')
        # Construct the full path to the photos directory
        photos_path = rubot_projects_path + '/photos/'
        nav2goals()

    except rospy.ROSInterruptException:
        rospy.loginfo("Navigation test finished.")