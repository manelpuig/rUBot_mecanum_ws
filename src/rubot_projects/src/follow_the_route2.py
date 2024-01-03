#!/usr/bin/env python3

import rospy
import yaml
from take_photo import TakePhoto
from rUBot_mecanum_ws.Documentation.files.Doc.Old_Docs.go_to_specific_point_on_map import GoToPose
from tf.transformations import quaternion_from_euler, euler_from_quaternion
from math import radians, degrees

if __name__ == '__main__':

    # Read information from yaml file
    with open("./src/robot_projects/rubot_projects/src/route2.yaml", 'r') as stream:
        dataMap = yaml.full_load(stream)

    try:
        # Initialize
        rospy.init_node('follow_route', anonymous=False)
        navigator = GoToPose()
        camera = TakePhoto()

        for obj in dataMap:

            if rospy.is_shutdown():
                break

            name = obj['filename']

            # Navigation
            rospy.loginfo("Go to %s pose", name[:-4])
            orientation=obj['angle']
            q = quaternion_from_euler(0.0, 0.0, radians(orientation['fi']))
            quaternion = {'r1' : q[0], 'r2' : q[1], 'r3' : q[2], 'r4' : q[3]}
            success = navigator.goto(obj['position'], quaternion)
            if not success:
                rospy.loginfo("Failed to reach %s pose", name[:-4])
                continue
            rospy.loginfo("Reached %s pose", name[:-4])

            # Take a photo
            if camera.take_picture(name):
                rospy.loginfo("Saved image " + name)
            else:
                rospy.loginfo("No images received")

            rospy.sleep(1)

    except rospy.ROSInterruptException:
        rospy.loginfo("Ctrl-C caught. Quitting")