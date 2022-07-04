#!/usr/bin/env python3
import rospy
from rubot_navigation import move_rubot
from math import sqrt,sin,cos


def square_path(v,td):
    move_rubot(v,0,0,td)
    move_rubot(0,v,0,td)
    move_rubot(-v,0,0,td)
    move_rubot(0,-v,0,td)


def circular_path(v,td):
    w=5
    for t in range(0,td,int(td/4)):
        move_rubot(v*cos(w*t),v*sin(w*t),0,t)


def triangular_path(v, td):
    move_rubot(v,0,0,td)
    move_rubot(-v,v,0,td/sqrt(2))
    move_rubot(-v,-v,0,td/sqrt(2))


if __name__ == '__main__':
    try:
        rospy.init_node('rubot_nav', anonymous=False)
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        td= rospy.get_param("~td")
        path= rospy.get_param("~path")

        if path == "Circular":
            circular_path(v,td)

        elif path == "Square":
            square_path(v, td)

        elif path == "Triangular":
            triangular_path(v, td)
        else:
            print('Error: unknown movement')

    except rospy.ROSInterruptException:
        pass
