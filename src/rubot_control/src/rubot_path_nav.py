#!/usr/bin/env python3
import rospy
from rubot_nav import move_rubot
from math import sqrt,sin,cos, pi


def square_path(v, td):
    move_rubot(v,0,0,td)
    move_rubot(0,v,0,td)
    move_rubot(-v,0,0,td)
    move_rubot(0,-v,0,td)


def triangular_path(v, td):
    move_rubot(v,0,0,td)
    move_rubot(-v,v,0,td/sqrt(2))
    move_rubot(-v,-v,0,td/sqrt(2))


def rombe_path(v, td):
    diagonal_speed = v / sqrt(2)
    move_rubot(diagonal_speed, diagonal_speed, 0, td)
    move_rubot(diagonal_speed, -diagonal_speed, 0, td)
    move_rubot(-diagonal_speed, -diagonal_speed, 0, td)
    move_rubot(-diagonal_speed, diagonal_speed, 0, td)

def star_path(v, td):
    for _ in range(5):
        move_rubot(v, 0, 0, td)
        move_rubot(0, 0, pi/3, td/10)


if __name__ == '__main__':
    try:
        rospy.init_node('rubot_nav', anonymous=False)
        v= rospy.get_param("~v")
        w= rospy.get_param("~w")
        td= rospy.get_param("~td")
        path= rospy.get_param("~path")

        if path == "Square":
            square_path(v, td)

        elif path == "Triangular":
            triangular_path(v, td)
        
        elif path == "Rombe":
            rombe_path(v, td)
        
        elif path == "Star":
            star_path(v, td)
        else:
            print('Error: unknown movement')

    except rospy.ROSInterruptException:
        pass
