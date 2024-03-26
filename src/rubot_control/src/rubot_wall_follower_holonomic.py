#! /usr/bin/env python3
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import numpy as np
import time

pub = None
d = 0
vx = 0
wz = 0
vy = 0 
vf = 0
last_turn_time = rospy.Time.now()
turn_duration = rospy.Duration(2)

def clbk_laser(msg):
    global pub, d, vx, wz, vy, vf, last_turn_time, turn_duration

    ranges = np.array(msg.ranges)
    ranges[np.isinf(ranges)] = 10.0
    min_distance_index = np.argmin(ranges)
    closest_distance = ranges[min_distance_index]
    angle_closest_distance = (min_distance_index * 360 / len(ranges)) - 180

    twist_msg = Twist()
    current_time = rospy.Time.now()

    if closest_distance < d:
        if -80 < angle_closest_distance < 80:
            if current_time - last_turn_time > turn_duration:
                twist_msg.linear.x = 0
                twist_msg.linear.y = vy * vf
                twist_msg.angular.z = 0
                last_turn_time = current_time
            else:
                twist_msg.linear.x = 0
                twist_msg.linear.y = 0
                twist_msg.angular.z = -np.sign(angle_closest_distance) * wz * vf
        elif -100 < angle_closest_distance <= -80 or 80 <= angle_closest_distance < 100:
            twist_msg.linear.x = 0
            twist_msg.linear.y = -np.sign(angle_closest_distance) * vy * vf
            twist_msg.angular.z = 0
        else:
            twist_msg.linear.x = -vx * vf
            twist_msg.linear.y = 0
            twist_msg.angular.z = 0
    else:
        twist_msg.linear.x = vx * vf
        twist_msg.linear.y = 0
        twist_msg.angular.z = 0
        last_turn_time = rospy.Time(0)

    pub.publish(twist_msg)


def shutdown():
    global pub
    twist_msg = Twist()
    twist_msg.linear.x = 0
    twist_msg.linear.y = 0
    twist_msg.angular.z = 0
    pub.publish(twist_msg)
    rospy.loginfo("Stop rUBot")

def main():
    global pub, d, vx, wz, vy, vf
    rospy.init_node('wall_follower')
    d = rospy.get_param("~distance_laser")
    vx = rospy.get_param("~forward_speed")
    wz = rospy.get_param("~rotation_speed")
    vy = rospy.get_param("~lateral_speed", 0.5) 
    vf = rospy.get_param("~speed_factor")

    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/scan', LaserScan, clbk_laser)
    rospy.on_shutdown(shutdown)
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        shutdown()
