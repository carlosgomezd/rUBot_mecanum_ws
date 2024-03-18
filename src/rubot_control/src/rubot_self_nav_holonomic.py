#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist

class rUBotHolonomic:
    def __init__(self):
        rospy.init_node("rubot_holonomic_nav", anonymous=False)
        self.distance_laser = rospy.get_param("~distance_laser")
        self.speed_factor = rospy.get_param("~speed_factor")
        self.forward_speed = rospy.get_param("~forward_speed")
        self.backward_speed = rospy.get_param("~backward_speed")
        self.rotation_speed = rospy.get_param("~rotation_speed") * 0.5  # Aplicar reducción aquí
        self.sideways_speed = rospy.get_param("~sideways_speed")

        self.msg = Twist()
        self.cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        rospy.Subscriber("/scan", LaserScan, self.callback_laser)
        rospy.on_shutdown(self.shutdown)

        self.rate = rospy.Rate(10)

    def start(self):
        while not rospy.is_shutdown():
            self.cmd_vel_publisher.publish(self.msg)
            self.rate.sleep()

    def callback_laser(self, scan):
        front_scan = min(min(scan.ranges[0:30]), min(scan.ranges[-30:]))
        right_scan = min(scan.ranges[31:90])
        left_scan = min(scan.ranges[-90:-31])

        if front_scan < self.distance_laser:
            self.msg.linear.x = 0
            self.msg.angular.z = self.rotation_speed  # Usa la velocidad de rotación reducida
            rospy.loginfo("Obstacle ahead! Rotating to find a clear path.")
        elif right_scan < self.distance_laser:
            self.msg.linear.x = self.forward_speed * self.speed_factor
            self.msg.linear.y = self.sideways_speed * self.speed_factor
            rospy.loginfo("Obstacle on the right! Moving left.")
        elif left_scan < self.distance_laser:
            self.msg.linear.x = self.forward_speed * self.speed_factor
            self.msg.linear.y = -self.sideways_speed * self.speed_factor
            rospy.loginfo("Obstacle on the left! Moving right.")
        else:
            self.msg.linear.x = self.forward_speed * self.speed_factor
            self.msg.linear.y = 0
            self.msg.angular.z = 0
            rospy.loginfo("Path clear. Moving forward.")

    def shutdown(self):
        self.msg.linear.x = 0
        self.msg.linear.y = 0
        self.msg.angular.z = 0
        self.cmd_vel_publisher.publish(self.msg)
        rospy.loginfo("Stopping the robot")

if __name__ == '__main__':
    try:
        robot = rUBotHolonomic()
        robot.start()
    except rospy.ROSInterruptException:
        pass
