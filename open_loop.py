#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from math import pi

class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)
        self.run()


    def run(self):
        vel = Twist()
        vel.linear.x = 0.5
        vel.angular.z = pi-pi
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(80): #4,0
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0
        vel.angular.z = pi/10
        for i in range(52): #90 turn
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0.5
        vel.angular.z = 0
        for i in range(80): #4,4
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0
        vel.angular.z = pi/10
        for i in range(55): #90 turn
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0.5
        vel.angular.z = 0
        for i in range(84): #0,4
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0
        vel.angular.z = pi/10
        for i in range(54): #90 turn
            self.vel_pub.publish(vel)
            self.rate.sleep()
        vel.linear.x = 0.5
        vel.angular.z = 0
        for i in range(68): #0,0
            self.vel_pub.publish(vel)
            self.rate.sleep()
        """   
        vel.linear.x = 0
        vel.angular.z = 0.05
        #while not rospy.is_shutdown():  # uncomment to use while loop
        for i in range(50):
            self.vel_pub.publish(vel)
            self.rate.sleep()
        """

if __name__ == '__main__':
    try:
        whatever = Turtlebot()
    except rospy.ROSInterruptException:
        rospy.loginfo("Action terminated.")
