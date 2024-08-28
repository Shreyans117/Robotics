#!/usr/bin/env python

from math import pi, sqrt, atan2, cos, sin
import numpy as np

import rospy
import tf
from std_msgs.msg import Empty
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist, Pose2D


class Turtlebot():
    def __init__(self):
        rospy.init_node("turtlebot_move")
        rospy.loginfo("Press Ctrl + C to terminate")
        self.vel_pub = rospy.Publisher("cmd_vel_mux/input/navi", Twist, queue_size=10)
        self.rate = rospy.Rate(10)

        # reset odometry to zero
        self.reset_pub = rospy.Publisher("mobile_base/commands/reset_odometry", Empty, queue_size=10)
        for i in range(10):
            self.reset_pub.publish(Empty())
            self.rate.sleep()
        
        # subscribe to odometry
        self.pose = Pose2D()
        self.logging_counter = 0
        self.trajectory = list()
        self.odom_sub = rospy.Subscriber("odom", Odometry, self.odom_callback)

        # Initial conditions
        self.previous_point = [0, 0]
        self.previous_velocity = [0, 0]
        self.vel_steering = 0.5
        self.vel = Twist()

        try:
            self.run()
        except rospy.ROSInterruptException:
            rospy.loginfo("Action terminated.")
        finally:
            # save trajectory into csv file
            np.savetxt('trajectory.csv', np.array(self.trajectory), fmt='%f', delimiter=',')


    def run(self):
        waypoints = [[0.5, 0], [0.5, -0.5] , [1, -0.5], [1, 0], [1, 0.5], \
                       [1.5, 0.5], [1.5, 0], [1.5, -0.5], [1, -0.5], [1, 0],\
                       [1, 0.5], [0.5, 0.5], [0.5, 0], [0, 0], [0, 0]]
        for i in range(len(waypoints)-1):
            self.move_to_point(waypoints[i], waypoints[i+1])


    def move_to_point(self, current_waypoint, next_waypoint):
        # generate polynomial trajectory and move to current_waypoint
        # next_waypoint is to help determine the velocity to pass current_waypoint
        
        # Set boundary conditions for position
        # Start from the previous point and end up at the current waypoint
        p_start = self.previous_point
        p_end = current_waypoint

        # Set boundary conditions for velocity
        v_start = self.previous_velocity
        dev = [(next_waypoint[i] - self.previous_point[i]) for i in range(2)]
        angle = atan2(dev[1], dev[0]);
        v_end = [self.vel_steering * cos(angle), self.vel_steering * sin(angle)]

        # Need to compute good value for T
        T = 5

        # Compute coefficient for X
        COEFF_X = self.polynomial_time_scaling_3rd_order(p_start[0], v_start[0], p_end[0], v_end[0], T)

        # Compute coefficient for y
        COEFF_Y = self.polynomial_time_scaling_3rd_order(p_start[1], v_start[1], p_end[1], v_end[1], T)

        # Supply velocity in small segments
        c = 10
        for i in range(c * T):
            t = 0.1 * i

            # Linear velocity
            # Compute v_end_x using COEFF_X, apply velocity equation
            # 
            v_end_x = (np.matrix([3 * (t ** 2), 2 * t, 1, 0]) * COEFF_X).item(0)

            # Compute v_end_y using COEFF_Y, apply velocity equation
            v_end_y = (np.matrix([3 * (t ** 2), 2 * t, 1, 0]) * COEFF_Y).item(0)

            rospy.loginfo("V_X: {0}, V_Y: {1}".format(v_end_x, v_end_y))

            # Create linear velocity
            self.vel.linear.x = sqrt((v_end_x ** 2) + (v_end_y ** 2))

            # Angular velocity
            # Compute deviation
            theta_dev = atan2(v_end_y, v_end_x)

            # Compute error
            error = theta_dev - self.pose.theta
            if error < -pi:
                error = error + (2 * pi)
            elif error > pi:
                error = error - (2 * pi)

            # Set Kp
            Kp = 5.0

            # Calculate angular velocity
            self.vel.angular.z = Kp * error

            # Publish velocities
            self.vel_pub.publish(self.vel)

            # Sleep after publichsing velocity
            self.rate.sleep()

        self.previous_point = current_waypoint
        self.previous_velocity = [v_end_x, v_end_y]


    def polynomial_time_scaling_3rd_order(self, p_start, v_start, p_end, v_end, T):
        # input: p,v: position and velocity of start/end point
        #        T: the desired time to complete this segment of trajectory (in second)
        # output: the coefficients of this polynomial
        A = np.matrix([[p_start], 
                      [v_start],
                      [p_end],
                      [v_end]])
        X = np.matrix([[0, 0, 0, 1],
                     [0, 0, 1, 0],
                     [T ** 3, T ** 2, T, 1],
                     [3 * (T ** 2), 2 * T, 1, 0]])

        X_INV = np.linalg.inv(X)
        B = X_INV * A

        return B


    def odom_callback(self, msg):
        # get pose = (x, y, theta) from odometry topic
        quarternion = [msg.pose.pose.orientation.x, msg.pose.pose.orientation.y,\
                    msg.pose.pose.orientation.z, msg.pose.pose.orientation.w]
        (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(quarternion)
        self.pose.theta = yaw
        self.pose.x = msg.pose.pose.position.x
        self.pose.y = msg.pose.pose.position.y

        # logging once every 100 times (Gazebo runs at 1000Hz; we save it at 10Hz)
        self.logging_counter += 1
        if self.logging_counter == 100:
            self.logging_counter = 0
            self.trajectory.append([self.pose.x, self.pose.y])  # save trajectory
            rospy.loginfo("odom: x=" + str(self.pose.x) +\
                ";  y=" + str(self.pose.y) + ";  theta=" + str(yaw))


if __name__ == '__main__':
    whatever = Turtlebot()