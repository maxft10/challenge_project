#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelStates

class RobotObstacle:

    def __init__(self):
        self.move = Twist()                                                             # Twist object that allows the robot to move
        self.not_first_callback = False                                                 # It is the first callback
        self.wall_speed = 0.0                                                           # Variable used to get wall's speed. We initialize it to 0
        self.wall_orientation = 0.0                                                     # Variable used to get wall's orientation. We initialize it to 0
        self.wall_detected = False                                                      # Wall not detected yet
        self.last_callback_time = 0.0                                                   # Time when distance_callback method is called
        self.wall_distance_between_callbacks = 0.0                                      # Distance crossed by the wall between consecutive callbacks

        self.forward = False
        self.backward = False

        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)                    # Publisher that allows to send velocity commands to the robot
        rospy.Subscriber("/scan", LaserScan, self.distance_callback)                    # Subscriber that reads LaserScan type data from topic named /scan
        rospy.Subscriber("/gazebo/model_states", ModelStates, self.model_callback)      # Subscriber that reads ModelState type data from topic named /gazebo/model_states
											                                            # ModelStates contains all the models loaded on Gazebo (ground, wall and robot)
                                                                                        # This subscriber allows us to get specific data about those models

        
    def distance_callback(self, dt):

        callback_time = rospy.get_time()                                                # Time when distance_callback method is called

        security_distance = rospy.get_param('distance_of_security') / 100               # Set at 20m in corresponding launch file, 20/100=20cm
        lb = 175                                                                        # Index of left back angle
        rb = 185                                                                        # Index of right back angle

        left_back = dt.ranges[lb]                                                       # Distance from the wall to the left back side of the robot, taken from /scan topic
        right_back = dt.ranges[rb]                                                      # Distance from the wall to the right back side of the robot, taken from /scan topic
        back = dt.ranges[180]                                                           # Distance from the wall to the back side of the robot, taken from /scan topic
        left = dt.ranges[90]                                                            # Distance from the wall to the left side of the robot, taken from /scan topic
        right = dt.ranges[270]                                                          # Distance from the wall to the right side of the robot, taken from /scan topic

        self.move.angular.z = self.wall_orientation                                     # To set angular velocity

        if self.not_first_callback:
            time_since_last_callback = rospy.get_time() - self.last_callback_time       # Time interval between consecutive callbacks
            self.wall_distance_between_callbacks = abs(self.wall_speed * time_since_last_callback)
            delta = abs(back - security_distance)
            velocity_correction_coefficient = delta / time_since_last_callback          # To adjust robot's speed according to distance from the wall
            theta = (abs(lb - rb) * (3.1416/2) /90) / time_since_last_callback          # Computing the angular velocity difference in order to adjust the robot's orientation

        if not self.wall_detected:
            if right > left:
                self.move.angular.z = -self.wall_orientation
            elif right < left:
                self.move.angular.z = self.wall_orientation

        # If the wall is too far but detected
        if back > security_distance + 10 and self.wall_detected:
            self.move.linear.x = 0
            self.wall_detected = False
            self.forward = False
            self.backward = False

        # If it's the first time that the wall is close enough
        if back <= security_distance + self.wall_distance_between_callbacks and not self.wall_detected:
            self.wall_detected = True                                                   # The wall is now detected

            # The robot move is the same direction as the wall
            if self.wall_speed > 0:
                self.forward = True
                self.move.linear.x = self.wall_speed
            else:
                self.backward = True
                self.move.linear.x = -self.wall_speed

        # If the wall is detected
        if self.wall_detected:
            if self.forward:
                if left_back > right_back:
                    self.move.angular.z = theta
                elif left_back < right_back:
                    self.move.angular.z = -theta

                if back < security_distance and self.wall_speed < 0:
                    self.move.linear.x = -(self.wall_speed - velocity_correction_coefficient)

                if back > security_distance and self.wall_speed < 0:
                    self.move.linear.x = -(self.wall_speed + velocity_correction_coefficient)

                if back < security_distance and self.wall_speed > 0:
                    self.move.linear.x = (self.wall_speed) + velocity_correction_coefficient

                if back > security_distance and self.wall_speed > 0:
                    self.move.linear.x = (self.wall_speed) - velocity_correction_coefficient

            if self.backward:
                if left_back > right_back:
                    self.move.angular.z = theta
                elif left_back < right_back:
                    self.move.angular.z = -theta

                if back < security_distance and self.wall_speed < 0:
                    self.move.linear.x = -(self.wall_speed - velocity_correction_coefficient)

                if back > security_distance and self.wall_speed < 0:
                    self.move.linear.x = -(self.wall_speed + velocity_correction_coefficient)

                if back < security_distance and self.wall_speed > 0:
                    self.move.linear.x = (self.wall_speed) + velocity_correction_coefficient

                if back > security_distance and self.wall_speed > 0:
                    self.move.linear.x = (self.wall_speed) - velocity_correction_coefficient

            if self.move.linear.x < 0:
                self.backward = True
                self.forward = False
            else:
                self.backward = False
                self.forward = True

        self.pub.publish(self.move)                                                     # To send velocity command to the robot
        self.last_callback_time = callback_time                                         # Changing the value of the variable counting time
        self.not_first_callback = True                                                  # It is not the first callback
        rospy.loginfo(self.move)                                                        # To display robot's velocity

    def model_callback(self, model):
        self.wall_speed = model.twist[1].linear.x                                       # We get wall's speed
        self.wall_orientation = model.pose[1].orientation.z                             # We get wall's orientation

def main():
    rospy.init_node("Avoidance_in_all_directions")                                      # Node initialization whose name is "Avoidance_in_all_directions"
    robot_obstacle = RobotObstacle()                                                    # Creation of a RobotObstacle object
    rospy.Rate(10).sleep() 
    rospy.spin()

if __name__ == '__main__':
    main()

