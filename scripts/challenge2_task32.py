#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from gazebo_msgs.msg import ModelState

class RobotObstacle:

    def __init__(self):
        self.move = Twist()
        self.not_first_callback = False
        self.wall_detected = False
        self.forward = False
        self.backward = False
        self.distance_wall_between_callback = 0.0
        self.current_speed = 0.0
        self.wall_spped = 0.0
        self.angular_z = 0.0
        self.last_callback_time = 0.0

        rospy.Subscriber("/gazebo/set_model_state", ModelState, self.model_callback)
        rospy.Subscriber("/scan", LaserScan, self.distance_callback)
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)

    def distance_callback(self, dt):
        if not self.not_first_callback:
            self.last_callback_time = rospy.get_time()
            self.not_first_callback = True

        callback_time = rospy.get_time()
        time_since_last_callback = callback_time - self.last_callback_time

        d = rospy.get_param('distance_of_security') / 100
        lb = 176
        rb = 184

        left_back = dt.ranges[lb]
        right_back = dt.ranges[rb]
        back = dt.ranges[180]
        left = dt.ranges[90]
        right = dt.ranges[270]

        self.move.angular.z = self.angular_z

        if self.not_first_callback:
            distance_wall_between_callback = abs(self.wall_spped * time_since_last_callback)
            delta = abs(back - d)
            velocity_correction_coefficient = delta / time_since_last_callback
            theta = (abs(lb - rb) * (3.1416 / 2) / 90) / time_since_last_callback

            if not self.wall_detected:
                if right > left:
                    self.move.angular.z = -self.angular_z
                else:
                    self.move.angular.z = self.angular_z

            if back > d + 10 and self.wall_detected:
                self.move.linear.x = 0
                self.wall_detected = False
                self.forward = False
                self.backward = False

            if back <= d + distance_wall_between_callback and not self.wall_detected:
                self.wall_detected = True

                if self.wall_spped > 0:
                    self.forward = True
                    self.current_speed = self.wall_spped
                else:
                    self.backward = True
                    self.current_speed = -self.wall_spped

                self.move.linear.x = self.current_speed

            if self.wall_detected:
                if self.forward:
                    if left_back > right_back:
                        self.move.angular.z = theta
                    if left_back < right_back:
                        self.move.angular.z = -theta
                    if back < d and self.wall_spped < 0:
                        self.current_speed = -(self.wall_spped - velocity_correction_coefficient)
                        self.move.linear.x = self.current_speed
                    if back > d and self.wall_spped < 0:
                        self.current_speed = -(self.wall_spped + velocity_correction_coefficient)
                        self.move.linear.x = self.current_speed
                    if back < d and self.wall_spped > 0:
                        self.current_speed = (self.wall_spped) + velocity_correction_coefficient
                        self.move.linear.x = self.current_speed
                    if back > d and self.wall_spped > 0:
                        self.current_speed = (self.wall_spped) - velocity_correction_coefficient
                        self.move.linear.x = self.current_speed

                if self.backward:
                    if left_back > right_back:
                        self.move.angular.z = theta
                    if left_back < right_back:
                        self.move.angular.z = -theta
                    if back < d and self.wall_spped < 0:
                        self.current_speed = -(self.wall_spped - velocity_correction_coefficient)
                        self.move.linear.x = self.current_speed
                    if back > d and self.wall_spped < 0:
                        self.current_speed = -(self.wall_spped + velocity_correction_coefficient)
                        self.move.linear.x = self.current_speed
                    if back < d and self.wall_spped > 0:
                        self.current_speed = (self.wall_spped) + velocity_correction_coefficient
                        self.move.linear.x = self.current_speed
                    if back > d and self.wall_spped > 0:
                        self.current_speed = (self.wall_spped) - velocity_correction_coefficient
                        self.move.linear.x = self.current_speed

                if self.current_speed < 0:
                    self.backward = True
                    self.forward = False
                else:
                    self.backward = False
                    self.forward = True

        self.pub.publish(self.move)
        self.last_callback_time = callback_time
        rospy.loginfo(self.move)

    def model_callback(self, data):
        self.wall_spped = data.twist.linear.x
        self.angular_z = data.pose.orientation.z

def main():
    rospy.init_node("Avoidance_in_all_directions")
    robot_obstacle = RobotObstacle()
    rospy.spin()

if __name__ == '__main__':
    main()

