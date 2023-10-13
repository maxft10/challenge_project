#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy                                                                          # Import the ROS Python library
from sensor_msgs.msg import LaserScan                                                 # Import LaserScan message type
from geometry_msgs.msg import Twist                                                   # Import Twist message type
from gazebo_msgs.msg import ModelStates                                               # Import ModelStates message type

class RobotObstacle :

	def __init__(self):
		self.security_distance = rospy.get_param('/distance_of_security')/100			# Set at 20m in corresponding launch file, 20/100=20cm

		self.not_first_callback = False			                                        # It is the first callback
		self.wall_speed = 0				                                	# Variable used to get wall's speed. We initialize to 0.
		self.wall_detected = False			                                        # Wall not detected yet
		self.last_callback_time = 0			                                        # Last time when distance_callback method is called
		self.wall_distance_between_callbacks = 0                                                # Distance crossed by the wall between consecutive callbacks
		
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)                            # Publisher that allows to send velocity commands to the robot
		rospy.Subscriber('/scan', LaserScan, self.distance_callback)   				# Subscriber that reads LaserScan type data from topic named /scan
		rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_callback)              # Subscriber that reads ModelStates type data from topic named /gazebo/model_states
																						# ModelStates contains all the models loaded on Gazebo (ground, wall and robot)
																						# This subscriber allows us to get specific data about those models


	def distance_callback(self, distance):
		front = distance.ranges[0]			                                	# Distance from the robot to the wall, taken from /scan topic
		move = Twist()				                                		# Twist Object to send velocity command to the robot
		callback_time = rospy.get_time()  							# Time when distance_callback method is called
		time_since_last_callback = 0                                                            # Time interval between consecutive callbacks
		velocity_correction_coefficient = 0                                                     # To adjust robot's speed according to distance from the wall

		
		if self.not_first_callback :
			time_since_last_callback = rospy.get_time() - self.last_callback_time
			self.wall_distance_between_callbacks = self.wall_speed * time_since_last_callback	     
			delta = abs(front - self.security_distance)                                    # Gap between front and security distance 
			velocity_correction_coefficient = delta/time_since_last_callback  	       # To adjust robot's speed according to distance from the wall
		

		# To check if it's the first time that the wall is close enough
		if front <= self.security_distance + self.wall_distance_between_callbacks and not(self.wall_detected): 
			self.wall_detected = True                                                      # The wall is now considered as detected
			move.linear.x = -self.wall_speed                                	       # Robot's speed is the opposite of the wall's speed


		# If the wall is detected and moving toward or backward the robot
		if self.wall_detected and (self.wall_speed < 0 or self.wall_speed > 0):
			# We compute the robot's speed depending on its front and distance between consecutive callbacks
			if front < self.security_distance + self.wall_distance_between_callbacks: 
				move.linear.x = -(self.wall_speed + velocity_correction_coefficient)
			if front > self.security_distance:
				move.linear.x = -(self.wall_speed - velocity_correction_coefficient)
			if front < self.security_distance:           
				move.linear.x = -(self.wall_speed + velocity_correction_coefficient)

		move.angular.z = 0                           					       # To set angular velocity to 0, so that it doesn't rotate

		self.pub.publish(move)              						       # To send velocity command to the robot
		self.not_first_callback = True			     				       # It is not the first callback
		self.last_callback_time = callback_time              				       # Changing the value of the variable counting time


		# To display data (can be commented) 
		#"""
		rospy.loginfo("robot_speed = %s; wall_speed = %s; speed_correction = %s", move.linear.x, self.wall_speed, velocity_correction_coefficient)
		rospy.loginfo("distance = %s; distance_of_security = %s; secu+distw = %s", front, self.security_distance, self.security_distance + self.wall_distance_between_callbacks)
		rospy.loginfo("wall_detected = %s; not_first_callback = %s", self.wall_detected, self.not_first_callback)
		rospy.loginfo(" ")
		#"""
		

	def model_callback(self, model):
		#names = model.name
		#rospy.loginfo("List of model names: %s", names)
		self.wall_speed = model.twist[1].linear.x            					# To get speed of model whose index is 1, 
																						# from /gazebo/model_states topic. This is the wall.
																						# Ground index is 0 and robot index is 2.


def main():
	rospy.init_node("Avoidance")                                 				        # Node initialization whose name is "Avoidance"
	robotObstacle = RobotObstacle()                              				        # Creation of a RobotObstacle object
        rospy.Rate(10).sleep()                                                                          # Pauses the program after n executions per second
        rospy.spin()                                                                                    # To keep on executing the program until its shutdown

if __name__ == '__main__':
    main()
		

