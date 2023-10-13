#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, std_msgs
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class RobotObstacle :
	
	def __init__(self):
		self.stop_distance = 0.5                                                        # Stop distance set to 0.5m

		self.laser_sub = rospy.Subscriber('/scan', LaserScan, self.distance_callback)   # Subscriber that reads LaserScan type data from topic named /scan
		self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)  				    # Publisher that allows to send velocity commands to the robot
		

	def distance_callback(self, distance):
		front = distance.ranges[0]                                                      # Distance from the robot to the wall, taken from /scan topic
		move = Twist()                                                                  # Twist Object to send velocity command to the robot


		# We set velocity based on distance
		if front <= self.stop_distance + 0.2:
			move.linear.x = 0
		elif front > self.stop_distance:
			move.linear.x = 0.4

		self.pub.publish(move)                                                   		# To send velocity command to the robot

		# To display data
		#"""
		rospy.loginfo("robot speed = %s", move.linear.x)
		rospy.loginfo("distance = %s; stop_distance = %s",front, self.stop_distance)
		#"""


def main():
	rospy.init_node("Emergency_Stop")              # Node initialization whose name is Emergency_Stop
	robotObstacle = RobotObstacle()                # Creation of a RobotObstacle object
	rospy.Rate(10).sleep() 
	rospy.spin()

if __name__ == '__main__':
    main()
		
		
		
		
		
		
		
		
		
		




