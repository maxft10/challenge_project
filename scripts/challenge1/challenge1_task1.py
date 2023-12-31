#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy, std_msgs, cv2                       # For interfacing ROS and Python, and processing camera images with OpenCV
from geometry_msgs.msg import Twist               # Use Twist messages to control robot speed
from sensor_msgs.msg import Image                 # Use Image messages to capture images from camera
import numpy as np
from cv_bridge import CvBridge , CvBridgeError    # To convert ROS images into OpenCv images

class LineFollower:                               # We define a class to follow the line

    def __init__(self):
        self.pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)                    # Publisher that allows to send velocity commands to the robot
        self.image = rospy.Subscriber('/camera/image_raw', Image, self.image_callback)  # A subscriber that received images from image_raw topic
        self.bridge = CvBridge()                                                        # A CvBridge object to convert images

    def image_callback(self, image_camera):                                             # This function is called everytime images from camera are received

        try:
            image = self.bridge.imgmsg_to_cv2(image_camera, "bgr8")                     # Image conversion
        except CvBridgeError as e:
            print(e) #print the error

        height, width, channels = image.shape                                           # We get image dimension
        search_top = int(3*height/4)                                                    # We define top and bottom of the search area of the line to follow
        search_bot = int(3*height/4 + 20)

        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)                              # Conversion from bgr image to HSV image

        # We define range of colors that will be considered yellow and
        # we create a binary mask
        lower_yellow = np.array([ 20, 100, 100])
        upper_yellow = np.array([50, 255, 255])
        yellow_mask = cv2.inRange(hsv_image, lower_yellow, upper_yellow)

        # We define range of colors that will be considered green and
        # we create a binary mask
        lower_green = np.array([ 20, 10, 10])
        upper_green = np.array([250, 255, 250])
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # We create masks with the same as size as mask of yellow and mask of green color
        mask_y = np.zeros_like(yellow_mask)
        mask_g = np.zeros_like(green_mask)

        # Pixels of the search areas become white
        mask_y[search_top:search_bot, :] = 255
        mask_g[search_top:search_bot, :] = 255

        # yellow_mask = cv2.bitwise_and(yellow_mask, mask_y)
        # green_mask = cv2.bitwise_and(green_mask, mask_g)

        # Moments computation of yellow and green areas
        M_yellow = cv2.moments(yellow_mask)
        M_green = cv2.moments(green_mask)

        velocity_cmd = Twist()                                                          # We create a Twist() objet to send velocity command to Turtlebot

        # If yellow line is detected, Turtlebot follows it
        if M_yellow['m00'] > 0:  
            # We compute of centroid coordinates of yellow image               
            yellow_cx = int(M_yellow['m10']/M_yellow['m00'])        
            yellow_cy = int(M_yellow['m01']/M_yellow['m00'])
            cv2.circle(image, (yellow_cx, yellow_cy), 20, (0,0,255), -1)                # Draw a circle at centroid position

            error = yellow_cx - width/2                                                 # Error beetween centroid position and center of image
            velocity_cmd.linear.x = 0.8
            velocity_cmd.angular.z = -error/100

        # Else if green mark is detected, Turtlebot stops
        elif M_green['m00'] > 0:                                                        
            velocity_cmd.linear.x = 0
            velocity_cmd.angular.z = 0

        # If no color is detected, Turtlebot stops and rotates to find a color
        else :                                    
           velocity_cmd.linear.x = 0
           velocity_cmd.angular.z = -1

        self.pub.publish(velocity_cmd)                                                  # Send velocity command to Turtlebot

        # Show images
        cv2.imshow("Image from camera", image)
	    # cv2.imshow("window", yellow_mask)
	    # cv2.imshow("window1", green_mask)
        cv2.waitKey(2)

def main():
    rospy.init_node('Line_Follower')                                                    # We initialize a node named "Line_Follower"
    follower = LineFollower()                                                           # We create a LineFollower object
    rospy.Rate(10).sleep()                                                              # Pauses the program after 10 executions per second
    rospy.spin()                                                                        # To keep on executing the program until its shutdown

if __name__ == '__main__':
    main()



