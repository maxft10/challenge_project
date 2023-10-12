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
        upper_green = np.array([100, 255, 250])
        green_mask = cv2.inRange(hsv_image, lower_green, upper_green)

        # We define range of colors that will be considered red and
        # we create a binary mask
        lower_red = np.array([0,200,255])
        upper_red = np.array([40,255,255])
        red_mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # We create masks with the same as size as mask of yellow, mask of red and mask of green color
        mask_y = np.zeros_like(yellow_mask)
        mask_g = np.zeros_like(green_mask)
        mask_r = np.zeros_like(red_mask)

        # Pixels of the search areas become whiite
        mask_y[search_top:search_bot, :] = 255
        mask_g[search_top:search_bot, :] = 255
        mask_r[search_top:search_bot, :] = 255

        yellow_mask = cv2.bitwise_and(yellow_mask, mask_y)
        red_mask = cv2.bitwise_and(red_mask, mask_r)
        green_mask = cv2.bitwise_and(green_mask, mask_g)

        # Moments computation of yellow, red and green areas
        M_yellow = cv2.moments(yellow_mask)
        M_red = cv2.moments(red_mask)
        M_green = cv2.moments(green_mask)

        velocity_cmd = Twist()                                                          # We create a Twist() objet to send velocity command to Turtlebot

        # If yellow line is detected, Turtlebot follows it
        if M_yellow['m00'] > 0 :       
            rospy.loginfo("Turtlebot on yellow line")
            # We compute of centroid coordinates of yellow image 
            yellow_cx = int(M_yellow['m10']/M_yellow['m00'])        
            yellow_cy = int(M_yellow['m01']/M_yellow['m00'])
            cv2.circle(image, (yellow_cx, yellow_cy), 20, (0,0,255), -1)                # Draw a circle at centroid position
            rospy.loginfo("yellow_cx = %s and yellow_cy = %s ",yellow_cx,yellow_cy)

            yellow_error = float(yellow_cx - width/2)                                   #Error between centroid position and center of image
            velocity_cmd.linear.x = 0.4
            rospy.loginfo("yellow_error = %s", yellow_error)

            # if abs(yellow_error)<100:
            #     velocity_cmd.linear.x = 0.3
            # elif abs(yellow_error)>=100:
            #     velocity_cmd.linear.x = 0.1

            if abs(yellow_error)>=10 and abs(yellow_error)<=20:
                velocity_cmd.linear.x = 0.3
            elif abs(yellow_error)>=80:
                velocity_cmd.linear.x = 0.1

            velocity_cmd.angular.z = -yellow_error/100

        # If red line is detected, Turtlebot follows it with a different velocity
        elif M_red['m00'] > 0 :        
            rospy.loginfo("Turtlebot on red line")
            # We compute of centroid coordinates of yellow image 
            red_cx = int(M_red['m10']/M_red['m00'])                    
            red_cy = int(M_red['m01']/M_red['m00'])
            cv2.circle(image, (red_cx, red_cy), 20, (255,0,0), -1)                      # Draw a circle at centroid position

            red_error = red_cx - width/2                                                # Error between centroid position and center of image
            velocity_cmd.linear.x = 0.3
            velocity_cmd.angular.z = -red_error/100

        # If green mark is detected, Turtlebot stops
        elif M_green['m00'] > 0:                                    
            rospy.loginfo("Turtlebot on green circle. It stops")
            velocity_cmd.linear.x = 0
            velocity_cmd.angular.z = 0

        # If no color is detected, Turtlebot stops and rotates to find a color
        else :
            rospy.loginfo("Turtlebot finds no color. It rotates, looking for one")                                   
            velocity_cmd.linear.x = 0
            velocity_cmd.angular.z = -1

        self.pub.publish(velocity_cmd)                                                  # Send velocity command to Turtlebot
        rospy.loginfo(velocity_cmd)
        rospy.loginfo("M_yellow['m00'] = %s", M_yellow['m00'])
        rospy.loginfo("M_red['m00'] = %s", M_red['m00'])

        #Show images
        cv2.imshow("Image from camera", image)
        cv2.imshow("window yellow", yellow_mask)
        cv2.imshow("window red", red_mask)
        cv2.waitKey(2)

def main():
    rospy.init_node('Line_Follower_Sharp_Turns')                                        # We initialize a node name Line_Follower_Sharp_Turns
    follower = LineFollower()          	                                                # We create a LineFollower object
    rospy.Rate(5).sleep() 
    rospy.spin()                                                                        # To keep on executing the program until its shutdown

if __name__ == '__main__':
    main()



