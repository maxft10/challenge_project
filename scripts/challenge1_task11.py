#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Twist
import math

class RotateRobot:
    def __init__(self):
        #rospy.init_node('Line_Follower')  
        self.rotate_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(10)  # Fréquence d'exécution du script

    def rotate(self, angle):
        twist_msg = Twist()
        angular_speed = 1.0  # Vitesse angulaire (ajustez selon votre besoin)
        twist_msg.angular.z = angular_speed if angle > 0 else -angular_speed  # Détermine la direction de la rotation
        rospy.loginfo('angular = %s', angular_speed)
        # Calculer le temps nécessaire pour atteindre l'angle spécifié
        duration = abs(angle / angular_speed)

        t0 = rospy.Time.now().to_sec()
        current_time = t0
        rospy.loginfo('time = %s', current_time)

        while (current_time - t0) < duration and not rospy.is_shutdown():
            self.rotate_pub.publish(twist_msg)
            current_time = rospy.Time.now().to_sec()
            rospy.loginfo('speed = %s', twist_msg)
            self.rate.sleep()

        twist_msg.angular.z = 0  # Arrêter la rotation
        self.rotate_pub.publish(twist_msg)
        rospy.loginfo('stop_speed = %s', twist_msg)


def main():
    rospy.init_node('Line_Follower')  #We initialize a node name Line_Follower
    follower = RotateRobot()         #We create a LineFollower object
    follower.rotate(math.pi / 4)
    follower.rate.sleep()
    rospy.spin()

if __name__ == '__main__':
    main()
    


