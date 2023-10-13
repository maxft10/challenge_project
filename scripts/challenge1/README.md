# ROS : Simulated Turtlebor3 Burger Project

## Challenge 1

The first challange consists in making our robot follow trajectories defined by colored lines on the ground and stop at its goal point.
To do that, he have to subscribe to a topic that allows to get images from our robot's camera. Then we process those images with OpenCv to sample only the needed parts and adjust our robot's behavior by sending velocty commands with [geometry_msgs/Twist](https://docs.ros.org/en/diamondback/api/geometry_msgs/html/msg/Twist.html) data type. based on what it perceives.
In most of the tasks, the lines is yellow, sometimes red, and the goal point is green.

- [Task 1](challenge1_task1.py) : Turtlebot has to follow a yellow line on a simulated Gazebo environment
To be able to follow the yellow line, we have to obtain images from robot's camera by susbcribing to a topic named `/camera/image_raw`. Then images are converted into OpenCV images. Futhermore, we perform colors tresholding, masking and we compute centroid coordinates to determine the direction Turtlebot has to move in.
We make robot move by publishing velocity commands through a topic named `/cmd_vel`.
If Turtlebot lost its trajectory tracking, it rotates around itself in order to look for its path. Turtlebot stops when the green goal point is detected.

- [Task 2](challenge1_task2.py) : It is similar to the previous one. The difference is there are red trajectories this time and our robot has to deccelerate when it detecs them. The detection works the same as the previous. We just add a mask and centroid for the red lines.

- [Task 3](challenge1_task3.py) : It is almot identical to task 2. However, at the end of the path, turns are sharp and Turtlebot can lose their tracking easier. To solver this problem we can it rotates around itself, looking for its trajectory. Futhermore after noticing that error, between x axis coordinate of centroid and half the width of the line, is bigger when the corner is difficult, we define intervals and conditions in which robot's speed will be increased or decreased.
