


This a project which aims at controlling a Turtlebot robot on different environments. The robot has to navigate from one starting point to a reaching point or not. To do that, it needs to exploit data from its sensors and adopt the optimal behavior based on them.

In this project there are several challenges divided in tasks which increases in difficulties.
In the first challenge, Turtlebot needs to use images obtained from its camera in order to follow a line and change its movement speed based on the color perceived.
In the second challenge, it needs to use data from its LIDAR in order to detect a wall moving in different directions and maintain a specific distance between it and the wall.
The third challenge is a mix of the previous ones : Turtlebot has to navigate in more complex environment, follow lines and avoid collisions.


## Organization of our work

All our work is contained within a ROS package named challenge_project. It containes files we have implemented and files we haven't. Those files are meant to describe Turtlebot and its sensors, the environments in which it will interact, and the ones that enable to run it and their environments with Gazebo or Rviz. We have modified those last files in order to also run the ones we have implemented.

There 3 differant challenges and 3 tasks for each one.
The names of our scripts respect the following structure : 
challenge(i)_task(j).py
i represent the challenge
j represents the task 

For example, challenge2_task3.py is a script associated with task 3 from challenge 2.

Our launch files follow a similar structure :
challenge(i)_task(j).launch

For example challenge2_task3.launch is associated with task 3 from challenge 2 and allows to run challenge2_task3.py, the environment associated with and Gazebo.


To be more precise our package is organized as such :

-launch folder : it contains launch files that enable to run several executables or other files at the same time
challenge(i)_task(j).launch : all these files that are used to launch the different codes and environments
gazebo.launch : to launch gazebo simulator with the good parameters 
rviz.launch : to launch rviz visualizer with its parameters and robot description

-scripts folder : a folder which contains our codes which are meant to specific tasks. They are separated within 3 other folders based on the challenges they are related to.
challenge(i)_task(j).py are files that carry tasks out. As said previously, there is a file for each task.
challenge(i)_task(j)_world_control.py are files that make some elements interact within our worlds during simulation. For example, 
challenge2_task2_world_control.py is used for task2 from challenge 2 and makes the wall move forward and backward.

-rviz folder contains configuration.rviz which saves all the features from rviz configuration

-urdf folder : a folder with the description of Turtlebot and its sensors
common_properties.xacro : Set the initial colors
● robot_with_sensors.gazebo : Add elements features (camera, lidar, motors) and plugins in gazebo to the robot_with_sensors
● robot_with_sensors.xacro : Contains xacro definition of the robot_with_sensors, its part, links and joints
● robot_with_sensors_materials.xacro : Contains other colors definition
● turtlebot3_burger.gazebo.xacro : Add elements features (wheels,caster,camera) and plugins in gazebo to the turtlebot3_burger
● turtlebot3_burger.urdf.xacro : Contains xacro definition of the turtlebot3_burger, its part, links and joints

- worlds folder : a folder with the different worlds for our project for gazebo usage. Each world file is associated with a challenge and a task.

Our ROS package depends on 
std_msgs a ROS package that provides standardized type of messages allowing ROS nodes to communicate.
turtlebot3_msgs a package that contains specific types of messages to Turtlebot
cv_bridge a package that allows to convert ROS images into OpenCV images and process them.



## ROS concepts

For this project we've used ROS (Robot Operating System) which is framework meant to robotics application. ROS contains all necessary elements to develop software for robots. If you aren't familiar with ROS and want to know more about it, click on the link below :



In fact, our scripts are considered as nodes in ROS. A node is an executable designed for one task. A node can interact with other nodes thanks to topics and services.

A topic is channel used by a node to communicate with other nodes. A node can send data, through a topic, which are received by another. A node sending data is called "publisher" while a node receiving data is called "subscriber".

Services are other types of channels used by nodes to exchange data. Differently from a topic, a node can use a service only to communicate with only one node. As a client, it sends a request through a service and receives a response back. While a as server, it receives a request and sends a response.

The master is the key element in a ROS ecosystem. It registers actives nodes, manages communications through topics and services and keep lists of availables ones. It also enables to manage parameters that can be used by nodes.


If you want to know more about ROS and its concepts, click on the link below :


## Our work

Our work has consisted in writing nodes that subscribe to topics in order to get data from our robot's sensors, publish data through another topic to send velocity commands to this robot in order to make it moves accordingly to what it perceives.

3.1. Challenge 1

The first challange consists in making our robot follow trajectories defined by colored lines on the ground and stop at its goal point.
To do that, he have to subscribe to a topic that allows to get images from our robot's camera. Then we process those images with OpenCv to sample only the need parts and adjust our robot's behavior based on what it perceives.
In most of the tasks, the lines is yellow, sometimes red, and the goal point is green.

Task 1 : 
In this task, Turtlebot has to follow a yellow line on a simulated Gazebo environment
To be able to follow the yellow line, we have to obtain images from robot's camera by susbcribing to a topic named /camera/image_raw. Then images are converted into OpenCV images. Futhermore, we perform colors tresholding, masking and we compute centroid coordinates to determine the direction Turtlebot has to move in.
We make robot move by publishing velocity commands through a topic named /cmd_vel.
If Turtlebot lost its trajectory tracking, it rotates around itself in order to look for its path. Turtlebot stops when the green goal point is detected.

Task 2 :
This taks is similar to the previous one. The difference is there are red trajectories this time and our robot has to deccelerate when it detecs them. The detection works the same as the previous. We just add a mask and centroid for the red lines.

Task 3 :
This task is almot identical to task 2. However, at the of the path, turns are sharp and Turtlebot can lose their tracking easier. To solver this problem we can it rotates around itself, looking for its trajectory. Futhermore after noticing that error, between x axis coordinate of centroid and half the width of the line, is bigger when the corner is difficult, we define intervals and conditions in which robot's speed will be increased or decreased.



3.2. Challenge 2

This challenge consists in making the robot avoid a wall that can be immobile or moving toward it. We still use the same /cmd_vel topic to send velocty commands to the robot. And to detect the wall, we subscribe to topic which provides us data from our robot's LIDAR. We also subscribe to another topic to get wall's speed.

Task 1 :
This task is simple : the robot just needs to stop when it reaches a certain position from the wall. To do that, we subscribe to a topic named /scan and we obtain data from robot's LIDAR. In this case, data are from sensor_msgs/LaserScan type. 
 
We are interested in the field that give us the distance between a robot and an obstacle in all directions. This field is ranges and is an array that contains the range measurements at different angles.
To know the distance in front of Turtlebot, we use the first element of ranges : ranges[0]. When the robot is close enough to the wall (stop distance specified by us), it stops.

Task 2 :
In this task, the wall is moving forward or backward the robot in the same direction and we have to make it maintains a safety distance with the wall.
To do that, we subscribe to the same /scan topic and use ranges[0] to get the distance between the robot and the wall in front of it. We also use /gazebo/model_states topic to get information about the wall, and espcially its velocity. Then we implement an algorithm which allows to keep that safety distance and make it move thank to /cmd_vel topic.

Task 3 : 
The wall moves in all directions and the robot has to maintain a safety distance. Futhermore while in the previous task, the wall is perpendicular to the robot, in this one, it isn't. This time, we also need ranges[90], ranges[180], ranges[270], ranges[175], ranges[185] to get informations respectively about the left, back, right, left back and right back sides of the robot. We use /gazebo/model_states topic to get wall's speed and orientation and make the robot move with /cmd_vel topic. Robot’s angular velocity around z axis is corrected in order to put its back in front of the moving wall when the wall is at the desired distance, or when the wall isn't well oriented.




