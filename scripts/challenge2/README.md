# ROS : Simulated Turtlebor3 Burger Project

## Challenge 2

This second challenge consists in making the robot avoid a wall that can be immobile or moving toward it. We still use the same **/cmd_vel** topic to send velocty commands to the robot with [geometry_msgs/Twist](https://docs.ros.org/en/diamondback/api/geometry_msgs/html/msg/Twist.html) data type. And to detect the wall, we subscribe to topic which provides us data from our robot's LIDAR. We also subscribe to another topic to get wall's speed.

- [Task 1](challenge2_task1.py) : The problem is simple. The robot just needs to stop when it reaches a certain position from the wall. To do that, we subscribe to a topic named **/scan** and we obtain data from robot's LIDAR. In this case, data are from [sensor_msgs/LaserScan](http://docs.ros.org/en/melodic/api/sensor_msgs/html/msg/LaserScan.html) type. 
We are interested in the field that give us the distance between a robot and an obstacle in all directions. This field is ranges and is an array that contains the range measurements at different angles.
To know the distance in front of Turtlebot, we use the first element of ranges : ranges[0]. When the robot is close enough to the wall (stop distance specified by us), it stops.

- [Task 2](challenge2_task2.py) : The wall is moving forward or backward the robot in the same direction and we have to make it maintains a safety distance with the wall.
To do that, we subscribe to the same **/scan** topic and use ranges[0] to get the distance between the robot and the wall in front of it. We also use **/gazebo/model_states** topic to get information about the wall, and espcially its velocity. Then we implement an algorithm which allows to keep that safety distance and make it move thank to **/cmd_vel** topic.

- [Task 3](challenge2_task3.py) : The wall moves in all directions and the robot has to maintain a safety distance. Futhermore while in the previous task, the wall is perpendicular to the robot, in this one, it isn't. This time, we also need ranges[90], ranges[180], ranges[270], ranges[175], ranges[185] to get informations respectively about the left, back, right, left back and right back sides of the robot. We use **/gazebo/model_states** topic to get wall's speed and orientation and make the robot move with **/cmd_vel** topic. Robot’s angular velocity around z axis is corrected in order to put its back in front of the moving wall when the wall is at the desired distance, or when the wall isn't well oriented.