# ROS : Simulated Turtlebor3 Burger Project

## Description

This folder contains all the files that describe the Turtlebot 3 robot, its sensors, and its other components:

- [common_properties.xacro](common_properties.xacro) : Sets colors.
- [robot_with_sensors_materials.xacro](robot_with_sensors_materials.xacro) : Contains definitions for additional colors.
- [robot_with_sensors.gazebo](robot_with_sensors.gazebo) : Adds feature elements (camera, lidar, motors) and plugins to the robot_with_sensors in Gazebo.
- [robot_with_sensors.xacro](robot_with_sensors.xacro) : Contains xacro definitions for robot_with_sensors, including its components, links, and joints.
- [turtlebot3_burger.gazebo.xacro](turtlebot3_burger.gazebo.xacro) : Adds feature elements (wheels, caster, camera) and plugins to the Turtlebot3 Burger in Gazebo.
- [turtlebot3_burger.urdf.xacro](turtlebot3_burger.urdf.xacro) : Contains xacro definition of the Turtlebot3 Burger, its parts, links, and joints.
