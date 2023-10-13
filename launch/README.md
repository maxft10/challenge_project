# ROS : Simulated Turtlebor3 Burger Project

## Description

This folder contains launches that enable the execution of our ROS nodes and the spawning of our robot in a simulated environment using Gazebo. Most launch files are associated with specific tasks:

- [gazebo.launch](gazebo.launch) : Launches the Gazebo simulator with the correct parameters in an empty environment.
- [rviz.launch](rviz.launch) : Launches the Rviz visualizer with its parameters and robot description.
- **challenge(i)_task(j).launch** : Files used to launch different scripts and environments.

      Where:
      - `i` represents the challenge.
      - `j` represents the task.

For example, [challenge2_task3.launch](challenge2_task3.launch) is associated with task 3 from challenge 2 and allows the execution of [challenge2_task3.py](../scripts/challenge2/challenge2_task3.py), along with its associated environment and Gazebo.
