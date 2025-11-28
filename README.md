# Assignment 1 - The turtlebot

The repository contains the solution of the assignment_1 by group 9.
The goal is to control the Turtlebot in the room simulated in 2D by RViz2 and in 3D by Gazebo GUI.
There are three important phases:

1- Detect the AprilTags in the enviroment;

2- Navigate in the room to reach the final point;

3- Localize three cylindrical tables.


# Project Structure

The project is implemented by five ROS2 packages:

**1. interfaces_assignment_1**

It is defined by Ready.msg to advice if the robot is ready to compute the navigation.

**2. laser_scan**

**3. my_apriltag_ros**

**4. my_launch**

**5. navigation_package**

The package contains two nodes: lyfecycle_client and navigation_node.
The first one defines the initial pose of robot and notify the navigation_node if robot is ready to move with Ready.msg.
The second one computes the goal position (in the middle of apriltags) and startups the navigation until final goal is reached.
