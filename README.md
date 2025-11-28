# Assignment 1 - The turtlebot

The repository contains the solution of the assignment_1 by group 9.
The goal is to make the robot move to a position between Apriltags and to detect three cylindrical tables placed somewhere in the room.
The project is implemented by five ROS2 packages:
**interfaces_assignment_1**: various confirmation messaged between nodes;
**laser_scan**: detection and identification of the three cylindrical tables;
**my_apriltag_ros**: apriltag detection and publication of their positions;
**my_launch**: launch file to execute the entire project;
**navigation_package**: startup and navigation of the robot until the goal is reached.
