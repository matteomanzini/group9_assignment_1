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
: defines Ready.msg to advice if the robot is ready to compute the navigation. If the variable `std_msgs/Bool ready` is true, robot can move; false otherwise.

**2. laser_scan**
: technique to detect three cylindrical tables. The node `scan_subcribers` publishes their position by clustering the points of laser scan and filtering them based on a range of acceptable distances.

**3. my_apriltag_ros**
: computes the coordinates and the four corners of two apriltags respect to the external camera. The data are used to compute the final goal pose.

**4. my_launch**
: contains the main lauch file `start_launch.xml`. It launches all single launch files of all the packages.

**5. navigation_package**
: contains lyfecycle_client and navigation_node nodes.
The first one defines the initial pose of robot and notify the navigation_node if robot is ready to move with Ready.msg.
The second one computes the goal position (in the middle of apriltags) and startups the navigation until final goal is reached.
