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
: contains the main lauch file `start_launch.xml`. It launches all single launch files `navigation_launch.xml`, `camera_36h11_launch.xml`, `assignment_1.launch.py` and `laser_launch.xml` with respect to a given order.

**5. navigation_package**
: contains navigation_node and lyfecycle_client nodes.
The first one computes the goal position (in the middle of apriltags) and startups the navigation until final goal is reached.
The second one defines the initial pose of robot and notifies the navigation_node if robot is ready to move with Ready.msg.



# Installation and Running

Open the terminal and follow the instructions below.


### Installation

**1. Create the workspace ROS2**: 

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

**2. Clone the repository**:

```
git clone https://github.com/matteomanzini/group9_assignment_1
```


### Running

**1. Return to the root of workspace**:

```
cd ~/ros2_ws
```

**2. Remove old builds (optional)**:

```
rm -rf build/ install/ log/
```

**3. Compile all packages**:

```
colcon build

```

**4. Generate ROS2 environment**:

```
source install/setup.bash
```

**5. Launch the project**:

```
ros2 launch my_launch start_launch.xml
```
