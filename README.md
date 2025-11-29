# Assignment 1 - The turtlebot

The repository contains the solution of the assignment_1 by group 9.
The project implements the navigation and object detection for a robot in the simulated environment, using *RViz2* for 2D visualization and *Gazebo GUI* for 3D simulation.

### Overview

The system allows the robot to:

1- Detect the AprilTags in the enviroment using the external camera;

2- Navigate in the room to reach the computed final position;

3- Localize three cylindrical tables using laser scan data.



# Project Structure

The project is implemented by five ROS2 packages:

**1. interfaces_assignment_1**

Defines the custom service interfaces:

*1. Ready.srv*: controls if the robot is ready to compute the navigation. The variable `std_msgs/Bool ready` is `true` if robot can move, `false` otherwise.

*2. Goalresult.srv*: checks the goal status with `std_msgs/Bool goal_status` and the goal position using `geometry_msgs/PoseStamped goal_pose`.

**2. laser_scan**
: technique to detect three cylindrical tables is by clustering the points of laser scan and filtering them based on a range of acceptable distances.

The node `scan_subcribers` publishes detected table positions.

**3. my_apriltag_ros**
: computes the coordinates of center and the four corners of two AprilTags relative to the external camera. Provides data to the navigation_node in **navigation_package** tocompute the final goal position.


**4. my_launch**
: contains the main launch file `start_launch.xml` for system startup. It launches all single launch files:

*1. `navigation_launch.xml`*;

*2. `camera_36h11_launch.xml`*;

*3. `assignment_1.launch.py`*;

*4. `laser_launch.xml`*.

**5. navigation_package**
: implements the logic of navigation inside the simulated environment.

It contains two nodes: 

*- navigation_node* computes the goal position (in the middle of apriltags) and startups the navigation until final goal is reached.

Performs coordinate transformation in map frame to act the navigation in RViz2.

*- lifecycle_client* sets the initial pose of robot and notifies the navigation_node if robot is ready to move with Ready.srv.



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

at the end of your .bashrc add:

```
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
```

**5. Launch the project**:

```
ros2 launch my_launch start_launch.xml
```
