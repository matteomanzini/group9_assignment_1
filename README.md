# Assignment 1 - The turtlebot

The repository contains the solution of the assignment_1 by group 9.

The project implements the navigation and object detection of the turtlebot in the simulated environment, using *RViz2* for 2D visualization and *Gazebo GUI* for 3D simulation.

### Overview

The system allows the robot to:

**• Detect the AprilTags** in the enviroment using the external camera;

**• Navigate in the room** to reach the computed final position;

**• Localize three cylindrical** tables using laser scan data.



# Project Structure

The project is implemented by five ROS2 packages:

### **• interfaces_assignment_1**

Defines the custom service interfaces:

*• `Ready.srv`* : controls if the robot is ready to compute the navigation. The response `std_msgs/Bool ready` is `true` if robot can move, `false` otherwise.

*• `Goalresult.srv`* : checks the final goal status and final position of the robot respectively with the `std_msgs/Bool goal_status` and `geometry_msgs/PoseStamped goal_pose` responses.

### **• laser_scan**
Technique to detect three cylindrical tables by clustering the points of laser scan and filtering them based on a range of acceptable distances.

The node `scan_subcribers` publishes detected table positions.

### **• my_apriltag_ros**
Computes the coordinates of center and the four corners of two AprilTags relative to the external camera. 

Provides data to the `navigation_node` in **navigation_package** to compute the final goal position.


### **• my_launch**
Contains the main launch file `start_launch.xml` for system startup. It launches all single launch files:

*1. `navigation_launch.xml`* : startups 2D pose estimate and Nav2 goal;

*2. `camera_36h11_launch.xml`* : initializes external camera and AprilTag detection;

*3. `assignment_1.launch.py`* : launches navigation and lifecycle client nodes;

*4. `laser_launch.xml`* : processes the laser for table detection.

### **• navigation_package**
: implements the logic of navigation inside the simulated environment.

It contains two nodes: 

*- lifecycle_client* sets the initial pose of robot and notifies the navigation_node if robot is ready to move with Ready.srv.

*- navigation_node* computes the goal position (in the middle of apriltags) and startups the navigation until final goal is reached.

Performs coordinate transformation in map frame to act the navigation in RViz2.



# Techica



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
