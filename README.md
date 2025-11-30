# Assignment 1 - The turtlebot

The project contains the solution of the first assignment for the Intelligent Robotics course.
The objective is to implement an autonomous navigation pipeline for the turtlebot in the simulated environment, using *RViz2* for 2D visualization and *Gazebo GUI* for 3D simulation.

### Overview

The system enables the turtlebot to:

**• Detect the AprilTags** in the enviroment using the external camera;

**• Navigate in the room** to reach the computed final position;

**• Localize three cylindrical** tables using laser scan data.



# Project Structure

The project is implemented by five ROS2 packages:

### **• interfaces_assignment_1**

Defines the custom service interfaces:

*• `srv/Ready.srv`* : controls if the robot is ready to compute the navigation. The response `std_msgs/Bool ready` is `true` if robot can move, `false` otherwise.

*• `srv/Goalresult.srv`* : checks the final goal status and final position of the robot respectively with the `std_msgs/Bool goal_status` and `geometry_msgs/PoseStamped goal_pose` responses.

### **• laser_scan**
Technique to detect three cylindrical tables by clustering the points of laser scan and filtering them based on a range of acceptable distances.

The node `scan_subcribers` publishes detected table positions.

### **• my_apriltag_ros**
Computes the coordinates of center and the four corners of two AprilTags relative to the external camera. 

Provides data to the `navigation_node` in **navigation_package** to compute the final goal position.

### **• my_launch**
Contains the main launch file *`launch/start_launch.xml`* for system startup. It launches all single launch files:

*1. `navigation_package/launch/navigation_launch.xml`* : startups 2D pose estimate and Nav2 goal;

*2. `my_apriltag_ros/launch/camera_36h11_launch.xml`* : initializes external camera and AprilTag detection;

*3. `ir_2526/ir_launch/launch/assignment_1.launch.py`* : launches navigation and lifecycle client nodes;

*4. `laser_scan/launch/laser_launch.xml`* : processes the laser for table detection.

*`launch/start_launch_without_apriltag.xml`* has the same functionality without stream messages.

### **• navigation_package**
Implements the logic of navigation inside the simulated environment.

It contains two nodes: 

• `lifecycle_client` sets the initial pose of robot and notifies the navigation_node if robot is ready to move with Ready.srv.

• `navigation_node` computes the goal position (in the middle of apriltags) and startups the navigation until final goal is reached.

Performs coordinate transformation in map frame to act the navigation in RViz2.



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

**3. Clone the repository ir_2526 for the simulation**:

```
git clone https://github.com/PieroSimonet/ir_2526.git
```
This package must be added inside `ros2_ws/src`.


### Final structure

The final structure of workspace must be:

<pre>
ros2_ws
└── src
   ├── group_assignment_1
   │   ├── interfaces_assignment_1
   │   ├── laser_scan
   │   ├── my_apriltag_ros
   │   ├── my_launch
   │   └── navigation_package
   └── ir_2526
       ├── apriltag_ros
       ├── ir_base
       ├── ir_description
       ├── ir_launch
       └── ir_movit_config

</pre>

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

**6. (OPTIONAL)** launch the project without the node that detects apriltags (to have a more clear printing):

```
ros2 launch my_launch start_launch_without_apriltag.xml
```

**in another new terminal**:

```
ros2 launch my_apriltag_ros camera_36h11_launch.xml
```
