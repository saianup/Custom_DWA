
````markdown
# Custom DWA Local Planner for ROS 2

## Overview
This project implements a **Custom Dynamic Window Approach (DWA) Local Planner** for a **TurtleBot3** robot in **ROS 2 Humble**.  
The planner generates velocity commands (`/cmd_vel`) to navigate toward a goal while avoiding obstacles using LIDAR data.  

Unlike the standard `nav2_dwb_controller`, this implementation is written from scratch in **Python (`rclpy`)**, allowing a clear understanding of trajectory sampling, scoring, and selection.

Goals can be set interactively through **RViz (2D Goal Pose)**, and the planner visualizes predicted paths and goals in real time.

---

## Features
- Full Dynamic Window Approach implementation from scratch.
- Samples and predicts feasible velocity commands under motion constraints.
- Evaluates trajectories based on:
  - Goal heading
  - Velocity smoothness
  - Obstacle clearance
- Publishes:
  - `/cmd_vel` for motion control  
  - `/dwa_marker` for trajectory and goal visualization
- Subscribes to:
  - `/odom` (Odometry)
  - `/scan` (LaserScan)
  - `/goal_pose` (RViz Goal Pose)
- Automatically stops when goal is reached.

````

## Dependencies
Make sure the following packages are installed before building:

```bash
sudo apt update
sudo apt install ros-humble-rclpy \
                 ros-humble-geometry-msgs \
                 ros-humble-nav-msgs \
                 ros-humble-sensor-msgs \
                 ros-humble-visualization-msgs \
                 ros-humble-tf-transformations \
                 ros-humble-turtlebot3 \
                 ros-humble-turtlebot3-gazebo \
                 ros-humble-rviz2
```

---

## Setup Instructions

### 1. Clone TurtleBot3 Simulation Packages

If not already installed, clone the official TurtleBot3 repositories:

```bash
cd ~/turtlebot_ws/src
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git
git clone -b humble-devel https://github.com/ROBOTIS-GIT/DynamixelSDK.git
```

---

### 2. Clone This Repository

```bash
cd ~/turtlebot_ws/src
git clone https://github.com/saianup/Custom_DWA.git 
```



### 3. Build the Package

```bash
cd ~/turtlebot_ws
colcon build --packages-select dwa_custom
source install/setup.bash
```



## Running the Planner

### Step 1 — Launch TurtleBot3 in Gazebo

```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

### Step 2 — Run the DWA Planner Node

Open a new terminal:

```bash
source ~/turtlebot_ws/install/setup.bash
ros2 run dwa_custom dwa_planner
```

### Step 3 — Launch RViz for Visualization

Open another terminal:

```bash
ros2 run rviz2 rviz2
```

Then:

1. Set **Fixed Frame** to `odom`.
2. Add the following topics:

   * **LaserScan** → `/scan`
   * **Marker** → `/dwa_marker`
3. Use **2D Goal Pose** tool to set a target goal.



## Demo Video

A short explanation of the DWA theory followed by the Gazebo demonstration:

### DWA Theory Explanation    
[![Watch Demo Video](https://img.youtube.com/vi/VPJGWbVfHac/0.jpg)](https://youtube.com/shorts/VPJGWbVfHac?si=R5hOXG8CrPLpUyYB)


### Gazebo Demonstration 
[![Watch Theory Video](https://img.youtube.com/vi/qPRI2pnHRCs/0.jpg)](https://youtu.be/qPRI2pnHRCs?si=SkHOAJGu3gr8J1pp)


## Directory Structure

```
turtlebot_ws/
└── src/
    └── dwa_custom/
        ├── package.xml
        ├── setup.py
        ├── README.md
        └── dwa_custom/
            ├── __init__.py
            └── planner.py
```


## Known Issues 

* Trajectory scoring weights require tuning for smoother motion.
* Obstacle avoidance can be improved with advanced cost functions.
