# ARISE KIRO -- Handover Execution Node

![Vulcanexus](https://img.shields.io/badge/Vulcanexus-Humble-00214c?style=for-the-badge&logo=ros)
![License](https://img.shields.io/badge/license-Apache%202.0-green?style=for-the-badge&logo=apache)
![Build Status](https://img.shields.io/badge/build-passing-brightgreen?style=for-the-badge&logo=github-actions&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?style=for-the-badge&logo=docker&logoColor=white)

## Table of Contents

- [Overview](#overview)
- [File Structure](#file-structure)
- [Configuration](#configuration)
- [Installation](#installation)
- [Usage](#usage)
- [Supported Distribution](#supported-distribution)
- [License](#license)

## Overview

The `kiro_handover_execution` node serves as the execution part of the pipeline. It orchestrates the communication between the calculation services and MoveIt2 to plan and execute collision-free trajectories to the optimal handover Poses. 

The node calls sequentially `GetActiveBodyID` and `ClusterHandoverVolume` (refer to [Handover Interfaces](https://github.com/nikolaslps/kiro_handover_interfaces) and [Handover Calculation](https://github.com/nikolaslps/kiro_handover_calculation) for more information) to identify the target handover points. After receiving the `<body_id>`, it implements an iterative planning strategy. If the manipulator cannot reach a target pose at low cluster value `k`, the node automatically requests more clusters and therefore more possible handover poses.

The custom node enforces orientation constraints during path planning to ensure the robot's end-effector remains in a desired orientation during planning (box parallel to the ground). 

The node also calls the octomap server in order to detect obstacles in front of the robot to avoid collisions.

## File Structure

```
kiro_handover_execution/
├── include/
│    └── kiro_handover_execution/
│           ├── arc_planner.hpp               # Cartesian arc path planning
│           ├── bezier_arcs.hpp               # Curve generation & obstacle detection
│           ├── feasibility.hpp               # IK solvability & collision checking
│           ├── workspace_checker.hpp         # UR10e workspace limits validation
│           ├── point_cloud_observer.hpp      # Real-time lidar subscription
│           ├── path_visualizer.hpp           # RViz visualization publisher
│           └── Ur10eCollisionModel.hpp       # Capsule-based collision model
├── src/
│   ├── arc_planner.cpp
│   ├── bezier_arcs.cpp
│   ├── feasibility.cpp
│   ├── handover_execution.cpp                # Main execution node & pipeline orchestrator
│   ├── workspace_checker.cpp
│   ├── point_cloud_observer.cpp
│   ├── path_visualizer.cpp
│   └── Ur10eCollisionModel.cpp
├── config/
│   └── handover_params.yaml                  # IK solver & runtime parameters
├── CMakeLists.txt                            # Build configuration
├── package.xml                               # Package metadata and dependencies
├── requirements.txt                          # Python dependencies if built from source
├── README.md                                 # Documentation
└── LICENSE                                   # License information
```

## Configuration

### Runtime Parameters (`handover_params.yaml`)

```yaml
/**:
  ros__parameters:
    robot_description_kinematics:
      ur_manipulator:
        kinematics_solver: trac_ik_kinematics_plugin/TRAC_IKKinematicsPlugin
        kinematics_solver_timeout: 0.05  # seconds
        solve_type: Distance             # IK solver type
```

### Runtime Arguments 
1. `use_sim_time:=false`: If `true`, it means simulation time. If `false`, it means working with a real robot.
2. `move_group_name:=ur_manipulator`: The `name` of the planning group as initialized inside MoveIt2 `robot.srdf` file.
3. `use_collision_capsules:=false`: If `true`, it means enabling UR10e capsules, bigger than the defaults of MoveIt2. Need to check more `Ur10eCollisionModel.hpp` and `Ur10eCollisionModel.cpp` files for correctly setting up the link names.

### Topic Subscriptions

| Topic | Type | Purpose |
|-------|------|---------|
| `/filtered_cloud` | `sensor_msgs/PointCloud2` | Based on this topic the collision detection with the planning scene is performed. Note: Requires manual setup inside `kiro_moveit_config/config/sensors_3d.yaml`. |

#### Example of the `sensors_3d.yaml` file

1. Using **3D Lidar's** topic
```yaml
sensors:
  - bpearl_lidar

bpearl_lidar:
  sensor_plugin: occupancy_map_monitor/PointCloudOctomapUpdater
  point_cloud_topic: /bpearl_lidar/points
  max_range: 5.0
  point_subsample: 1
  padding_offset: 0.1
  padding_scale: 1.0
  max_update_rate: 1.0
  filtered_cloud_topic: filtered_cloud
  filter_box_padding: 0.1
  filter_box_scale: 1.0
```
2. Using **Image's Depth** topic

```yaml
sensors:
  - realsense_depth_image

realsense_depth_image:
    sensor_plugin: occupancy_map_monitor/DepthImageOctomapUpdater
    image_topic: /camera/camera/depth/image_rect_raw
    queue_size: 3
    near_clipping_plane_distance: 0.01
    far_clipping_plane_distance: 2.0
    shadow_threshold: 0.2
    padding_scale: 1.0
    max_update_rate: 5.0
    filtered_cloud_topic: filtered_cloud
```

## Installation

### 1. Docker Container (Recommended)
For the most stable experience, we recommend using our pre-configured Docker environment.
* Refer to the [ARISE KIRO Docker Repository](https://github.com/andvatistas/ARISE-KIRO-reusable-modules) for setup assistance.
* Follow the provided `README.md` within that repository to pull the image and launch the container.

### 2. Building from Source
**Note**: Building from source has not been fully tested in all environments. We strongly recommend using the Docker version above.

#### Prerequisites
* **Environment:** ROS 2 Humble (Vulcanexus image recommended).
* **Hardware/Simulation:** All tests and configurations were performed using a **Universal Robots UR10e** manipulator.
* **MoveIt 2:** Ensure MoveIt 2 is installed and correctly configured for your robot description.
#### Setup Workspace
Clone the repositories into your ROS 2 workspace `src` folder:

```bash
cd ~/ros2_ws/src
```

```bash
# Trac IK Planner
git clone https://bitbucket.org/traclabs/trac_ik.git
git -C trac_ik checkout 210f767
```

```bash
# Handover service definitions
git clone https://github.com/nikolaslps/kiro_handover_interfaces
```

```bash
# Handover Execution Node
git clone https://github.com/nikolaslps/kiro_handover_execution.git
```

Install Dependencies
```bash
cd ~/ros2_ws
sudo rosdep init # May not be necessary
rosdep update
apt-get update
rosdep install --from-paths src --ignore-src -y -r --rosdistro humble
```

Download the python requirements
```bash
cd ~/ros2_ws/src/kiro_handover_execution/
pip install -r requirements.txt
```

Build this package by running the following from inside the `ros2_ws`:
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

## Usage
Launch the handover nodes by running:
```bash
ros2 run kiro_handover_execution handover_execution --ros-args \
-p use_sim_time:=false \
-p move_group_name:=ur_manipulator \
-p use_collision_capsules:=false \
--params-file src/kiro_handover_execution/config/handover_params.yaml 
```

In order to trigger the hri body detection and handover calculation to start, run:
```bash
ros2 service call /activate_handover kiro_handover_interfaces/srv/ActivateHandover "{handover_phase: true}"
```

## Supported Distribution
* **ROS 2 Humble on Vulcanexus image**
* Tested on the **Universal Robot 10e** manipulator.

## License
This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.