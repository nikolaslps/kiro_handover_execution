# ARISE KIRO -- Handover Execution Node

![Vulcanexus](https://img.shields.io/badge/Vulcanexus-Humble-00214c?style=for-the-badge&logo=ros)
![License](https://img.shields.io/badge/license-Apache%202.0-green?style=for-the-badge&logo=apache)
![Build Status](https://img.shields.io/badge/build-manual-lightgrey?style=for-the-badge&logo=github-actions&logoColor=white)
![Docker](https://img.shields.io/badge/Docker-Ready-2496ED?style=for-the-badge&logo=docker&logoColor=white)

## Table of Contents

- [Overview](#overview)
- [Supported Setup](#supported-setup)
- [File Structure](#file-structure)
- [Contact Information](#contact-information)
- [Acknowledgements](#acknowledgements)
- [License](#license)

## Overview

The `kiro_handover_execution` node serves as the execution part of the pipeline. It orchestrates the communication between the calculation services and MoveIt2 to plan and execute collision-free trajectories to the optimal handover Poses. 

The node calls sequentially `GetActiveBodyID` and `ClusterHandoverVolume` (refer to [Handover Interfaces](https://github.com/nikolaslps/kiro_handover_interfaces) and [Handover Calculation](https://github.com/nikolaslps/kiro_handover_calculation) for more information) to identify the target handover points. After receiving the `<body_id>`, it implements an iterative planning strategy. If the manipulator cannot reach a target pose at low cluster value `k`, the node automatically requests more clusters and therefore more possible handover poses.

The custom node enforces orientation constraints during path planning to ensure the robot's end-effector remains in a desired orientation during planning (box parallel to the ground). 

The node also calls the octomap server in order to detect obstacles in front of the robot to avoid collisions.

## Supported Setup

| Category | Tested On | Expected Compatibility | Not Supported / Unknown |
| :--- | :--- | :--- | :--- |
| **Middleware & OS** | **Vulcanexus Humble** (Ubuntu 22.04 LTS) utilizing **Fast DDS** as the default RMW middleware layer | Standard ROS 2 Humble setups | Older ROS distributions (e.g., Foxy, Galactic) or ROS 1 |
| **Sensors** | **Intel RealSense D455** & **Bpearl 3D Lidar** | Any RGB-D sensor or Lidar providing standardized PointCloud2 data streams | Monocular 2D webcams (lacking spatial depth parameters) |
| **Manipulators** | **Universal Robots UR10e** | Any ROS 2 / MoveIt2-configured arm with a defined planning group and compatible IK solver | Only tested with the UR10e manipulator (6 DoFs). For more DoFs, unknown outcome. |

## File Structure

```text
kiro_handover_execution/
├── config/
│   └── handover_params.yaml                  # IK solver & runtime parameters
├── docker/
│   ├── Docker-Install.md                     # Docker Installation and Launch Manual
│   ├── Dockerfile                            # Dockerfile based on Vulcanexus image
│   ├── run_kiro_hri_exec.bash                # Script to launch the Docker Container
│   └── setup_hri_exec.sh                     # Script to build the Docker Container
├── docs/
│   ├── 01_arise_context.md                   # ARISE Ecosystem Context & Core Integration
│   ├── 02_interfaces.md                      # Interface Documentation
│   ├── 03_installation.md                    # Installation and Usage Guide
│   ├── 04_ros_configuration.md               # ROS2 Configuration
│   ├── 05_basic_demo_how_to_use.md           # Documentation for demo (how-to-use)
│   └── 06_role_in_demonstrator.md            # Role in the TRL 6-7 Demonstrator
├── include/
│    └── kiro_handover_execution/
│           ├── arc_planner.hpp               # Cartesian arc path planning
│           ├── bezier_arcs.hpp               # Curve generation & obstacle detection
│           ├── feasibility.hpp               # IK solvability & collision checking
│           ├── workspace_checker.hpp         # UR10e workspace limits validation
│           ├── point_cloud_observer.hpp      # Real-time lidar subscription
│           ├── path_visualizer.hpp           # RViz visualization publisher
│           └── Ur10eCollisionModel.hpp       # Capsule-based collision model
├── media/                                    # Images 
├── src/
│   ├── arc_planner.cpp
│   ├── bezier_arcs.cpp
│   ├── feasibility.cpp
│   ├── handover_execution.cpp                # Main execution node & pipeline orchestrator
│   ├── workspace_checker.cpp
│   ├── point_cloud_observer.cpp
│   ├── path_visualizer.cpp
│   └── Ur10eCollisionModel.cpp
├── .gitignore
├── CMakeLists.txt                            # Build configuration
├── LICENSE                                   # License information
├── package.xml                               # Package metadata and dependencies
├── README.md                                 # Overview of the ARISE KIRO specific package
└── requirements.txt                          # Python dependencies if built from source
```

> [!WARNING]
> **1. Hardware Specification (UR10e Manipulator Only)**
> The geometric constraint validations, forward/inverse kinematics mappings, link name lookups, and capsule-based collision boundary configurations are strictly calibrated and verified for the **Universal Robots UR10e** arm. Running alternative kinematic chains or manipulators from other vendors requires manually updating the joint models, link transformations, and capsule matrices inside the source logic. Manipulators with distinct structural topologies are natively unsupported.
>
> **2. Critical Upstream Dependencies**
> This package operates as an execution and path-validation downstream layer and is strictly dependent on the **[kiro_handover_calculation](https://github.com/nikolaslps/kiro_handover_calculation)** module. The pipeline orchestrator cannot compute trajectories or command the arm autonomously without actively receiving the tracking subject's localized `body_id` and the clustered point cloud target centroids supplied by the upstream service node.

## Contact Information

For queries regarding the development, replication, or integration of this execution module within the ARISE framework, feel free to reach out:

**Maintainer:**
* **Developer:** [Nikolaos Lappas](https://github.com/nikolaslps)
* **Email:** [nikolas.lappas.2003@gmail.com](mailto:nikolas.lappas.2003@gmail.com)

**Project Contacts (IKNOWHOW SA):**
* Maria Kampa: [mkampa@iknowhow.com](mailto:mkampa@iknowhow.com)

## Acknowledgements
Developed as part of the KIRO experiment, co-funded by the European Union under the Horizon Europe ARISE project (GA 101135784).

## License
This project is licensed under the Apache License 2.0. See the [LICENSE](LICENSE) file for details.

**Copyright:**
© ATHENA RC (KIRO partner), on behalf of the KIRO experiment within the ARISE project (Horizon Europe GA 101135784).