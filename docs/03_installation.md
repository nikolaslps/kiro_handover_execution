# Installation and Usage Guide

This guide covers setting up the `kiro_handover_execution` pipeline. You can either deploy using the recommended docker container layout included directly in this repository or build natively from source.

---

## Installation

### 1. Pre-configured Docker Container (Recommended)
For a stable development experience with all system and Python dependencies pre-installed, use the native Docker environment files located inside the `/docker` directory of this repository.

* Refer to the internal [Docker Installation](../docker/Docker-Install.md) for full installation setup steps.


### 2. Building from Source

> [!WARNING]
> Native compilation on the host machine has not been fully verified across all hardware profiles (only tested on Ubuntu 22.04 and partially on Ubuntu 24.04). Using the integrated Docker automation above is highly recommended to prevent dependency drift or configuration conflicts.

> [!IMPORTANT]
> **Prerequisites & Execution Requirements** 
> * **Environment:** Ensure your system is running **ROS 2 Humble**, preferably deployed on a [Vulcanexus Humble](https://docs.vulcanexus.org/en/latest/) base image.
> * **Hardware/Simulation Integration:** All geometric configurations, collision matrices, and workspace checks are strictly verified for a **Universal Robots UR10e** manipulator. 
> * **MoveIt 2 Core Stack:** MoveIt 2 must be installed and properly configured alongside your specific robot description and planning scene groups.

#### Setup Workspace
Create a standard development workspace and an underlying source directory:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

> [!IMPORTANT]
> **Repository Placement:** Before proceeding, you must manually copy some files from this repository straight into your new source space.
> 
> Your directory layout **must** look exactly like this for the build system to work:
> ```text
> ~/ros2_ws/src/kiro_handover_execution/                            
> ├── include/
> │   └── kiro_handover_execution/
> │       ├── arc_planner.hpp                     
> │       ├── bezier_arcs.hpp                     
> │       ├── feasibility.hpp                     
> │       ├── workspace_checker.hpp               
> │       ├── point_cloud_observer.hpp            
> │       ├── path_visualizer.hpp                 
> │       └── Ur10eCollisionModel.hpp             
> ├── src/
> │   ├── arc_planner.cpp
> │   ├── bezier_arcs.cpp
> │   ├── feasibility.cpp
> │   ├── handover_execution.cpp                  
> │   ├── workspace_checker.cpp
> │   ├── point_cloud_observer.cpp
> │   ├── path_visualizer.cpp
> │   └── Ur10eCollisionModel.cpp
> ├── config/
> │   └── handover_params.yaml                    
> ├── CMakeLists.txt                              
> ├── package.xml                                 
> └── requirements.txt                            
> ```

Clone the remaining mandatory external packages into that same `~/ros2_ws/src` folder:

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

