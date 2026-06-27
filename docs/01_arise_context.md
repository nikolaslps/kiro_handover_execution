# ARISE Ecosystem Context & Core Integration

This document outlines how the `kiro_handover_execution` module interoperates with the core components of the ARISE All-in-one Middleware ecosystem, ensuring compliance with standardization protocols and cross-platform reusability.

## 1. Vulcanexus & ROS 2 Alignment
The module is natively built and verified on the **ROS 2 Vulcanexus** ecosystem, leveraging advanced middleware layer features to guarantee deterministic behavior and sub-centimeter accuracy in close-proximity human-robot collaboration:

* **eProsima Fast DDS Optimization & RMW Configuration:** The path generation and sensor monitoring threads employ tailored Quality of Service (QoS) profiles to safely manage high-frequency data streams. To fully exploit eProsima's high-performance throughput, the integrated container setup directly targets the fast intermediate middleware implementation. The repository's workspace automation script inside the `/docker` directory explicitly forces this binding by setting the underlying environment flag:
  ```bash
  --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  ```
* **Centralized Discovery Service Configuration:** To eliminate heavy network multicast traffic overhead common on shared industrial plant networks, the system is designed to support eProsima's **Discovery Server** paradigm, by uncommenting the following encironmental variable inside the `/docker/run_kiro_hri_calc.bash` script. 
  ```bash
  --env ROS_DISCOVERY_SERVER=10.0.17.100:11811 # setup yours
  ```
* **Multi-Threaded Execution & Asynchronous Coordination:** To prevent execution deadlocks when calling upstream services (such as `cluster_handover_volume` and `get_active_body_id`) while simultaneously spinning state subscribers, the main entry point (`handover_execution.cpp`) instantiates a localized, multi-threaded worker engine:
  ```cpp
  rclcpp::executors::MultiThreadedExecutor executor;
  ```

## 2. ROS4HRI & ROS4RI Standardization Compliance
To ensure this package functions as a plug-and-play asset across different ARISE application fields, it establishes a direct bridge between ROS4HRI (Human-Robot Interaction) perception node and industrial manipulator planners:
* **MoveIt 2 Master Orchestration Integration:** The pipeline acts as the automation bridge for the UR10e manipulator. It natively wraps MoveIt 2's `MoveGroupInterface` and `PlanningSceneMonitor` to compute continuous, collision-free interaction trajectories. By consuming the candidate clustered handover centroids calculated upstream by the [`kiro_handover_calculation`](https://github.com/nikolaslps/kiro_handover_calculation) package (which utilizes the standard ROS4HRI framework for body tracking and skeleton joint state extraction), the pipeline evaluates the trajectory starting from the manipulator's initial joint space configuration. It iteratively audits each target point until it returns a fully feasible, collision-free, and Kinematically reachable path.