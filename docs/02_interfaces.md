# Interface Documentation

This page lists all exposed topics, service interfaces, parameters, and standardization bindings provided by the core `kiro_handover_execution` workspace package.

## 1. ROS 2 Vulcanexus Node

### Node: `handover_execution`
The primary pipeline orchestrator and motion generation node. It coordinates with upstream perception layers, performs obstacle avoidance parsing, projects workspace safety limits, and drives the UR10e manipulator.

* **Subscribed Topics:**
    * `/bpearl_lidar/points` (`sensor_msgs/msg/PointCloud2`) — 3D Lidar depth point stream monitored continuously feeding the Octomap server with point data for collision checking.
* **Published Topics:**
    * `/handover/BezierCurves` (`visualization_msgs/msg/MarkerArray`) — Spatial markers showing candidate Cartesian Bezier paths, evaluated clearance zones, and localized obstacle collision points inside RViz 2.
* **Service Servers:**
    * `/activate_handover` (`kiro_handover_interfaces/srv/ActivateHandover`) — Main entry trigger. When set to `true`, it triggers the calculation stack, requests target centroids, conducts multi-tiered collision/kinematic testing, and executes the physical handover trajectory.
* **Service Clients:**
    * `/activate_handover_calc` (`kiro_handover_interfaces/srv/ActivateHandover`) — Triggers the upstream tracking module to start or to sleep dynamically depending on the current operational loop state.
    * `/get_active_body_id` (`kiro_handover_interfaces/srv/GetActiveBodyID`) — Queries the localized human subject token tracking matrix.
    * `/cluster_handover_volume` (`kiro_handover_interfaces/srv/ClusterHandoverVolume`) — Obtains the discrete clustered target array of $k$ interaction centroids calculated by the perception module.

> [!NOTE]
> **KIRO Interfaces:** For more information regarding the exact service definitions and structural interface parameters, please refer to the corresponding package repository:[`kiro_handover_interfaces`](https://github.com/nikolaslps/kiro_handover_interfaces).

---

## 2. Industrial Standards & Downstream Automation Compliance

| Standard Interface / Frame | Functional Role |
| :--- | :--- |
| MoveIt 2 Planning Scene | Interacts natively with `MoveGroupInterface` and `PlanningSceneMonitor` to ensure state representations match physical boundaries before joint commanding. |
| Cartesian Trajectory Feed | Dispatches deterministic trajectories via `computeCartesianPath` down to the industrial manipulator controller bus with an advanced maximum resolution step (`eef_step = 0.01`m). |
| Capsule Collision Grid | Explicitly maps the structural frame links of the UR10e manipulator (`shoulder_link`, `upper_arm_link`, `forearm_link`, `wrist_1_link`, `wrist_2_link`, `wrist_3_link`) into analytical capsule boundaries to audit obstacle distance queries dynamically. |
