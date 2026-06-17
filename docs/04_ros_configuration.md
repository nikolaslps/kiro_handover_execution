# ROS 2 Configuration Guide

This section covers the runtime optimization settings, node execution arguments, and 3D perception sensor plugin mappings required for the `kiro_handover_execution` package.

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

1. Option A: Using **3D Lidar's** topic
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
2. Option B: Using **Image's Depth** topic

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