# Docker Installation and Launch

This is the recommended way of installing and running the `kiro_handover_execution` package. 

## Build the Docker Container
Build the docker image by running the executable from the repository root .
```bash
chmod +x ./docker/setup_hri_exec.sh # if not executable already
```
```
./docker/setup_hri_exec.sh
```

If the installation finishes correctly, you will be greeted with a success message:

> [!NOTE]
> **Expected Build Output:**
> ```text
> ================================================
> SUCCESS: Workspace ready and Image built.
> To run: ./docker/run_kiro_hri_exec.bash
> ================================================
> ```

## Start the Docker Container
Run the executable.
```bash
chmod +x ./docker/run_kiro_hri_exec.bash # if not executable already
```
```
./docker/run_kiro_hri_exec.bash
```

> [!NOTE]
> **Expected Terminal Transition:**
> ```text
> non-network local connections being added to access control list
> root@container-environment:~/hri_exec_ws#
> ```

## From inside the container 
```bash
ros2 run kiro_handover_execution handover_execution --ros-args \
-p use_sim_time:=false \
-p move_group_name:=ur_manipulator \
-p use_collision_capsules:=false \
--params-file src/kiro_handover_execution/config/handover_params.yaml 
```

> [!TIP]
> Before launching, make sure to check the `ros-args`. For a full deep-dive into configurable parameters, see the [ROS Configuration Documentation](../docs/04_ros_configuration.md).


## Testing the services
This package uses the [kiro_handover_interfaces](https://github.com/nikolaslps/kiro_handover_interfaces) package in order to communicate with the mission controller (FSM) which orchestrates the desired tasks and the [kiro_handover_calculation](https://github.com/nikolaslps/kiro_handover_execution) package which is responsible for the human-receiver tracking and optimal handover poses calculation.

### Service to start the Execution node, which sequentially triggers the Calculation node:
```bash
ros2 service call /activate_handover kiro_handover_interfaces/srv/ActivateHandover "{handover_phase: true}"
```