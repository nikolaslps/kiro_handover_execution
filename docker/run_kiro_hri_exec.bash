#!/bin/bash
# docker build -t kiro_hri_exec_vulcanexus .

# Make sure X11 can be accessed
xhost +local:docker

# Run the container
docker run -it \
    --rm \
    --name hri_execution_vulcanexus \
    --network host \
    --ipc host \
    --pid host \
    --env RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
    --env DISPLAY=$DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --env DEBIAN_FRONTEND=noninteractive \
    --volume "$(pwd)/hri_exec_ws:/root/hri_exec_ws:rw" \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --device /dev/video0:/dev/video0 \
    --privileged \
    --tty \
    kiro_hri_exec_vulcanexus # replace with your built image tag if different

# If using NVIDIA GPU. add the following flags
    # --gpus all \
    # --env NVIDIA_VISIBLE_DEVICES=all \
    # --env NVIDIA_DRIVER_CAPABILITIES=all \

# If want to use Discovery Server, add the following flags
    # --env ROS_DISCOVERY_SERVER=10.0.17.100:11811 \

# If want to specify Domain ID, add the following flag
    # --env ROS_DOMAIN_ID=25 \