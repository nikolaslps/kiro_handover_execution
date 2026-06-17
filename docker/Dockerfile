FROM eprosima/vulcanexus:humble-desktop

RUN apt-get update && apt-get install -y \
    python3-pip python3-opencv ffmpeg libsm6 libxext6 libnlopt-dev libnlopt-cxx-dev\
    git wget curl nano vim \
    ros-humble-diagnostic-aggregator \
    ros-humble-joint-state-publisher-gui \
    ros-humble-robot-state-publisher \
    ros-humble-tf-transformations \
    ros-humble-rqt-image-view \
    ros-humble-usb-cam \
    ros-humble-sensor-msgs \
    ros-humble-tf-transformations \
    ros-humble-topic-tools \
    ros-humble-octomap \
    ros-humble-octomap-server \
    ros-humble-octomap-mapping \
    ros-humble-octomap-ros \
    ros-humble-octomap-msgs \
    ros-humble-moveit \
    ros-humble-moveit-servo \
    ros-humble-moveit-visual-tools \
    ros-humble-tf2-geometry-msgs \
    ros-humble-interactive-markers \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-ur-description \
    ros-humble-rmw-fastrtps-cpp \
    && rm -rf /var/lib/apt/lists/*
ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp 

RUN pip3 install --no-cache-dir catkin_pkg lark-parser psutil ikpy lap transforms3d pycparser numpy==1.23.5 empy==3.3.4

WORKDIR /root/hri_exec_ws

RUN echo "source /opt/vulcanexus/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f /root/hri_exec_ws/install/setup.bash ]; then source /root/hri_exec_ws/install/setup.bash; fi" >> ~/.bashrc

# Default entrypoint
CMD ["/bin/bash"]