#!/bin/bash

CONTAINER_NAME="kiro_hri_exec_vulcanexus"
WS_NAME="hri_exec_ws"
SRC_NAME="src"

echo -e "\e[36m===================================================\e[0m"
echo -e "\e[36m      KIRO HRI EXECUTION: Master Setup System      \e[0m"
echo -e "\e[36m===================================================\e[0m"

REPO_ROOT="$(pwd)"
if [ ! -f "package.xml" ] || [ ! -d "docker" ]; then
    echo -e "\e[31mError: Please run this script from the repository root directory!\e[0m"
    echo -e "e.g., ./docker/setup_hri_exec.sh"
    exit 1
fi

echo -e "\e[33m[1/3] Creating local shared workspace '$WS_NAME'...\e[0m"
mkdir -p "$WS_NAME/$SRC_NAME"

echo -e "\e[33m[2/4] Copying local package to workspace...\e[0m"
if [ -d "$WS_NAME/$SRC_NAME/kiro_handover_execution" ]; then
    echo -e "\e[32m  -> Local package already copied. Refreshing source...\e[0m"
    rm -rf "$WS_NAME/$SRC_NAME/kiro_handover_execution"
fi
mkdir -p "$WS_NAME/$SRC_NAME/kiro_handover_execution"
rsync -av . "$WS_NAME/$SRC_NAME/kiro_handover_execution" --exclude="$WS_NAME" --exclude=".git" > /dev/null


echo -e "\e[33m[2/3] Cloning repositories to host ...\e[0m"
cd "$WS_NAME/$SRC_NAME"

if [ ! -d "kiro_handover_interfaces" ]; then 
    echo -e "\e[33m  -> Cloning kiro_handover_interfaces...\e[0m"
    git clone https://github.com/nikolaslps/kiro_handover_interfaces
fi

# In order to use trac_ik instead of the default KDL planner for moveit
if [ ! -d "trac_ik" ]; then 
    echo -e "\e[33m  -> Cloning and checking out trac_ik (commit 210f767)...\e[0m"
    git clone https://bitbucket.org/traclabs/trac_ik.git
    cd trac_ik
    git checkout 210f767
    cd ..
fi

cd "$REPO_ROOT"

# Build the Docker Image
echo -e "\e[33m[3/3] Building the Docker image '$CONTAINER_NAME'...\e[0m"
if [ -f "docker/Dockerfile" ]; then
    docker build -t "$CONTAINER_NAME" -f docker/Dockerfile docker/
else
    echo -e "\e[31mError: Dockerfile not found in docker/ directory!\e[0m"
    exit 1
fi

echo -e "\e[33m[4/4] Building ROS 2 Workspace inside the container...\e[0m"

docker run --rm \
    --volume "$(pwd)/hri_exec_ws:/root/hri_exec_ws:rw" \
    $CONTAINER_NAME \
    bash -c "source /opt/vulcanexus/humble/setup.bash && \
            cd /root/hri_exec_ws && \
            colcon build --symlink-install"

echo -e "\e[36m================================================\e[0m"
echo -e "\e[32mSUCCESS: Workspace ready and Image built.\e[0m"
echo -e "\e[36mTo run: ./docker/run_kiro_hri_exec.bash\e[0m"
echo -e "\e[36m================================================\e[0m"