#!/bin/bash

echo "Starting Lite6 ROS 2 Docker Environment..."

# 1. Allow local X11 connections (Required for PyQt5 and RViz GUI)
xhost +local:root

# 2. Define the container name and image name
IMAGE_NAME="lite6_mujoco_env"
CONTAINER_NAME="lite6_container"

# 3. Build the Docker image if it doesn't exist yet
if [[ "$(docker images -q $IMAGE_NAME 2> /dev/null)" == "" ]]; then
    echo "Building Docker image for the first time. This may take a few minutes..."
    docker build -t $IMAGE_NAME .
fi

# 4. Run the container with GPU, GUI, and Volume Mounting enabled
docker run -it --rm \
    --name $CONTAINER_NAME \
    --net host \
    --gpus all \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/src:/root/lite6_ws/src" \
    $IMAGE_NAME