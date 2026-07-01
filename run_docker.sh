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

# 4. Auto-detect GPU for Hardware Acceleration
GPU_FLAGS=""
if command -v nvidia-smi &> /dev/null; then
    echo "🟢 NVIDIA GPU detected. Enabling NVIDIA Container Toolkit..."
    GPU_FLAGS="--gpus all --env=NVIDIA_DRIVER_CAPABILITIES=all"
elif [ -d "/dev/dri" ]; then
    echo "🟢 AMD/Intel GPU detected. Enabling DRI hardware acceleration..."
    GPU_FLAGS="--device=/dev/dri:/dev/dri"
else
    echo "🟡 No GPU detected. RViz2 and MuJoCo will run using CPU software rendering (expect low FPS)."
    # Optional: Force software rendering environment variable if needed
    GPU_FLAGS="--env=LIBGL_ALWAYS_SOFTWARE=1"
fi

# 5. Run the container
docker run -it --rm \
    --name $CONTAINER_NAME \
    --net host \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --volume="$(pwd)/src:/root/lite6_ws/src" \
    $GPU_FLAGS \
    $IMAGE_NAME