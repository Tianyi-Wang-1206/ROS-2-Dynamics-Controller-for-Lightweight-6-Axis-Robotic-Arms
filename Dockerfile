# 1. Base Image: ROS 2 Humble Desktop
FROM osrf/ros:humble-desktop

# (Optional) Use local sources
RUN sed -i 's/archive.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/security.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list && \
    sed -i 's/ports.ubuntu.com/mirrors.ustc.edu.cn/g' /etc/apt/sources.list

RUN find /etc/apt/ -type f -exec sed -i 's/packages.ros.org/mirrors.ustc.edu.cn/g' {} + && \
    find /etc/apt/ -type f -exec sed -i 's/repo.ros2.org/mirrors.ustc.edu.cn\/ros2/g' {} +

# 2. Install basic system tools and PyQt5
RUN apt-get update && apt-get install -y \
    curl git build-essential \
    python3-pip python3-pyqt5 \
    && rm -rf /var/lib/apt/lists/*

# 3. Inject the local MuJoCo directory into the Docker container
COPY mujoco-3.9.0 /opt/mujoco/mujoco-3.9.0

# Set MuJoCo Environment Variables inside the container
ENV MUJOCO_DIR=/opt/mujoco/mujoco-3.9.0
ENV LD_LIBRARY_PATH=${MUJOCO_DIR}/lib:${LD_LIBRARY_PATH:-}

# 4. Install Python dependencies
RUN pip3 install -i https://pypi.mirrors.ustc.edu.cn/simple "numpy<2.0.0" scipy rosdepc

# 5. Set up the ROS 2 workspace
ENV WS_DIR=/root/lite6_ws
RUN mkdir -p ${WS_DIR}/src
WORKDIR ${WS_DIR}

# 6. Copy the project's source code into the container
COPY src ${WS_DIR}/src

# Install ROS 2 dependencies automatically via rosdep
RUN apt-get update && \
    rosdepc init && \
    rosdepc update && \
    rosdepc install --from-paths src --ignore-src -r -y && \
    rm -rf /var/lib/apt/lists/*

# 7. Automatically source ROS 2 and the workspace
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "if [ -f ${WS_DIR}/install/setup.bash ]; then source ${WS_DIR}/install/setup.bash; fi" >> ~/.bashrc

# 8. Start with a bash shell
CMD ["bash"]