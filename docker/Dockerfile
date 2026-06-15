# syntax=docker/dockerfile:1
# Build with BuildKit for pip cache mounts: DOCKER_BUILDKIT=1 docker compose build

FROM ros:humble-ros-base

# ROS + system deps (install before NodeSource so apt/ros repos stay intact)
RUN apt-get update && apt-get install -y --no-install-recommends \
    ca-certificates \
    curl \
    gnupg2 \
    lsb-release \
    python3-pip \
    python3-requests \
    python3-colcon-common-extensions \
    portaudio19-dev \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    software-properties-common \
    ffmpeg \
    ros-humble-rviz2 \
    ros-humble-rosbridge-server \
    ros-humble-rosapi \
    ros-humble-compressed-image-transport \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-twist-mux \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
    ros-humble-navigation2 \
    ros-humble-nav2-amcl \
    ros-humble-nav2-bringup \
    ros-humble-nav2-map-server \
    ros-humble-nav2-util \
    ros-humble-pointcloud-to-laserscan \
    ros-humble-slam-toolbox \
    ros-humble-foxglove-bridge \
    ros-humble-vision-msgs \
    && rm -rf /var/lib/apt/lists/*

# Node.js for go2_dashboard Vite build / dev (after ROS packages)
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - \
    && apt-get update \
    && apt-get install -y --no-install-recommends nodejs \
    && rm -rf /var/lib/apt/lists/*

# Create workspace
WORKDIR /ros2_ws

# Install Python deps before copying full source so code edits don't re-run pip
COPY requirements.txt /tmp/requirements.txt
RUN --mount=type=cache,target=/root/.cache/pip \
    pip install --prefer-binary -r /tmp/requirements.txt

# Copy source after pip layer (cached unless requirements.txt changes)
COPY . /ros2_ws/src

# Resolve any remaining rosdep keys (most deps already installed above)
RUN apt-get update && . /opt/ros/${ROS_DISTRO}/setup.sh && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build

# Source ROS2 and workspace in bashrc
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /root/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> /root/.bashrc

# Set environment variables
ENV ROBOT_IP=""
ENV CONN_TYPE="webrtc"
ENV WEBRTC_SERVER_HOST="0.0.0.0"
ENV WEBRTC_SERVER_PORT="9991"

# Remove apt cache
RUN rm -rf /var/lib/apt/lists/*

# add /ros2_ws to the entrypoint
RUN sed --in-place --expression \
      '$isource "/ros2_ws/install/setup.bash"' \
      /ros_entrypoint.sh

CMD ["ros2", "launch", "go2_robot_sdk", "robot.launch.py"]
