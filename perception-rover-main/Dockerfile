FROM perception_rover_vnc

RUN apt-get update && apt-get install -y \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-nav2-minimal-tb3-sim \
    ros-jazzy-turtlebot3-description \
    ros-jazzy-slam-toolbox \
    ros-jazzy-ros-gz-sim \
    git \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# Copy perception_rover source into workspace
COPY . /ros2_ws/src/perception_rover

# Build perception_rover
RUN . /opt/ros/jazzy/setup.sh && \
    cd /ros2_ws && \
    colcon build --packages-select perception_rover
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc
