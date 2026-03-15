FROM ros:jazzy-ros-base

# Install additional tools you need
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    && rm -rf /var/lib/apt/lists/*

# Set up workspace
WORKDIR /ros2_ws
COPY src /ros2_ws/src

# Build workspace
RUN . /opt/ros/jazzy/setup.sh && \
    colcon build --symlink-install

# Source ROS 2 on container startup
RUN echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
RUN echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

CMD ["/bin/bash"]