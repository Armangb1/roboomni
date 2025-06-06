FROM ros:humble

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DOMAIN_ID=1

# Create workspace early to leverage Docker layer caching
WORKDIR /ros2_ws

# Install system dependencies and clean up in single layer
RUN rm -f /etc/apt/sources.list.d/ros2-latest.list && \
    apt-get update && \
    apt-get install -y --no-install-recommends \
    curl \
    git \
    python3-colcon-common-extensions && \
    # Install ROS apt source
    export ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F "tag_name" | awk -F\" '{print $4}') && \
    curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb" && \
    apt-get install -y /tmp/ros2-apt-source.deb && \
    apt-get update



# Copy source code
COPY src/ src/

# Clone micro-ROS setup (separate layer for caching)
RUN git clone -b ${ROS_DISTRO} https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# Install dependencies (leverages package.xml caching)
RUN rosdep update && \
    rosdep install --from-paths src --ignore-src -y --rosdistro=${ROS_DISTRO} &&\
    # Clean up
    rm -rf /var/lib/apt/lists/* /tmp/ros2-apt-source.deb && \
    apt-get clean

# Build workspace and micro-ROS agent in single layer
RUN . /opt/ros/${ROS_DISTRO}/setup.sh && \
    colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release && \
    . install/setup.sh && \
    ros2 run micro_ros_setup create_agent_ws.sh && \
    ros2 run micro_ros_setup build_agent.sh

# Create entrypoint script
RUN echo '#!/bin/bash\n\
    set -e\n\
    source /opt/ros/${ROS_DISTRO}/setup.bash\n\
    source /ros2_ws/install/setup.bash\n\
    exec "$@"' > /ros_entrypoint.sh && \
    chmod +x /ros_entrypoint.sh

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["ros2", "launch", "mecanum_bringup", "system_bringup.launch.py"]