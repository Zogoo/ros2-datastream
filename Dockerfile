FROM ros:lyrical-ros-core

ENV DEBIAN_FRONTEND=noninteractive
ENV ROS_DISTRO=lyrical

# System dependencies
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-numpy \
    python3-opencv \
    python3-colcon-common-extensions \
    python3-setuptools \
    python3-pytest \
    ros-lyrical-foxglove-bridge \
    ros-lyrical-rosbridge-suite \
    ros-lyrical-cv-bridge \
    ros-lyrical-rosbag2 \
    ros-lyrical-rosbag2-storage-mcap \
    ros-lyrical-rosbag2-transport \
    ros-lyrical-tf2-ros \
    ros-lyrical-tf2-tools \
    && rm -rf /var/lib/apt/lists/*

# Quality-gate tooling (make check runs these inside this image)
RUN pip3 install --no-cache-dir --break-system-packages ruff mypy

# Create workspace
WORKDIR /ros2_ws

# Copy source packages + shared geometry/spec JSON (single source of truth)
COPY src/ /ros2_ws/src/
COPY shared/ /ros2_ws/shared/

# Build workspace
RUN /bin/bash -c "source /opt/ros/lyrical/setup.bash && \
    colcon build --symlink-install \
    --packages-select onsen_dummy_robot onsen_ai_worker onsen_robot_state"

# Entrypoint
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
