# Ubuntu 20.04 / ROS1 Noetic image for building and running SUPER.
FROM ros:noetic-ros-base-focal

SHELL ["/bin/bash", "-o", "pipefail", "-c"]
RUN apt-get update && DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential cmake git ca-certificates pkg-config python3-dev \
    libeigen3-dev libdw-dev libyaml-cpp-dev libopencv-dev \
    libglfw3-dev libglew-dev libncurses5-dev libncursesw5-dev \
    libgl1-mesa-dri mesa-utils \
    ros-noetic-pcl-ros ros-noetic-pcl-conversions ros-noetic-rosfmt \
    ros-noetic-tf ros-noetic-message-filters ros-noetic-mavros-msgs \
    ros-noetic-rviz \
    && rm -rf /var/lib/apt/lists/* \
    && ln -s /usr/include/eigen3/Eigen /usr/include/Eigen

# Build artifacts in bind mounts belong to the host user, not root.
ARG LOCAL_UID=1000
ARG LOCAL_GID=1000
RUN groupadd --gid "$LOCAL_GID" super \
    && useradd --uid "$LOCAL_UID" --gid "$LOCAL_GID" --create-home --shell /bin/bash super
COPY entrypoint.sh /usr/local/bin/super-entrypoint
RUN chmod +x /usr/local/bin/super-entrypoint \
    && printf '\nsource /opt/ros/noetic/setup.bash\nif [ -f "$SUPER_WS/devel/setup.bash" ]; then source "$SUPER_WS/devel/setup.bash"; fi\n' >> /home/super/.bashrc
USER super
ENTRYPOINT ["/usr/local/bin/super-entrypoint"]
CMD ["bash"]
