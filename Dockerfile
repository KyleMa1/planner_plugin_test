ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO} AS base

ARG DEBIAN_FRONTEND=noninteractive

RUN apt-get update && apt-get install -y \
    curl \
    git \
    python3-pip \
    python3-vcstool \
    python3-rosdep \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws

COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
