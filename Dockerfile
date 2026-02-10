ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}

ARG ROS_DISTRO
RUN apt-get update && apt-get upgrade -y && apt-get install -y --no-install-recommends \
    clang-format \
    clang-tidy \
    ros-${ROS_DISTRO}-ament-cmake-clang-format \
    ros-${ROS_DISTRO}-ament-cmake-clang-tidy \
    ros-${ROS_DISTRO}-ament-lint-auto \
    ros-${ROS_DISTRO}-ament-lint-common \
    ros-${ROS_DISTRO}-ament-cmake-gtest \
    ros-${ROS_DISTRO}-ament-cmake-ros \
    ros-${ROS_DISTRO}-std-msgs \
    ros-${ROS_DISTRO}-std-srvs \
    ros-${ROS_DISTRO}-example-interfaces \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-ros2launch \
  && rm -rf /var/lib/apt/lists/*
