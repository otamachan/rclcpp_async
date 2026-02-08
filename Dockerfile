FROM ros:jazzy

RUN apt-get update && apt-get install -y --no-install-recommends \
    clang-format \
    clang-tidy \
    ros-jazzy-ament-cmake-clang-format \
    ros-jazzy-ament-cmake-clang-tidy \
    ros-jazzy-ament-lint-auto \
    ros-jazzy-ament-lint-common \
    ros-jazzy-ament-cmake-gtest \
    ros-jazzy-ament-cmake-ros \
    ros-jazzy-std-msgs \
    ros-jazzy-std-srvs \
    ros-jazzy-example-interfaces \
    ros-jazzy-ros2launch \
  && rm -rf /var/lib/apt/lists/*
