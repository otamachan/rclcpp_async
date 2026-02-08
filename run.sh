#!/usr/bin/env bash
set -e

REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"
ROS_DISTRO="${ROS_DISTRO:-jazzy}"
IMAGE_NAME="rclcpp_async:${ROS_DISTRO}"

if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
  echo "Image '$IMAGE_NAME' not found. Building..."
  docker build --build-arg "ROS_DISTRO=${ROS_DISTRO}" -t "$IMAGE_NAME" "$REPO_ROOT"
fi

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "$REPO_ROOT:$REPO_ROOT" \
  -w "$REPO_ROOT/ws" \
  -e HOME=/tmp/home \
  -e "ROS_DISTRO=${ROS_DISTRO}" \
  "$IMAGE_NAME" \
  "$@"
