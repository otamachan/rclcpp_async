#!/usr/bin/env bash
set -e

REPO_ROOT="$(cd "$(dirname "$0")" && pwd)"
IMAGE_NAME="rclcpp_async"

if ! docker image inspect "$IMAGE_NAME" &>/dev/null; then
  echo "Image '$IMAGE_NAME' not found. Building..."
  docker build -t "$IMAGE_NAME" "$REPO_ROOT"
fi

docker run --rm \
  --user "$(id -u):$(id -g)" \
  -v "$REPO_ROOT:$REPO_ROOT" \
  -w "$REPO_ROOT/ws" \
  -e HOME=/tmp/home \
  "$IMAGE_NAME" \
  "$@"
