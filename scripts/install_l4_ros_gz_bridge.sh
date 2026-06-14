#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash

if ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  echo "ros_gz_bridge already installed: $(ros2 pkg prefix ros_gz_bridge)"
  exit 0
fi

echo "Installing ros-humble-ros-gz-bridge..."
sudo apt-get update
sudo apt-get install -y ros-humble-ros-gz-bridge

source /opt/ros/humble/setup.bash
ros2 pkg prefix ros_gz_bridge
