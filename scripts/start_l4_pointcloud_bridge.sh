#!/usr/bin/env bash
set -eo pipefail

POINT_CLOUD_TOPIC="${POINT_CLOUD_TOPIC:-/l4/depth_camera/points}"

source /opt/ros/humble/setup.bash
set -u

if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  echo "ros_gz_bridge is not installed or not discoverable." >&2
  echo "Install it in a terminal with:" >&2
  echo "  sudo apt-get install -y ros-humble-ros-gz-bridge" >&2
  exit 2
fi

echo "Bridging Gazebo -> ROS on ${POINT_CLOUD_TOPIC}"
echo "ROS2 mapper should subscribe to ${POINT_CLOUD_TOPIC}"
exec ros2 run ros_gz_bridge parameter_bridge   "${POINT_CLOUD_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
