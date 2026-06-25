#!/usr/bin/env bash
set -eo pipefail

POINT_CLOUD_TOPIC="${POINT_CLOUD_TOPIC:-/l4/depth_camera/points}"
ROS_POINT_CLOUD_TOPIC="${ROS_POINT_CLOUD_TOPIC:-${POINT_CLOUD_TOPIC}}"
BRIDGE_IMPL="${BRIDGE_IMPL:-native}"
REPACK_XYZ="${REPACK_XYZ:-true}"
SAMPLE_STEP="${SAMPLE_STEP:-4}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

source /opt/ros/humble/setup.bash
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "${PROJECT_DIR}/install/setup.bash" ]]; then
  source "${PROJECT_DIR}/install/setup.bash"
fi
set -u

if [[ "${BRIDGE_IMPL}" == "native" ]]; then
  echo "Bridging Gazebo -> ROS with l4_gz_pointcloud_bridge_node"
  echo "Gazebo topic: ${POINT_CLOUD_TOPIC}"
  echo "ROS2 topic:   ${ROS_POINT_CLOUD_TOPIC}"
  echo "Repack XYZ:   ${REPACK_XYZ}, sample_step=${SAMPLE_STEP}"
  exec ros2 run l4_perception_mapping l4_gz_pointcloud_bridge_node --ros-args \
    -p gz_topic:="${POINT_CLOUD_TOPIC}" \
    -p ros_topic:="${ROS_POINT_CLOUD_TOPIC}" \
    -p repack_xyz:="${REPACK_XYZ}" \
    -p sample_step:="${SAMPLE_STEP}"
fi

if ! ros2 pkg prefix ros_gz_bridge >/dev/null 2>&1; then
  echo "ros_gz_bridge is not installed or not discoverable." >&2
  echo "Install it in a terminal with:" >&2
  echo "  sudo apt-get install -y ros-humble-ros-gz-bridge" >&2
  exit 2
fi

echo "Bridging Gazebo -> ROS with ros_gz_bridge on ${POINT_CLOUD_TOPIC}"
echo "ROS2 mapper should subscribe to ${ROS_POINT_CLOUD_TOPIC}"
exec ros2 run ros_gz_bridge parameter_bridge   "${POINT_CLOUD_TOPIC}@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
