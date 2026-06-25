#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/l4/depth_camera/points}"
ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/local}"
POSE_TOPIC="${POSE_TOPIC:-/mavros/local_position/pose}"
USE_ODOM="${USE_ODOM:-false}"
QUERY_X="${QUERY_X:-0.0}"
QUERY_Y="${QUERY_Y:-0.0}"
QUERY_Z="${QUERY_Z:-0.0}"
VOXEL_SIZE_M="${VOXEL_SIZE_M:-0.15}"
LOCAL_RADIUS_M="${LOCAL_RADIUS_M:-6.0}"
QUERY_RATE_HZ="${QUERY_RATE_HZ:-10.0}"
TRANSFORM_TO_ODOM_FRAME="${TRANSFORM_TO_ODOM_FRAME:-false}"
OUTPUT_FRAME_ID="${OUTPUT_FRAME_ID:-map}"
CAMERA_OFFSET_X="${CAMERA_OFFSET_X:-0.22}"
CAMERA_OFFSET_Y="${CAMERA_OFFSET_Y:-0.0}"
CAMERA_OFFSET_Z="${CAMERA_OFFSET_Z:-0.06}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u

(ros2 node list --no-daemon --spin-time 5 >/tmp/l4_gazebo_mapper_node_warmup.log 2>&1 || true) &
(ros2 topic list -t --no-daemon --spin-time 5 >/tmp/l4_gazebo_mapper_topic_warmup.log 2>&1 || true) &

exec ros2 run l4_perception_mapping l4_pointcloud_mapper_node --ros-args \
  -p pointcloud_topic:="${POINTCLOUD_TOPIC}" \
  -p odom_topic:="${ODOM_TOPIC}" \
  -p pose_topic:="${POSE_TOPIC}" \
  -p use_odom:="${USE_ODOM}" \
  -p query_x:="${QUERY_X}" \
  -p query_y:="${QUERY_Y}" \
  -p query_z:="${QUERY_Z}" \
  -p voxel_size_m:="${VOXEL_SIZE_M}" \
  -p local_radius_m:="${LOCAL_RADIUS_M}" \
  -p query_rate_hz:="${QUERY_RATE_HZ}" \
  -p transform_to_odom_frame:="${TRANSFORM_TO_ODOM_FRAME}" \
  -p output_frame_id:="${OUTPUT_FRAME_ID}" \
  -p camera_offset_x:="${CAMERA_OFFSET_X}" \
  -p camera_offset_y:="${CAMERA_OFFSET_Y}" \
  -p camera_offset_z:="${CAMERA_OFFSET_Z}"
