#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export POINTCLOUD_TOPIC="${POINTCLOUD_TOPIC:-/l4/depth_camera/points}"
export ODOM_TOPIC="${ODOM_TOPIC:-/mavros/local_position/local}"
export POSE_TOPIC="${POSE_TOPIC:-/mavros/local_position/pose}"
export USE_ODOM="${USE_ODOM:-true}"
export TRANSFORM_TO_ODOM_FRAME="${TRANSFORM_TO_ODOM_FRAME:-true}"
export OUTPUT_FRAME_ID="${OUTPUT_FRAME_ID:-map}"
export QUERY_RATE_HZ="${QUERY_RATE_HZ:-10.0}"
export LOCAL_RADIUS_M="${LOCAL_RADIUS_M:-6.0}"
export VOXEL_SIZE_M="${VOXEL_SIZE_M:-0.15}"

exec "${SCRIPT_DIR}/run_l4_gazebo_mapper.sh"
