#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SCENARIO="${SCENARIO:-S1_single_front_obstacle}"
DURATION_S="${DURATION_S:-8}"
QUERY_X="${QUERY_X:-2.00}"
QUERY_Y="${QUERY_Y:-0.00}"
QUERY_Z="${QUERY_Z:-2.00}"
VOXEL_SIZE_M="${VOXEL_SIZE_M:-0.15}"
LOCAL_RADIUS_M="${LOCAL_RADIUS_M:-6.0}"
SAMPLE_STEP_M="${SAMPLE_STEP_M:-0.18}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u

publisher_pid=""
cleanup() {
  if [[ -n "${publisher_pid}" ]] && kill -0 "${publisher_pid}" >/dev/null 2>&1; then
    kill "${publisher_pid}" >/dev/null 2>&1 || true
    wait "${publisher_pid}" >/dev/null 2>&1 || true
  fi
}
trap cleanup EXIT

ros2 run l4_perception_mapping l4_synthetic_cloud_node --ros-args \
  -p scenario:="${SCENARIO}" \
  -p sample_step_m:="${SAMPLE_STEP_M}" &
publisher_pid="$!"

sleep 1

# Local DDS discovery in this WSL setup can lag behind process startup.
# The graph probes and one-shot echo make the demo deterministic without changing runtime nodes.
(ros2 node list --no-daemon --spin-time 5 >/tmp/l4_mapping_node_warmup.log 2>&1 || true) &
(ros2 topic list -t --no-daemon --spin-time 5 >/tmp/l4_mapping_topic_warmup.log 2>&1 || true) &
timeout 6 ros2 topic echo /l4/points --once --no-daemon >/tmp/l4_mapping_cloud_warmup.log 2>&1 || true

set +e
timeout "${DURATION_S}" ros2 run l4_perception_mapping l4_pointcloud_mapper_node --ros-args \
  -p use_odom:=false \
  -p query_x:="${QUERY_X}" \
  -p query_y:="${QUERY_Y}" \
  -p query_z:="${QUERY_Z}" \
  -p voxel_size_m:="${VOXEL_SIZE_M}" \
  -p local_radius_m:="${LOCAL_RADIUS_M}"
mapper_code="$?"
set -e

if [[ "${mapper_code}" == "124" ]]; then
  exit 0
fi

exit "${mapper_code}"
