#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S:-180.0}"
USE_STAMPED_CMD_VEL="${USE_STAMPED_CMD_VEL:-true}"

source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u

# Local FastDDS discovery can be slow in this WSL setup.
# Keeping no-daemon graph probes alive during startup reliably warms discovery.
(ros2 node list --no-daemon --spin-time 10 >/tmp/l1_velocity_node_warmup.log 2>&1 || true) &
(ros2 topic list -t --no-daemon --spin-time 10 >/tmp/l1_velocity_topic_warmup.log 2>&1 || true) &

exec ros2 run l1_velocity_control l1_velocity_node --ros-args \
  -p use_stamped_cmd_vel:="${USE_STAMPED_CMD_VEL}" \
  -p mission_timeout_s:="${MISSION_TIMEOUT_S}"
