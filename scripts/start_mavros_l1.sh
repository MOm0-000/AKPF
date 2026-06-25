#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
source /opt/ros/humble/setup.bash
FCU_URL="${FCU_URL:-udp://:14550@}"
GCS_URL="${GCS_URL:-udp://:14551@}"
set -u

ros2 launch mavros node.launch \
  fcu_url:="${FCU_URL}" \
  gcs_url:="${GCS_URL}" \
  tgt_system:=1 \
  tgt_component:=1 \
  pluginlists_yaml:="${PROJECT_DIR}/config/mavros_l1_pluginlists.yaml" \
  config_yaml:=/opt/ros/humble/share/mavros/launch/apm_config.yaml \
  namespace:=mavros
