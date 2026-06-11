#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

source /opt/ros/humble/setup.bash
set -u

ros2 launch mavros node.launch \
  fcu_url:=udp://:14550@ \
  gcs_url:=udp://:14551@ \
  tgt_system:=1 \
  tgt_component:=1 \
  pluginlists_yaml:="${PROJECT_DIR}/config/mavros_l1_pluginlists.yaml" \
  config_yaml:=/opt/ros/humble/share/mavros/launch/apm_config.yaml \
  namespace:=mavros
