#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

SCENARIO="${SCENARIO:-S1_single_front_obstacle}"
MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S:-0.0}"
USE_STAMPED_CMD_VEL="${USE_STAMPED_CMD_VEL:-true}"
ENABLE_REPULSION="${ENABLE_REPULSION:-true}"
ENABLE_KINO="${ENABLE_KINO:-true}"
ENABLE_CURL="${ENABLE_CURL:-true}"
ENABLE_AERO="${ENABLE_AERO:-true}"
ENABLE_LOCAL_TARGET="${ENABLE_LOCAL_TARGET:-true}"
ANTI_RETREAT_MARGIN="${ANTI_RETREAT_MARGIN:-0.18}"
ANTI_RETREAT_PROGRESS="${ANTI_RETREAT_PROGRESS:--0.05}"
ANTI_RETREAT_PENALTY="${ANTI_RETREAT_PENALTY:-8.0}"
RECOVERY_PROGRESS_FLOOR="${RECOVERY_PROGRESS_FLOOR:-0.05}"
CANDIDATE_SAFE_MARGIN="${CANDIDATE_SAFE_MARGIN:-0.02}"
CANDIDATE_COMFORT_MARGIN="${CANDIDATE_COMFORT_MARGIN:-0.38}"
LOCAL_TARGET_CLEARANCE="${LOCAL_TARGET_CLEARANCE:-0.08}"
LOCAL_TARGET_INFLATE_EXTRA="${LOCAL_TARGET_INFLATE_EXTRA:-0.25}"
LOCAL_TARGET_HOLD_RADIUS="${LOCAL_TARGET_HOLD_RADIUS:-0.65}"
LOCAL_TARGET_MIN_ADVANCE="${LOCAL_TARGET_MIN_ADVANCE:-0.85}"
LOCAL_TARGET_REPULSION_SCALE="${LOCAL_TARGET_REPULSION_SCALE:-0.0}"
LOCAL_TARGET_PROGRESS_WEIGHT="${LOCAL_TARGET_PROGRESS_WEIGHT:-0.55}"
LOCAL_TARGET_SAMPLE_STEP="${LOCAL_TARGET_SAMPLE_STEP:-0.20}"
PROGRESS_STALL_TIMEOUT_S="${PROGRESS_STALL_TIMEOUT_S:-8.0}"
PROGRESS_STALL_EPSILON="${PROGRESS_STALL_EPSILON:-0.20}"

case "${MISSION_TIMEOUT_S}" in
  *.*) ;;
  *) MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S}.0" ;;
esac

source /opt/ros/humble/setup.bash
source "${PROJECT_DIR}/install/setup.bash"
set -u

# Local FastDDS discovery can be slow in this WSL setup.
(ros2 node list --no-daemon --spin-time 10 >/tmp/l3_akpf_node_warmup.log 2>&1 || true) &
(ros2 topic list -t --no-daemon --spin-time 10 >/tmp/l3_akpf_topic_warmup.log 2>&1 || true) &

exec ros2 run l3_akpf_navigation l3_akpf_node --ros-args \
  -p scenario:="${SCENARIO}" \
  -p mission_timeout_s:="${MISSION_TIMEOUT_S}" \
  -p use_stamped_cmd_vel:="${USE_STAMPED_CMD_VEL}" \
  -p enable_repulsion:="${ENABLE_REPULSION}" \
  -p enable_kinodynamic:="${ENABLE_KINO}" \
  -p enable_curl:="${ENABLE_CURL}" \
  -p enable_aero:="${ENABLE_AERO}" \
  -p enable_local_target:="${ENABLE_LOCAL_TARGET}" \
  -p anti_retreat_margin:="${ANTI_RETREAT_MARGIN}" \
  -p anti_retreat_progress:="${ANTI_RETREAT_PROGRESS}" \
  -p anti_retreat_penalty:="${ANTI_RETREAT_PENALTY}" \
  -p recovery_progress_floor:="${RECOVERY_PROGRESS_FLOOR}" \
  -p candidate_safe_margin:="${CANDIDATE_SAFE_MARGIN}" \
  -p candidate_comfort_margin:="${CANDIDATE_COMFORT_MARGIN}" \
  -p local_target_clearance:="${LOCAL_TARGET_CLEARANCE}" \
  -p local_target_inflate_extra:="${LOCAL_TARGET_INFLATE_EXTRA}" \
  -p local_target_hold_radius:="${LOCAL_TARGET_HOLD_RADIUS}" \
  -p local_target_min_advance:="${LOCAL_TARGET_MIN_ADVANCE}" \
  -p local_target_repulsion_scale:="${LOCAL_TARGET_REPULSION_SCALE}" \
  -p local_target_progress_weight:="${LOCAL_TARGET_PROGRESS_WEIGHT}" \
  -p local_target_sample_step:="${LOCAL_TARGET_SAMPLE_STEP}" \
  -p progress_stall_timeout_s:="${PROGRESS_STALL_TIMEOUT_S}" \
  -p progress_stall_epsilon:="${PROGRESS_STALL_EPSILON}"
