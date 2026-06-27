#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
source "${SCRIPT_DIR}/terminal_launcher_common.sh"

SCENE="S1"
SITL_DELAY_S=35
GAZEBO_DELAY_S=8
MAVROS_DELAY_S=8
BRIDGE_DELAY_S=4
MAPPER_DELAY_S=4
SHIELD_DELAY_S=3
BACKEND="auto"
DRY_RUN=false
PRE_CLEANUP=true
CLEANUP_ONLY=false
DISTRO="${WSL_DISTRO_NAME:-ld666}"
AUTO_CLEANUP=true
AUTO_CLEANUP_DELAY_S=15

usage() {
  cat <<'EOF'
Usage:
  bash scripts/open_l5_shield_terminals.sh [S1|S2|S3|S4|S5]

Options:
  --scenario SCENE       Scenario key or full scenario name. Default: S1
  --sitl-delay SEC       Delay after opening terminal 2 / SITL. Default: 35
  --backend auto|wt|xterm
  --distro NAME          WSL distro name used by Windows Terminal. Default: current WSL distro or ld666
  --no-cleanup           Do not pre-clean old Gazebo/SITL/MAVROS/L5 processes.
  --cleanup-only         Only clean matching processes/windows, then exit.
  --no-auto-cleanup      Do not clean automatically after L5 AKPF node exits.
  --auto-cleanup-delay SEC
                         Delay after L5 AKPF node exits before cleanup. Default: 15
  --dry-run              Print terminal commands without opening terminals.
  -h, --help             Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --scenario)
      SCENE="${2:?missing scenario}"
      shift 2
      ;;
    --sitl-delay)
      SITL_DELAY_S="${2:?missing seconds}"
      shift 2
      ;;
    --backend)
      BACKEND="${2:?missing backend}"
      shift 2
      ;;
    --distro)
      DISTRO="${2:?missing distro}"
      shift 2
      ;;
    --no-cleanup)
      PRE_CLEANUP=false
      shift
      ;;
    --cleanup-only)
      CLEANUP_ONLY=true
      shift
      ;;
    --no-auto-cleanup)
      AUTO_CLEANUP=false
      shift
      ;;
    --auto-cleanup-delay|--post-end-cleanup-delay)
      AUTO_CLEANUP_DELAY_S="${2:?missing seconds}"
      shift 2
      ;;
    --dry-run)
      DRY_RUN=true
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    S1|S2|S3|S4|S5|S1_*|S2_*|S3_*|S4_*|S5_*)
      SCENE="$1"
      shift
      ;;
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

case "${SCENE}" in
  S1|S1_single_front_obstacle) WORLD_FILE="S1_depth_camera.sdf"; SCENARIO_NAME="S1_single_front_obstacle" ;;
  S2|S2_narrow_gate) WORLD_FILE="S2_depth_camera.sdf"; SCENARIO_NAME="S2_narrow_gate" ;;
  S3|S3_corridor) WORLD_FILE="S3_depth_camera.sdf"; SCENARIO_NAME="S3_corridor" ;;
  S4|S4_table_or_low_obstacle) WORLD_FILE="S4_depth_camera.sdf"; SCENARIO_NAME="S4_table_or_low_obstacle" ;;
  S5|S5_corner) WORLD_FILE="S5_depth_camera.sdf"; SCENARIO_NAME="S5_corner" ;;
  *)
    echo "Unknown L5 scenario: ${SCENE}" >&2
    exit 2
    ;;
esac

cleanup_known_processes() {
  tc_cleanup_common_round TERM "L5"
  tc_kill_processes_with_text 'l4_gz_pointcloud_bridge_node'
  tc_kill_processes_with_text 'l4_pointcloud_mapper_node'
  tc_kill_processes_with_text 'l5_safety_shield_node'
  tc_kill_processes_with_text 'l3_akpf_node'
  sleep 2
  tc_cleanup_common_round 9 "L5"
  tc_kill_processes_with_text 'l4_gz_pointcloud_bridge_node' 9
  tc_kill_processes_with_text 'l4_pointcloud_mapper_node' 9
  tc_kill_processes_with_text 'l5_safety_shield_node' 9
  tc_kill_processes_with_text 'l3_akpf_node' 9
  tc_cleanup_xterms_round TERM "L5"
  sleep 1
  tc_cleanup_xterms_round 9 "L5"
}

print_leftovers() {
  tc_print_common_leftovers "L5"
  tc_print_processes_with_text 'l4_gz_pointcloud_bridge_node'
  tc_print_processes_with_text 'l4_pointcloud_mapper_node'
  tc_print_processes_with_text 'l5_safety_shield_node'
  tc_print_processes_with_text 'l3_akpf_node'
}

if [[ "${PRE_CLEANUP}" == "true" || "${CLEANUP_ONLY}" == "true" ]]; then
  echo "Cleaning old L5/Gazebo/SITL/MAVROS processes..."
  cleanup_known_processes
  echo "Remaining matching processes:"
  print_leftovers
fi
if [[ "${CLEANUP_ONLY}" == "true" ]]; then
  exit 0
fi

tc_select_backend
tc_prepare_run_dir "l5_terminals_${SCENARIO_NAME}"

gazebo_cmd=$(cat <<EOF
cd "${PROJECT_DIR}"
export GZ_SIM_RESOURCE_PATH="\$PWD/models:\$HOME/ardupilot_gazebo/models:\$HOME/ardupilot_gazebo/worlds:\${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="\$HOME/ardupilot_gazebo/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE=1
unset __NV_PRIME_RENDER_OFFLOAD || true
unset __GLX_VENDOR_LIBRARY_NAME || true
unset __VK_LAYER_NV_optimus || true
gz sim -v4 -r "worlds/l4/${WORLD_FILE}"
EOF
)

sitl_cmd=$(cat <<'EOF'
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
EOF
)

mavros_cmd=$(cat <<EOF
cd "${PROJECT_DIR}"
./scripts/start_mavros_l1.sh
EOF
)

bridge_cmd=$(cat <<EOF
cd "${PROJECT_DIR}"
./scripts/start_l4_pointcloud_bridge.sh
EOF
)

mapper_cmd=$(cat <<EOF
cd "${PROJECT_DIR}"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run l4_perception_mapping l4_pointcloud_mapper_node --ros-args \
  -p pointcloud_topic:=/l4/depth_camera/points \
  -p odom_topic:=/mavros/local_position/local \
  -p pose_topic:=/mavros/local_position/pose \
  -p use_odom:=true \
  -p transform_to_odom_frame:=true \
  -p output_frame_id:=map \
  -p query_rate_hz:=10.0 \
  -p local_radius_m:=6.0 \
  -p voxel_size_m:=0.15 \
  -p map_memory_s:=90.0 \
  -p max_map_voxels:=80000 \
  -p camera_offset_x:=0.22 \
  -p camera_offset_y:=0.0 \
  -p camera_offset_z:=0.06
EOF
)

shield_cmd=$(cat <<EOF
cd "${PROJECT_DIR}"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run l5_safety_shield l5_safety_shield_node --ros-args \
  -p raw_cmd_topic:=/l5/raw_cmd_vel \
  -p safe_cmd_topic:=/mavros/setpoint_velocity/cmd_vel \
  -p nearest_distance_topic:=/l4/nearest_distance \
  -p nearest_normal_topic:=/l4/nearest_normal \
  -p odom_topic:=/mavros/local_position/local \
  -p state_topic:=/mavros/state \
  -p map_timeout_s:=1.0 \
  -p cmd_timeout_s:=0.5 \
  -p body_radius:=0.35 \
  -p safety_margin:=0.15 \
  -p stop_d_eff:=0.08 \
  -p caution_d_eff:=0.85 \
  -p toward_speed_gain:=0.8 \
  -p max_xy_speed:=0.35 \
  -p max_z_speed:=0.22 \
  -p min_altitude:=1.2 \
  -p max_altitude:=2.6
EOF
)

mission_cmd=$(cat <<EOF
cd "${PROJECT_DIR}"
if [[ ! -x "install/l5_safety_shield/lib/l5_safety_shield/l5_safety_shield_node" ]]; then
  echo "Missing L5 build. Run: source /opt/ros/humble/setup.bash && colcon build --packages-select l3_akpf_navigation l4_perception_mapping l5_safety_shield" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
set +e
ros2 run l3_akpf_navigation l3_akpf_node --ros-args \
  -p scenario:="${SCENARIO_NAME}" \
  -p mission_timeout_s:=220.0 \
  -p use_stamped_cmd_vel:=true \
  -p stamped_cmd_vel_topic:=/l5/raw_cmd_vel \
  -p odom_topic:=/mavros/local_position/local \
  -p pose_topic:=/mavros/local_position/pose \
  -p distance_source:=perception_map \
  -p perception_cloud_topic:=/l4/local_cloud \
  -p perception_fallback_to_truth:=false \
  -p enable_local_target:=false \
  -p max_xy_speed:=0.35 \
  -p candidate_safe_margin:=0.15 \
  -p candidate_comfort_margin:=0.50 \
  -p local_target_clearance:=0.15 \
  -p emergency_d_eff:=0.35 \
  -p recovery_exit_d_eff:=0.85 \
  -p recovery_exit_path_margin:=0.15 \
  -p recovery_clear_exit_d_eff:=1.70 \
  -p recovery_min_duration_s:=3.0 \
  -p recovery_xy_speed:=0.22 \
  -p recovery_climb_speed:=0.12
status=\$?
set -e
if [[ "${AUTO_CLEANUP}" == "true" ]]; then
  echo "L5 AKPF node exited; cleanup in ${AUTO_CLEANUP_DELAY_S}s..."
  sleep "${AUTO_CLEANUP_DELAY_S}"
  bash "${SCRIPT_PATH}" --cleanup-only
else
  echo "Auto cleanup disabled."
fi
exit \${status}
EOF
)

tc_launch_terminal "L5 T1 Gazebo ${SCENARIO_NAME}" "${gazebo_cmd}"
tc_sleep_between "${GAZEBO_DELAY_S}"
tc_launch_terminal "L5 T2 SITL ArduCopter" "${sitl_cmd}"
tc_sleep_between "${SITL_DELAY_S}"
tc_launch_terminal "L5 T3 MAVROS" "${mavros_cmd}"
tc_sleep_between "${MAVROS_DELAY_S}"
tc_launch_terminal "L5 T4 pointcloud bridge" "${bridge_cmd}"
tc_sleep_between "${BRIDGE_DELAY_S}"
tc_launch_terminal "L5 T5 local mapper" "${mapper_cmd}"
tc_sleep_between "${MAPPER_DELAY_S}"
tc_launch_terminal "L5 T6A safety shield" "${shield_cmd}"
tc_sleep_between "${SHIELD_DELAY_S}"
tc_launch_terminal "L5 T6B perception AKPF ${SCENARIO_NAME}" "${mission_cmd}"

echo "L5 terminals opened for ${SCENARIO_NAME}."
