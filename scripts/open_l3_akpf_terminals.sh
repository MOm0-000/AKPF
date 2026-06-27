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
  bash scripts/open_l3_akpf_terminals.sh [S1|S2|S3|S4|S5|S6|S8|S9]

Options:
  --scenario SCENE       Scenario key or full scenario name. Default: S1
  --sitl-delay SEC       Delay after opening terminal 2 / SITL. Default: 35
  --backend auto|wt|xterm
  --distro NAME          WSL distro name used by Windows Terminal. Default: current WSL distro or ld666
  --no-cleanup           Do not pre-clean old Gazebo/SITL/MAVROS/L3 processes.
  --cleanup-only         Only clean matching processes/windows, then exit.
  --no-auto-cleanup      Do not clean automatically after L3 node exits.
  --auto-cleanup-delay SEC
                         Delay after L3 node exits before cleanup. Default: 15
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
    S1|S2|S3|S4|S5|S6|S8|S9|S1_*|S2_*|S3_*|S4_*|S5_*|S6_*|S8_*|S9_*)
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
  S1|S1_single_front_obstacle) SCENARIO_NAME="S1_single_front_obstacle"; MISSION_TIMEOUT_S="120" ;;
  S2|S2_narrow_gate) SCENARIO_NAME="S2_narrow_gate"; MISSION_TIMEOUT_S="140" ;;
  S3|S3_corridor) SCENARIO_NAME="S3_corridor"; MISSION_TIMEOUT_S="140" ;;
  S4|S4_table_or_low_obstacle) SCENARIO_NAME="S4_table_or_low_obstacle"; MISSION_TIMEOUT_S="140" ;;
  S5|S5_corner) SCENARIO_NAME="S5_corner"; MISSION_TIMEOUT_S="160" ;;
  S6|S6_cluttered_boxes) SCENARIO_NAME="S6_cluttered_boxes"; MISSION_TIMEOUT_S="150" ;;
  S8|S8_vertical_constraint) SCENARIO_NAME="S8_vertical_constraint"; MISSION_TIMEOUT_S="140" ;;
  S9|S9_multi_corner) SCENARIO_NAME="S9_multi_corner"; MISSION_TIMEOUT_S="160" ;;
  *)
    echo "Unknown L3 scenario: ${SCENE}" >&2
    exit 2
    ;;
esac

cleanup_known_processes() {
  tc_cleanup_common_round TERM "L3"
  tc_kill_processes_with_text 'l3_akpf_node'
  sleep 2
  tc_cleanup_common_round 9 "L3"
  tc_kill_processes_with_text 'l3_akpf_node' 9
  tc_cleanup_xterms_round TERM "L3"
  sleep 1
  tc_cleanup_xterms_round 9 "L3"
}

print_leftovers() {
  tc_print_common_leftovers "L3"
  tc_print_processes_with_text 'l3_akpf_node'
}

if [[ "${PRE_CLEANUP}" == "true" || "${CLEANUP_ONLY}" == "true" ]]; then
  echo "Cleaning old L3/Gazebo/SITL/MAVROS processes..."
  cleanup_known_processes
  echo "Remaining matching processes:"
  print_leftovers
fi
if [[ "${CLEANUP_ONLY}" == "true" ]]; then
  exit 0
fi

tc_select_backend
tc_prepare_run_dir "l3_terminals_${SCENARIO_NAME}"

gazebo_cmd=$(cat <<EOF
cd "${PROJECT_DIR}"
GZ_SERVER_ONLY=false ./scripts/start_l2_world.sh "${SCENARIO_NAME}"
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

mission_cmd=$(cat <<EOF
cd "${PROJECT_DIR}"
if [[ ! -x "install/l3_akpf_navigation/lib/l3_akpf_navigation/l3_akpf_node" ]]; then
  echo "Missing L3 build. Run: source /opt/ros/humble/setup.bash && colcon build --packages-select l3_akpf_navigation" >&2
  exit 2
fi
set +e
SCENARIO="${SCENARIO_NAME}" MISSION_TIMEOUT_S="${MISSION_TIMEOUT_S}" ./scripts/run_l3_akpf.sh
status=\$?
set -e
if [[ "${AUTO_CLEANUP}" == "true" ]]; then
  echo "L3 node exited; cleanup in ${AUTO_CLEANUP_DELAY_S}s..."
  sleep "${AUTO_CLEANUP_DELAY_S}"
  bash "${SCRIPT_PATH}" --cleanup-only
else
  echo "Auto cleanup disabled."
fi
exit \${status}
EOF
)

tc_launch_terminal "L3 T1 Gazebo ${SCENARIO_NAME}" "${gazebo_cmd}"
tc_sleep_between "${GAZEBO_DELAY_S}"
tc_launch_terminal "L3 T2 SITL ArduCopter" "${sitl_cmd}"
tc_sleep_between "${SITL_DELAY_S}"
tc_launch_terminal "L3 T3 MAVROS" "${mavros_cmd}"
tc_sleep_between "${MAVROS_DELAY_S}"
tc_launch_terminal "L3 T4 AKPF ${SCENARIO_NAME}" "${mission_cmd}"

echo "L3 terminals opened for ${SCENARIO_NAME}."
