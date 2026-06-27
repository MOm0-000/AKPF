#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"
source "${SCRIPT_DIR}/terminal_launcher_common.sh"

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
  bash scripts/open_l0_baseline_terminals.sh

Options:
  --sitl-delay SEC       Delay after opening terminal 2 / SITL. Default: 35
  --backend auto|wt|xterm
  --distro NAME          WSL distro name used by Windows Terminal. Default: current WSL distro or ld666
  --no-cleanup           Do not pre-clean old Gazebo/SITL/MAVROS/L0 processes.
  --cleanup-only         Only clean matching processes/windows, then exit.
  --no-auto-cleanup      Do not clean automatically after offb_node exits.
  --auto-cleanup-delay SEC
                         Delay after offb_node exits before cleanup. Default: 15
  --dry-run              Print terminal commands without opening terminals.
  -h, --help             Show this help.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
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
    *)
      echo "Unknown argument: $1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

cleanup_known_processes() {
  tc_cleanup_common_round TERM "L0"
  tc_kill_processes_with_text 'offb_node'
  sleep 2
  tc_cleanup_common_round 9 "L0"
  tc_kill_processes_with_text 'offb_node' 9
  tc_cleanup_xterms_round TERM "L0"
  sleep 1
  tc_cleanup_xterms_round 9 "L0"
}

print_leftovers() {
  tc_print_common_leftovers "L0"
  tc_print_processes_with_text 'offb_node'
}

if [[ "${PRE_CLEANUP}" == "true" || "${CLEANUP_ONLY}" == "true" ]]; then
  echo "Cleaning old L0/Gazebo/SITL/MAVROS processes..."
  cleanup_known_processes
  echo "Remaining matching processes:"
  print_leftovers
fi
if [[ "${CLEANUP_ONLY}" == "true" ]]; then
  exit 0
fi

tc_select_backend
tc_prepare_run_dir "l0_terminals"

gazebo_cmd=$(cat <<'EOF'
bash -ic 'export LIBGL_ALWAYS_SOFTWARE=1; unset __NV_PRIME_RENDER_OFFLOAD; unset __GLX_VENDOR_LIBRARY_NAME; unset __VK_LAYER_NV_optimus; gz sim -v4 -r ~/ardupilot_gazebo/worlds/iris_runway.sdf'
EOF
)

sitl_cmd=$(cat <<'EOF'
cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550
EOF
)

mavros_cmd=$(cat <<'EOF'
source /opt/ros/humble/setup.bash
ros2 run mavros mavros_node --ros-args \
  -p fcu_url:=udp://:14550@ \
  -p gcs_url:=udp://:14551@
EOF
)

mission_cmd=$(cat <<EOF
if [[ ! -f "\${HOME}/ws_offboard/install/setup.bash" ]]; then
  echo "Missing ~/ws_offboard/install/setup.bash; L0 offboard workspace is not built." >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "\${HOME}/ws_offboard/install/setup.bash"
set +e
ros2 run offboard_control offb_node
status=\$?
set -e
if [[ "${AUTO_CLEANUP}" == "true" ]]; then
  echo "offb_node exited; cleanup in ${AUTO_CLEANUP_DELAY_S}s..."
  sleep "${AUTO_CLEANUP_DELAY_S}"
  bash "${SCRIPT_PATH}" --cleanup-only
else
  echo "Auto cleanup disabled."
fi
exit \${status}
EOF
)

tc_launch_terminal "L0 T1 Gazebo runway GUI" "${gazebo_cmd}"
tc_sleep_between "${GAZEBO_DELAY_S}"
tc_launch_terminal "L0 T2 SITL ArduCopter" "${sitl_cmd}"
tc_sleep_between "${SITL_DELAY_S}"
tc_launch_terminal "L0 T3 MAVROS" "${mavros_cmd}"
tc_sleep_between "${MAVROS_DELAY_S}"
tc_launch_terminal "L0 T4 offb_node mission" "${mission_cmd}"

echo "L0 terminals opened."
