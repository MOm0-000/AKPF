#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SCRIPT_PATH="$(readlink -f "${BASH_SOURCE[0]}")"

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
AUTO_CLEANUP_TIMEOUT_S=360
MONITOR_POLL_S=2.0

usage() {
  cat <<'EOF'
Usage:
  bash scripts/open_l7_1_perception_terminals.sh [S1|S2|S3|S4|S5]

Options:
  --scenario SCENE       Scenario key or full scenario name. Default: S1
  --sitl-delay SEC       Delay after opening terminal 2 / SITL. Default: 35
  --backend auto|wt|xterm
  --distro NAME          WSL distro name used by Windows Terminal. Default: current WSL distro or ld666
  --no-cleanup           Do not pre-clean old Gazebo/SITL/MAVROS/L4/L5/L7 processes.
  --cleanup-only         Only clean matching processes/windows, then exit.
  --no-auto-cleanup      Do not clean automatically after landing/crash.
  --auto-cleanup-delay SEC
                         Delay after landing/crash before cleanup. Default: 15
  --auto-cleanup-timeout SEC
                         Cleanup if no terminal event is detected by this time. Default: 360
  --dry-run              Print terminal commands without opening terminals.

Examples:
  bash scripts/open_l7_1_perception_terminals.sh S5
  bash scripts/open_l7_1_perception_terminals.sh --scenario S2 --sitl-delay 45
EOF
}

wt_usable() {
  command -v wt.exe >/dev/null 2>&1 || return 1
  [[ -e /proc/sys/fs/binfmt_misc/WSLInterop ]] || return 1
  timeout 3s wt.exe --version >/dev/null 2>&1 || return 1
}

xterm_usable() {
  command -v xterm >/dev/null 2>&1 || return 1
  [[ -n "${DISPLAY:-}" ]] || return 1
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
    --auto-cleanup-timeout)
      AUTO_CLEANUP_TIMEOUT_S="${2:?missing seconds}"
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
  S1|S1_single_front_obstacle)
    WORLD_FILE="S1_depth_camera.sdf"
    SCENARIO_NAME="S1_single_front_obstacle"
    ;;
  S2|S2_narrow_gate)
    WORLD_FILE="S2_depth_camera.sdf"
    SCENARIO_NAME="S2_narrow_gate"
    ;;
  S3|S3_corridor)
    WORLD_FILE="S3_depth_camera.sdf"
    SCENARIO_NAME="S3_corridor"
    ;;
  S4|S4_table_or_low_obstacle)
    WORLD_FILE="S4_depth_camera.sdf"
    SCENARIO_NAME="S4_table_or_low_obstacle"
    ;;
  S5|S5_corner)
    WORLD_FILE="S5_depth_camera.sdf"
    SCENARIO_NAME="S5_corner"
    ;;
  *)
    echo "Unknown L7.1 scenario: ${SCENE}" >&2
    echo "Allowed: S1, S2, S3, S4, S5 or full scenario names." >&2
    exit 2
    ;;
esac

PERCEPTION_POLICY="${PROJECT_DIR}/artifacts/tmp_archive_20260627/l6_training/l6_l63_perception_mixed_bc/policy_best.pt"
if [[ ! -f "${PERCEPTION_POLICY}" ]]; then
  echo "Missing perception policy: ${PERCEPTION_POLICY}" >&2
  exit 2
fi

xterm_pids_with_text() {
  local text="$1"
  while read -r pid comm args; do
    if [[ "${comm}" == "xterm" && "${args}" == *"${text}"* ]]; then
      printf '%s\n' "${pid}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

kill_xterms_with_text() {
  local text="$1"
  local signal="${2:-TERM}"
  local -a pids=()
  mapfile -t pids < <(xterm_pids_with_text "${text}")
  if ((${#pids[@]})); then
    if [[ "${signal}" == "TERM" ]]; then
      kill "${pids[@]}" 2>/dev/null || true
    else
      kill "-${signal}" "${pids[@]}" 2>/dev/null || true
    fi
  fi
}

print_xterms_with_text() {
  local text="$1"
  while read -r pid comm args; do
    if [[ "${comm}" == "xterm" && "${args}" == *"${text}"* ]]; then
      printf '%s %s %s\n' "${pid}" "${comm}" "${args}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

process_pids_with_text() {
  local text="$1"
  while read -r pid comm args; do
    case "${comm}" in
      bash|sh|dash|zsh|fish|grep|awk|sed|timeout)
        continue
        ;;
    esac
    if [[ "${args}" == *"${text}"* ]]; then
      printf '%s\n' "${pid}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

kill_processes_with_text() {
  local text="$1"
  local signal="${2:-TERM}"
  local -a pids=()
  mapfile -t pids < <(process_pids_with_text "${text}")
  if ((${#pids[@]})); then
    if [[ "${signal}" == "TERM" ]]; then
      kill "${pids[@]}" 2>/dev/null || true
    else
      kill "-${signal}" "${pids[@]}" 2>/dev/null || true
    fi
  fi
}

print_processes_with_text() {
  local text="$1"
  while read -r pid comm args; do
    case "${comm}" in
      bash|sh|dash|zsh|fish|grep|awk|sed|timeout)
        continue
        ;;
    esac
    if [[ "${args}" == *"${text}"* ]]; then
      printf '%s %s %s\n' "${pid}" "${comm}" "${args}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

process_pids_with_comm_and_text() {
  local comm_filter="$1"
  local text="$2"
  while read -r pid comm args; do
    if [[ "${comm}" == "${comm_filter}" && "${args}" == *"${text}"* ]]; then
      printf '%s\n' "${pid}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

kill_processes_with_comm_and_text() {
  local comm_filter="$1"
  local text="$2"
  local signal="${3:-TERM}"
  local -a pids=()
  mapfile -t pids < <(process_pids_with_comm_and_text "${comm_filter}" "${text}")
  if ((${#pids[@]})); then
    if [[ "${signal}" == "TERM" ]]; then
      kill "${pids[@]}" 2>/dev/null || true
    else
      kill "-${signal}" "${pids[@]}" 2>/dev/null || true
    fi
  fi
}

print_processes_with_comm_and_text() {
  local comm_filter="$1"
  local text="$2"
  while read -r pid comm args; do
    if [[ "${comm}" == "${comm_filter}" && "${args}" == *"${text}"* ]]; then
      printf '%s %s %s\n' "${pid}" "${comm}" "${args}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

cleanup_known_processes() {
  pkill -x arducopter 2>/dev/null || true
  pkill -x mavproxy.py 2>/dev/null || true
  pkill -x mavros_node 2>/dev/null || true
  kill_processes_with_text 'gz sim'
  kill_processes_with_comm_and_text 'ruby' 'gz'
  kill_processes_with_text 'sim_vehicle.py'
  kill_processes_with_text 'l4_gz_pointcloud_bridge_node'
  kill_processes_with_text 'l4_pointcloud_mapper_node'
  kill_processes_with_text 'l5_safety_shield_node'
  kill_processes_with_text 'l7_policy_node'
  kill_xterms_with_text "L7.1"
  kill_xterms_with_text "ArduCopter"
  kill_xterms_with_text "MAVProxy"
  sleep 2
  pkill -9 -x arducopter 2>/dev/null || true
  pkill -9 -x mavproxy.py 2>/dev/null || true
  pkill -9 -x mavros_node 2>/dev/null || true
  kill_processes_with_text 'gz sim' "9"
  kill_processes_with_comm_and_text 'ruby' 'gz' "9"
  kill_processes_with_text 'sim_vehicle.py' "9"
  kill_processes_with_text 'l4_gz_pointcloud_bridge_node' "9"
  kill_processes_with_text 'l4_pointcloud_mapper_node' "9"
  kill_processes_with_text 'l5_safety_shield_node' "9"
  kill_processes_with_text 'l7_policy_node' "9"
  kill_xterms_with_text "L7.1" "9"
  kill_xterms_with_text "ArduCopter" "9"
  kill_xterms_with_text "MAVProxy" "9"
}

print_leftovers() {
  pgrep -a -x arducopter || true
  pgrep -a -x mavproxy.py || true
  pgrep -a -x mavros_node || true
  print_processes_with_text 'gz sim'
  print_processes_with_comm_and_text 'ruby' 'gz'
  print_processes_with_text 'sim_vehicle.py'
  print_processes_with_text 'l4_gz_pointcloud_bridge_node'
  print_processes_with_text 'l4_pointcloud_mapper_node'
  print_processes_with_text 'l5_safety_shield_node'
  print_processes_with_text 'l7_policy_node'
  print_xterms_with_text "L7.1"
  print_xterms_with_text "ArduCopter"
  print_xterms_with_text "MAVProxy"
}

if [[ "${PRE_CLEANUP}" == "true" || "${CLEANUP_ONLY}" == "true" ]]; then
  echo "Cleaning old L7.1/Gazebo/SITL/MAVROS processes..."
  cleanup_known_processes
  echo "Remaining matching processes:"
  print_leftovers
fi
if [[ "${CLEANUP_ONLY}" == "true" ]]; then
  exit 0
fi

if [[ "${BACKEND}" == "auto" ]]; then
  if wt_usable; then
    BACKEND="wt"
  elif xterm_usable; then
    BACKEND="xterm"
  else
    echo "No usable terminal backend found." >&2
    echo "Need working WSL interop + Windows Terminal, or xterm with DISPLAY set." >&2
    exit 2
  fi
fi

if [[ "${BACKEND}" != "wt" && "${BACKEND}" != "xterm" ]]; then
  echo "Unsupported backend: ${BACKEND}. Use auto, wt, or xterm." >&2
  exit 2
fi

if [[ "${BACKEND}" == "wt" ]] && ! wt_usable; then
  echo "Windows Terminal backend requested, but wt.exe is not executable from this WSL session." >&2
  echo "Use --backend xterm, or enable WSL interop and retry." >&2
  exit 2
fi

if [[ "${BACKEND}" == "xterm" ]] && ! xterm_usable; then
  echo "xterm backend requested, but xterm or DISPLAY is unavailable." >&2
  exit 2
fi

if [[ "${DRY_RUN}" != "true" ]]; then
  RUN_DIR="${TMPDIR:-/tmp}/l7_1_terminals_${SCENARIO_NAME}_$(date +%Y%m%d_%H%M%S)_$$"
  mkdir -p "${RUN_DIR}"
  echo "Terminal command files: ${RUN_DIR}"
fi
TERMINAL_INDEX=0

make_cmd() {
  local title="$1"
  local body="$2"
  cat <<EOF
#!/usr/bin/env bash
echo "===== ${title} ====="
echo "Scenario: ${SCENARIO_NAME}"
echo
cat <<'VISIBLE_COMMAND'
${body}
VISIBLE_COMMAND
echo
(
set -e
${body}
)
status=\$?
echo
echo "Command exited with status \${status}."
echo "Close this terminal when finished, or press Ctrl-D after the shell prompt appears."
exec bash
EOF
}

launch_terminal() {
  local title="$1"
  local body="$2"

  if [[ "${DRY_RUN}" == "true" ]]; then
    echo
    echo "===== DRY RUN: ${title} ====="
    printf '%s\n' "${body}"
    return 0
  fi

  TERMINAL_INDEX=$((TERMINAL_INDEX + 1))
  local safe_title
  safe_title="$(printf '%s' "${title}" | tr -cs 'A-Za-z0-9._-' '_' | sed 's/^_//;s/_$//')"
  local cmd_file
  cmd_file="${RUN_DIR}/$(printf '%02d' "${TERMINAL_INDEX}")_${safe_title}.sh"
  make_cmd "${title}" "${body}" > "${cmd_file}"
  chmod +x "${cmd_file}"

  if [[ "${BACKEND}" == "wt" ]]; then
    wt.exe -w 0 new-tab --title "${title}" wsl.exe -d "${DISTRO}" --cd "${PROJECT_DIR}" -- bash "${cmd_file}" >/dev/null 2>&1 &
    return 0
  fi

  nohup xterm -T "${title}" -hold -e bash "${cmd_file}" > "${cmd_file%.sh}.xterm.log" 2>&1 &
}

sleep_between() {
  local seconds="$1"
  if [[ "${DRY_RUN}" == "true" ]]; then
    echo "[dry-run] sleep ${seconds}s"
    return 0
  fi
  sleep "${seconds}"
}

start_auto_cleanup_monitor() {
  if [[ "${AUTO_CLEANUP}" != "true" ]]; then
    echo "Auto cleanup monitor disabled."
    return 0
  fi
  if [[ "${DRY_RUN}" == "true" ]]; then
    echo "[dry-run] auto cleanup monitor would start; delay=${AUTO_CLEANUP_DELAY_S}s timeout=${AUTO_CLEANUP_TIMEOUT_S}s"
    return 0
  fi

  local monitor_py="${RUN_DIR}/auto_cleanup_monitor.py"
  local monitor_sh="${RUN_DIR}/auto_cleanup_monitor.sh"
  local monitor_log="${RUN_DIR}/auto_cleanup_monitor.log"

  cat > "${monitor_py}" <<'PY'
#!/usr/bin/env python3
import argparse
import glob
import os
import subprocess
import sys
import time

try:
    import rclpy
    from rclpy.node import Node
    from mavros_msgs.msg import State
    from nav_msgs.msg import Odometry
except Exception as exc:  # noqa: BLE001
    rclpy = None
    IMPORT_ERROR = exc
else:
    IMPORT_ERROR = None


def write_log(path: str, text: str) -> None:
    stamp = time.strftime("%Y-%m-%d %H:%M:%S")
    with open(path, "a", encoding="utf-8") as f:
        f.write(f"[{stamp}] {text}\n")


def read_text(path: str) -> str:
    try:
        with open(path, "r", encoding="utf-8", errors="ignore") as f:
            return f.read()
    except OSError:
        return ""


def find_l7_log(scenario: str, start_time: float) -> str | None:
    candidates: list[str] = []
    patterns = [
        os.path.expanduser("~/.ros/log/python3_*"),
        os.path.expanduser("~/.ros/log/l7_policy_node_*"),
    ]
    for pattern in patterns:
        for path in glob.glob(pattern):
            try:
                if os.path.getmtime(path) < start_time - 5.0:
                    continue
            except OSError:
                continue
            head = read_text(path)[:8192]
            if (
                "L7 policy node started" in head
                and f"scenario={scenario}" in head
                and "encoder_mode=perception" in head
            ):
                candidates.append(path)
    if not candidates:
        return None
    return max(candidates, key=lambda p: os.path.getmtime(p))


class TelemetryMonitor(Node):
    def __init__(self) -> None:
        super().__init__("l7_auto_cleanup_monitor")
        self.armed = False
        self.ever_armed = False
        self.origin_z: float | None = None
        self.rel_z = 0.0
        self.max_rel_z = 0.0
        self.have_odom = False
        self.create_subscription(State, "/mavros/state", self._state_cb, 10)
        self.create_subscription(Odometry, "/mavros/local_position/local", self._odom_cb, 10)

    def _state_cb(self, msg: State) -> None:
        self.armed = bool(msg.armed)
        self.ever_armed = self.ever_armed or self.armed

    def _odom_cb(self, msg: Odometry) -> None:
        z = float(msg.pose.pose.position.z)
        if self.origin_z is None:
            self.origin_z = z
        self.rel_z = z - self.origin_z
        self.max_rel_z = max(self.max_rel_z, self.rel_z)
        self.have_odom = True


def detect_log_event(text: str) -> tuple[str | None, bool]:
    goal_reached = "L7 goal reached" in text
    if "State -> DONE" in text:
        return "landed_done", goal_reached
    failure_needles = [
        "State -> FAILSAFE",
        "L7 mission timeout",
        "Traceback",
        "RuntimeError",
        "Exception",
        "crash",
        "Crash",
        "CRASH",
    ]
    for needle in failure_needles:
        if needle in text:
            return f"failure_or_crash_log:{needle}", goal_reached
    return None, goal_reached


def run_cleanup(args: argparse.Namespace, reason: str) -> int:
    write_log(args.log_path, f"terminal event detected: {reason}")
    write_log(args.log_path, f"waiting {args.delay_s:.1f}s before cleanup")
    time.sleep(max(0.0, args.delay_s))
    write_log(args.log_path, "running cleanup-only")
    result = subprocess.run(
        ["bash", args.script_path, "--cleanup-only"],
        cwd=args.project_dir,
        text=True,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        check=False,
    )
    write_log(args.log_path, f"cleanup exit={result.returncode}")
    if result.stdout:
        write_log(args.log_path, result.stdout.rstrip())
    return result.returncode


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--project-dir", required=True)
    parser.add_argument("--script-path", required=True)
    parser.add_argument("--scenario", required=True)
    parser.add_argument("--delay-s", type=float, required=True)
    parser.add_argument("--timeout-s", type=float, required=True)
    parser.add_argument("--poll-s", type=float, default=2.0)
    parser.add_argument("--log-path", required=True)
    args = parser.parse_args()

    start_time = time.time()
    write_log(args.log_path, f"monitor started scenario={args.scenario} delay={args.delay_s}s timeout={args.timeout_s}s")

    node = None
    if rclpy is None:
        write_log(args.log_path, f"ROS monitor unavailable, falling back to log-only monitor: {IMPORT_ERROR}")
    else:
        rclpy.init(args=None)
        node = TelemetryMonitor()

    l7_log = None
    goal_reached = False
    try:
        while True:
            if node is not None:
                rclpy.spin_once(node, timeout_sec=0.1)

            if l7_log is None:
                l7_log = find_l7_log(args.scenario, start_time)
                if l7_log is not None:
                    write_log(args.log_path, f"tracking l7 log: {l7_log}")

            if l7_log is not None:
                text = read_text(l7_log)
                event, log_goal = detect_log_event(text)
                goal_reached = goal_reached or log_goal
                if event is not None:
                    return run_cleanup(args, event)

            if node is not None and node.have_odom and node.max_rel_z >= 0.8:
                if node.ever_armed and not node.armed and node.rel_z < 0.5:
                    reason = "landed_or_disarmed_after_takeoff"
                    if goal_reached:
                        reason = "landed_disarmed_after_goal"
                    return run_cleanup(args, reason)
                if not goal_reached and node.armed and node.rel_z < 0.25:
                    return run_cleanup(args, "possible_crash_ground_contact")

            elapsed = time.time() - start_time
            if args.timeout_s > 0 and elapsed >= args.timeout_s:
                return run_cleanup(args, f"monitor_timeout_{args.timeout_s:.0f}s")

            time.sleep(max(0.2, args.poll_s))
    finally:
        if node is not None:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
PY
  chmod +x "${monitor_py}"

  cat > "${monitor_sh}" <<EOF
#!/usr/bin/env bash
set -eo pipefail
cd "${PROJECT_DIR}"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
python3 "${monitor_py}" \\
  --project-dir "${PROJECT_DIR}" \\
  --script-path "${SCRIPT_PATH}" \\
  --scenario "${SCENARIO_NAME}" \\
  --delay-s "${AUTO_CLEANUP_DELAY_S}" \\
  --timeout-s "${AUTO_CLEANUP_TIMEOUT_S}" \\
  --poll-s "${MONITOR_POLL_S}" \\
  --log-path "${monitor_log}"
EOF
  chmod +x "${monitor_sh}"
  nohup bash "${monitor_sh}" > "${RUN_DIR}/auto_cleanup_monitor.launch.log" 2>&1 &
  echo "Auto cleanup monitor started: ${monitor_log}"
  echo "Cleanup delay after landing/crash: ${AUTO_CLEANUP_DELAY_S}s"
}

GAZEBO_CMD="cd \"${PROJECT_DIR}\"
export GZ_SIM_RESOURCE_PATH=\"${PROJECT_DIR}/models:${PROJECT_DIR}/worlds:\${HOME}/ardupilot_gazebo/models:\${HOME}/ardupilot_gazebo/worlds:\${GZ_SIM_RESOURCE_PATH:-}\"
export GZ_SIM_SYSTEM_PLUGIN_PATH=\"\${HOME}/ardupilot_gazebo/build:\${GZ_SIM_SYSTEM_PLUGIN_PATH:-}\"
export LIBGL_ALWAYS_SOFTWARE=1
gz sim -v4 -r \"worlds/l4/${WORLD_FILE}\""

SITL_CMD='cd ~/ardupilot/Tools/autotest
python3 ./sim_vehicle.py -v ArduCopter -f gazebo-iris --model JSON --map --console --out=udp:127.0.0.1:14550'

MAVROS_CMD="cd \"${PROJECT_DIR}\"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 launch mavros node.launch \\
  fcu_url:=udp://:14550@ \\
  gcs_url:=udp://:14551@ \\
  tgt_system:=1 \\
  tgt_component:=1 \\
  pluginlists_yaml:=\"\${PWD}/config/mavros_l1_pluginlists.yaml\" \\
  config_yaml:=/opt/ros/humble/share/mavros/launch/apm_config.yaml \\
  namespace:=mavros"

BRIDGE_CMD="cd \"${PROJECT_DIR}\"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run l4_perception_mapping l4_gz_pointcloud_bridge_node --ros-args \\
  -p gz_topic:=/l4/depth_camera/points \\
  -p ros_topic:=/l4/depth_camera/points \\
  -p repack_xyz:=true \\
  -p sample_step:=4"

MAPPER_CMD="cd \"${PROJECT_DIR}\"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run l4_perception_mapping l4_pointcloud_mapper_node --ros-args \\
  -p pointcloud_topic:=/l4/depth_camera/points \\
  -p odom_topic:=/mavros/local_position/local \\
  -p pose_topic:=/mavros/local_position/pose \\
  -p use_odom:=true \\
  -p transform_to_odom_frame:=true \\
  -p output_frame_id:=map \\
  -p query_rate_hz:=10.0 \\
  -p local_radius_m:=6.0 \\
  -p voxel_size_m:=0.15 \\
  -p map_memory_s:=90.0 \\
  -p max_map_voxels:=80000 \\
  -p camera_offset_x:=0.22 \\
  -p camera_offset_y:=0.0 \\
  -p camera_offset_z:=0.06"

SHIELD_CMD="cd \"${PROJECT_DIR}\"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ros2 run l5_safety_shield l5_safety_shield_node --ros-args \\
  -p raw_cmd_topic:=/l5/raw_cmd_vel \\
  -p safe_cmd_topic:=/mavros/setpoint_velocity/cmd_vel \\
  -p nearest_distance_topic:=/l4/nearest_distance \\
  -p nearest_normal_topic:=/l4/nearest_normal \\
  -p odom_topic:=/mavros/local_position/local \\
  -p state_topic:=/mavros/state \\
  -p map_timeout_s:=1.0 \\
  -p cmd_timeout_s:=0.5 \\
  -p body_radius:=0.35 \\
  -p safety_margin:=0.15 \\
  -p stop_d_eff:=0.08 \\
  -p caution_d_eff:=0.85 \\
  -p toward_speed_gain:=0.8 \\
  -p max_xy_speed:=0.35 \\
  -p max_z_speed:=0.22 \\
  -p min_altitude:=1.2 \\
  -p max_altitude:=2.6"

POLICY_CMD="cd \"${PROJECT_DIR}\"
source /opt/ros/humble/setup.bash
source install/setup.bash
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
PERCEPTION_POLICY=\"${PERCEPTION_POLICY}\"
test -f \"\${PERCEPTION_POLICY}\" || { echo \"missing perception policy: \${PERCEPTION_POLICY}\"; return 1 2>/dev/null || exit 1; }
echo \"\${PERCEPTION_POLICY}\"
ros2 run l7_policy_deployment l7_policy_node --ros-args \\
  -p scenario:=${SCENARIO_NAME} \\
  -p policy_path:=\"\${PERCEPTION_POLICY}\" \\
  -p encoder_mode:=perception \\
  -p raw_cmd_topic:=/l5/raw_cmd_vel \\
  -p odom_topic:=/mavros/local_position/local \\
  -p pose_topic:=/mavros/local_position/pose \\
  -p state_topic:=/mavros/state \\
  -p shield_active_topic:=/l5/shield_active \\
  -p nearest_distance_topic:=/l4/nearest_distance \\
  -p nearest_normal_topic:=/l4/nearest_normal \\
  -p local_cloud_topic:=/l4/local_cloud \\
  -p map_timeout_s:=1.0 \\
  -p local_cloud_max_points:=2500 \\
  -p takeoff_alt:=2.0 \\
  -p goal_radius:=0.50 \\
  -p max_xy_speed:=0.35 \\
  -p max_z_speed:=0.22 \\
  -p auto_mission:=true \\
  -p require_guided:=true \\
  -p mission_timeout_s:=220.0"

echo "Opening L7.1 perception terminals for ${SCENARIO_NAME} with backend=${BACKEND}"
echo "Terminal 2 / SITL delay before MAVROS: ${SITL_DELAY_S}s"

launch_terminal "L7.1 T1 Gazebo ${SCENE}" "${GAZEBO_CMD}"
sleep_between "${GAZEBO_DELAY_S}"
launch_terminal "L7.1 T2 SITL ${SCENE}" "${SITL_CMD}"
echo "Waiting ${SITL_DELAY_S}s after terminal 2 / SITL..."
sleep_between "${SITL_DELAY_S}"
launch_terminal "L7.1 T3 MAVROS ${SCENE}" "${MAVROS_CMD}"
sleep_between "${MAVROS_DELAY_S}"
launch_terminal "L7.1 T4 PointCloud Bridge ${SCENE}" "${BRIDGE_CMD}"
sleep_between "${BRIDGE_DELAY_S}"
launch_terminal "L7.1 T5 Local Mapper ${SCENE}" "${MAPPER_CMD}"
sleep_between "${MAPPER_DELAY_S}"
launch_terminal "L7.1 T6A Safety Shield ${SCENE}" "${SHIELD_CMD}"
sleep_between "${SHIELD_DELAY_S}"
launch_terminal "L7.1 T6B Policy ${SCENE}" "${POLICY_CMD}"
start_auto_cleanup_monitor

cat <<EOF

Opened terminals for ${SCENARIO_NAME}.
Check terminal 6B for:
  encoder_mode=perception
  policy=.../l6_l63_perception_mixed_bc/policy_best.pt

Auto cleanup:
  enabled=${AUTO_CLEANUP}
  delay_after_landing_or_crash=${AUTO_CLEANUP_DELAY_S}s
  monitor_timeout=${AUTO_CLEANUP_TIMEOUT_S}s

If a terminal fails immediately, its tab stays open and prints the command plus exit status.
EOF
