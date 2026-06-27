#!/usr/bin/env bash

tc_wt_usable() {
  command -v wt.exe >/dev/null 2>&1 || return 1
  [[ -e /proc/sys/fs/binfmt_misc/WSLInterop ]] || return 1
  timeout 3s wt.exe --version >/dev/null 2>&1 || return 1
}

tc_xterm_usable() {
  command -v xterm >/dev/null 2>&1 || return 1
  [[ -n "${DISPLAY:-}" ]] || return 1
}

tc_select_backend() {
  if [[ "${DRY_RUN}" == "true" ]]; then
    return 0
  fi

  if [[ "${BACKEND}" == "auto" ]]; then
    if tc_wt_usable; then
      BACKEND="wt"
    elif tc_xterm_usable; then
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

  if [[ "${BACKEND}" == "wt" ]] && ! tc_wt_usable; then
    echo "Windows Terminal backend requested, but wt.exe is not executable from this WSL session." >&2
    echo "Use --backend xterm, or enable WSL interop and retry." >&2
    exit 2
  fi

  if [[ "${BACKEND}" == "xterm" ]] && ! tc_xterm_usable; then
    echo "xterm backend requested, but xterm or DISPLAY is unavailable." >&2
    exit 2
  fi
}

tc_xterm_pids_with_text() {
  local text="$1"
  while read -r pid comm args; do
    if [[ "${comm}" == "xterm" && "${args}" == *"${text}"* ]]; then
      printf '%s\n' "${pid}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

tc_kill_xterms_with_text() {
  local text="$1"
  local signal="${2:-TERM}"
  local -a pids=()
  mapfile -t pids < <(tc_xterm_pids_with_text "${text}")
  if ((${#pids[@]})); then
    if [[ "${signal}" == "TERM" ]]; then
      kill "${pids[@]}" 2>/dev/null || true
    else
      kill "-${signal}" "${pids[@]}" 2>/dev/null || true
    fi
  fi
}

tc_print_xterms_with_text() {
  local text="$1"
  while read -r pid comm args; do
    if [[ "${comm}" == "xterm" && "${args}" == *"${text}"* ]]; then
      printf '%s %s %s\n' "${pid}" "${comm}" "${args}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

tc_process_pids_with_text() {
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

tc_kill_processes_with_text() {
  local text="$1"
  local signal="${2:-TERM}"
  local -a pids=()
  mapfile -t pids < <(tc_process_pids_with_text "${text}")
  if ((${#pids[@]})); then
    if [[ "${signal}" == "TERM" ]]; then
      kill "${pids[@]}" 2>/dev/null || true
    else
      kill "-${signal}" "${pids[@]}" 2>/dev/null || true
    fi
  fi
}

tc_print_processes_with_text() {
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

tc_process_pids_with_comm_and_text() {
  local comm_filter="$1"
  local text="$2"
  while read -r pid comm args; do
    if [[ "${comm}" == "${comm_filter}" && "${args}" == *"${text}"* ]]; then
      printf '%s\n' "${pid}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

tc_kill_processes_with_comm_and_text() {
  local comm_filter="$1"
  local text="$2"
  local signal="${3:-TERM}"
  local -a pids=()
  mapfile -t pids < <(tc_process_pids_with_comm_and_text "${comm_filter}" "${text}")
  if ((${#pids[@]})); then
    if [[ "${signal}" == "TERM" ]]; then
      kill "${pids[@]}" 2>/dev/null || true
    else
      kill "-${signal}" "${pids[@]}" 2>/dev/null || true
    fi
  fi
}

tc_print_processes_with_comm_and_text() {
  local comm_filter="$1"
  local text="$2"
  while read -r pid comm args; do
    if [[ "${comm}" == "${comm_filter}" && "${args}" == *"${text}"* ]]; then
      printf '%s %s %s\n' "${pid}" "${comm}" "${args}"
    fi
  done < <(ps -eo pid=,comm=,args=)
}

tc_cleanup_common_round() {
  local signal="$1"
  local title_marker="$2"
  if [[ "${signal}" == "TERM" ]]; then
    pkill -x arducopter 2>/dev/null || true
    pkill -x mavproxy.py 2>/dev/null || true
    pkill -x mavros_node 2>/dev/null || true
  else
    pkill "-${signal}" -x arducopter 2>/dev/null || true
    pkill "-${signal}" -x mavproxy.py 2>/dev/null || true
    pkill "-${signal}" -x mavros_node 2>/dev/null || true
  fi
  tc_kill_processes_with_text 'gz sim' "${signal}"
  tc_kill_processes_with_comm_and_text 'ruby' 'gz' "${signal}"
  tc_kill_processes_with_text 'sim_vehicle.py' "${signal}"
}

tc_cleanup_xterms_round() {
  local signal="$1"
  local title_marker="$2"
  tc_kill_xterms_with_text "${title_marker}" "${signal}"
  tc_kill_xterms_with_text "ArduCopter" "${signal}"
  tc_kill_xterms_with_text "MAVProxy" "${signal}"
}

tc_print_common_leftovers() {
  local title_marker="$1"
  pgrep -a -x arducopter || true
  pgrep -a -x mavproxy.py || true
  pgrep -a -x mavros_node || true
  tc_print_processes_with_text 'gz sim'
  tc_print_processes_with_comm_and_text 'ruby' 'gz'
  tc_print_processes_with_text 'sim_vehicle.py'
  tc_print_xterms_with_text "${title_marker}"
  tc_print_xterms_with_text "ArduCopter"
  tc_print_xterms_with_text "MAVProxy"
}

tc_make_cmd() {
  local title="$1"
  local body="$2"
  cat <<EOF
#!/usr/bin/env bash
echo "===== ${title} ====="
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

tc_prepare_run_dir() {
  local prefix="$1"
  if [[ "${DRY_RUN}" != "true" ]]; then
    RUN_DIR="${TMPDIR:-/tmp}/${prefix}_$(date +%Y%m%d_%H%M%S)_$$"
    mkdir -p "${RUN_DIR}"
    echo "Terminal command files: ${RUN_DIR}"
  fi
  TERMINAL_INDEX=0
}

tc_launch_terminal() {
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
  tc_make_cmd "${title}" "${body}" > "${cmd_file}"
  chmod +x "${cmd_file}"

  if [[ "${BACKEND}" == "wt" ]]; then
    wt.exe -w 0 new-tab --title "${title}" wsl.exe -d "${DISTRO}" --cd "${PROJECT_DIR}" -- bash "${cmd_file}" >/dev/null 2>&1 &
    return 0
  fi

  nohup xterm -T "${title}" -hold -e bash "${cmd_file}" > "${cmd_file%.sh}.xterm.log" 2>&1 &
}

tc_sleep_between() {
  local seconds="$1"
  if [[ "${DRY_RUN}" == "true" ]]; then
    echo "[dry-run] sleep ${seconds}s"
    return 0
  fi
  sleep "${seconds}"
}
