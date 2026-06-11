#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

export GZ_SIM_RESOURCE_PATH="${HOME}/ardupilot_gazebo/models:${HOME}/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
unset __NV_PRIME_RENDER_OFFLOAD || true
unset __GLX_VENDOR_LIBRARY_NAME || true
unset __VK_LAYER_NV_optimus || true
set -u

for world in "${PROJECT_DIR}"/worlds/l2/*.sdf; do
  name="$(basename "${world}" .sdf)"
  echo "[L2] Checking ${name}"
  if timeout 12s gz sim -s -r -v3 "${world}" >/tmp/l2_${name}.log 2>&1; then
    echo "[L2] ${name}: exited before timeout"
  else
    status=$?
    if [[ ${status} -eq 124 ]]; then
      echo "[L2] ${name}: loaded and ran for 12s"
    else
      echo "[L2] ${name}: failed with status ${status}" >&2
      tail -80 "/tmp/l2_${name}.log" >&2 || true
      exit ${status}
    fi
  fi
  pkill -f "gz sim -s -r -v3 ${world}" 2>/dev/null || true
  sleep 1
done
