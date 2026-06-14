#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
SCENARIO="${1:-S1_depth_camera}"
WORLD_FILE="${PROJECT_DIR}/worlds/l4/${SCENARIO}.sdf"

if [[ ! -f "${WORLD_FILE}" ]]; then
  echo "Unknown L4 scenario: ${SCENARIO}" >&2
  echo "Available scenarios:" >&2
  ls "${PROJECT_DIR}/worlds/l4"/*.sdf | xargs -n1 basename | sed 's/.sdf$//' >&2
  exit 2
fi

export GZ_SIM_RESOURCE_PATH="${PROJECT_DIR}/models:${HOME}/ardupilot_gazebo/models:${HOME}/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${HOME}/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export LIBGL_ALWAYS_SOFTWARE="${LIBGL_ALWAYS_SOFTWARE:-1}"
unset __NV_PRIME_RENDER_OFFLOAD || true
unset __GLX_VENDOR_LIBRARY_NAME || true
unset __VK_LAYER_NV_optimus || true
GZ_SERVER_ONLY="${GZ_SERVER_ONLY:-true}"
set -u

if [[ "${GZ_SERVER_ONLY}" == "true" ]]; then
  exec gz sim -s -v4 -r "${WORLD_FILE}"
fi

exec gz sim -v4 -r "${WORLD_FILE}"
