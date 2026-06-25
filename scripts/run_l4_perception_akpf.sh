#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

export DISTANCE_SOURCE="${DISTANCE_SOURCE:-perception_map}"
export PERCEPTION_CLOUD_TOPIC="${PERCEPTION_CLOUD_TOPIC:-/l4/local_cloud}"
export PERCEPTION_STALE_TIMEOUT_S="${PERCEPTION_STALE_TIMEOUT_S:-1.0}"
export PERCEPTION_FALLBACK_TO_TRUTH="${PERCEPTION_FALLBACK_TO_TRUTH:-false}"
export PERCEPTION_MIN_POINTS="${PERCEPTION_MIN_POINTS:-10}"

# First L4.3 validation should avoid truth-geometry local target generation.
export ENABLE_LOCAL_TARGET="${ENABLE_LOCAL_TARGET:-false}"

exec "${SCRIPT_DIR}/run_l3_akpf.sh"
