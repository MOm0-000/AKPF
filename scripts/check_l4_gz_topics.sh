#!/usr/bin/env bash
set -eo pipefail

PATTERN="${1:-l4/depth_camera}"

echo "Gazebo topics matching: ${PATTERN}"
gz topic -l | grep -E "${PATTERN}|points|depth|camera" || true

echo
echo "If the depth camera is active, expect a topic similar to:"
echo "  /l4/depth_camera/points"
