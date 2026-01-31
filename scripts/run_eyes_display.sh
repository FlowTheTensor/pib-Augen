#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME="${1:-multirepo-ros-display-1}"
WORKSPACE_PATH="${2:-/app/ros2_ws}"

if ! command -v docker >/dev/null 2>&1; then
  echo "docker ist nicht installiert oder nicht im PATH." >&2
  exit 1
fi

if ! docker ps --format '{{.Names}}' | grep -q "^${CONTAINER_NAME}$"; then
  echo "Container '${CONTAINER_NAME}' läuft nicht." >&2
  exit 1
fi

docker exec -it "${CONTAINER_NAME}" bash -lc "source /opt/ros/humble/setup.bash && source ${WORKSPACE_PATH}/install/setup.bash && ros2 launch eyes_display eyes_display.launch.py"
