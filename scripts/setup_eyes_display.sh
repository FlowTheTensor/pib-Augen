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

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
PACKAGE_DIR="${PROJECT_ROOT}"

if [ ! -f "${PACKAGE_DIR}/package.xml" ]; then
  echo "Kein ROS2-Paket gefunden in ${PACKAGE_DIR}." >&2
  exit 1
fi

TARGET_SRC="${WORKSPACE_PATH}/src/eyes_display"

echo "Kopiere Paket in Container..."
docker exec "${CONTAINER_NAME}" bash -lc "mkdir -p '${WORKSPACE_PATH}/src'"
docker cp "${PACKAGE_DIR}" "${CONTAINER_NAME}:${TARGET_SRC}"

echo "Installiere Abhängigkeiten im Container (python3-pyglet)..."
docker exec -it "${CONTAINER_NAME}" bash -lc "apt update && apt install -y python3-pyglet"

echo "Baue Paket im Container..."
docker exec -it "${CONTAINER_NAME}" bash -lc "source /opt/ros/humble/setup.bash && cd '${WORKSPACE_PATH}' && rm -rf build/ install/ log/ && colcon build --symlink-install --packages-select eyes_display"

echo "Fertig. Zum Starten im Container:"
echo "  docker exec -it ${CONTAINER_NAME} bash -lc \"source /opt/ros/humble/setup.bash && source ${WORKSPACE_PATH}/install/setup.bash && ros2 launch eyes_display eyes_display.launch.py\""
