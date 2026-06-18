#!/usr/bin/env bash
# =============================================================================
# run_unige_docker.sh — start the unige_legged container with the colcon
# workspace mounted, ready to `colcon build` and run the ROS 2 nodes.
#
# Isaac Sim runs on the HOST; the container shares the host network + the same
# ROS_DOMAIN_ID so its nodes discover the Isaac ROS 2 bridge over DDS.
#
#   ./docker/run_unige_docker.sh                # interactive shell
#   CMD="ros2 launch spot_sar_bringup perception.launch.py" ./docker/run_unige_docker.sh
# =============================================================================
set -euo pipefail

IMAGE="${IMAGE:-thanhnc19/unige_legged}"
TAG="${TAG:-latest}"
ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
CONTAINER_WS=/opt/unige_ws

# Repo root = two levels up from this script (docker/ -> repo). Mount the whole
# colcon src so the container sees every package; build dirs stay container-side.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"

GPU_FLAGS=()
if command -v nvidia-smi >/dev/null 2>&1; then
  GPU_FLAGS=(--gpus all)
fi

exec docker run -it --rm \
  "${GPU_FLAGS[@]}" \
  --net=host --ipc=host \
  -e ROS_DOMAIN_ID="${ROS_DOMAIN_ID}" \
  -e DISPLAY="${DISPLAY:-}" \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v "${REPO_DIR}:${CONTAINER_WS}/src/quadruped_isaac_sim:rw" \
  -w "${CONTAINER_WS}" \
  "${IMAGE}:${TAG}" \
  bash -lc "${CMD:-bash}"
