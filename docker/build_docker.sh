#!/usr/bin/env bash
# =============================================================================
# build_docker.sh — build the thanhnc19/unige_legged ROS 2 environment image.
#
#   ./docker/build_docker.sh
#   IMAGE=thanhnc19/unige_legged TAG=v1.0 ./docker/build_docker.sh
#   WITH_YOLO=1 ./docker/build_docker.sh          # + bake in the ~/yolo_venv learned detector
# =============================================================================
set -euo pipefail

IMAGE="${IMAGE:-thanhnc19/unige_legged}"
TAG="${TAG:-latest}"
WITH_YOLO="${WITH_YOLO:-0}"          # 1 = bake in the YOLOv8 detector venv (+~2-3 GB); 0 = HSV only

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ">> Building '${IMAGE}:${TAG}' from docker/Dockerfile  (WITH_YOLO=${WITH_YOLO})"
docker build \
  --build-arg WITH_YOLO="${WITH_YOLO}" \
  -t "${IMAGE}:${TAG}" \
  -f "${SCRIPT_DIR}/Dockerfile" \
  "${SCRIPT_DIR}"

echo ">> Done. Image:"
docker image ls "${IMAGE}:${TAG}"
echo ">> Next: ./docker/push_docker.sh   (pushes ${IMAGE}:${TAG} to Docker Hub)"
