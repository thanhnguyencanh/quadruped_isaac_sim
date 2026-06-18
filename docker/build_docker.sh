#!/usr/bin/env bash
# =============================================================================
# build_docker.sh — build the thanhnc19/unige_legged ROS 2 environment image.
#
#   ./docker/build_docker.sh
#   IMAGE=thanhnc19/unige_legged TAG=v1.0 ./docker/build_docker.sh
# =============================================================================
set -euo pipefail

IMAGE="${IMAGE:-thanhnc19/unige_legged}"
TAG="${TAG:-latest}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo ">> Building '${IMAGE}:${TAG}' from docker/Dockerfile"
docker build \
  -t "${IMAGE}:${TAG}" \
  -f "${SCRIPT_DIR}/Dockerfile" \
  "${SCRIPT_DIR}"

echo ">> Done. Image:"
docker image ls "${IMAGE}:${TAG}"
echo ">> Next: ./docker/push_docker.sh   (pushes ${IMAGE}:${TAG} to Docker Hub)"
