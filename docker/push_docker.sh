#!/usr/bin/env bash
# =============================================================================
# push_docker.sh — push the image to Docker Hub (thanhnc19/unige_legged).
#
#   ./docker/push_docker.sh                 # push thanhnc19/unige_legged:latest
#   TAG=v1.0 ./docker/push_docker.sh        # push a version tag (also retags latest)
#
# Requires `docker login` with the thanhnc19 Docker Hub account.
# =============================================================================
set -euo pipefail

IMAGE="${IMAGE:-thanhnc19/unige_legged}"
TAG="${TAG:-latest}"
DOCKERHUB_USER="${DOCKERHUB_USER:-thanhnc19}"

if ! docker image inspect "${IMAGE}:${TAG}" >/dev/null 2>&1; then
  echo "ERROR: '${IMAGE}:${TAG}' not found locally. Run ./docker/build_docker.sh first." >&2
  exit 1
fi

# Ensure we are logged in to Docker Hub (docker.io).
if ! docker system info 2>/dev/null | grep -q "Username:"; then
  echo ">> Not logged in to Docker Hub. Running 'docker login' as ${DOCKERHUB_USER}..."
  docker login -u "${DOCKERHUB_USER}"
fi

echo ">> Pushing ${IMAGE}:${TAG} ..."
docker push "${IMAGE}:${TAG}"

# Always keep :latest current when pushing a version tag.
if [[ "$TAG" != "latest" ]]; then
  docker tag "${IMAGE}:${TAG}" "${IMAGE}:latest"
  docker push "${IMAGE}:latest"
fi

echo ">> Pushed. View at: https://hub.docker.com/r/${IMAGE}/tags"
