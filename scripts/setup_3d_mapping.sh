#!/usr/bin/env bash
# setup_3d_mapping.sh — install the 3D-mapping deps for the two-floor building demo.
#
# Two stacks:
#   A) OctoMap 3D voxel map        — apt only, GPU-free, LOW RISK. (point cloud uses depth_image_proc,
#      already installed.) This alone gives you `mapping3d.launch.py` (point cloud + octomap + RViz).
#   B) leggedrobotics elevation_mapping_cupy — a ROS 2 *dev-branch* SOURCE build + CuPy (GPU). HEAVY:
#      shares the 8 GB VRAM with the Isaac RTX renderer (OOM risk), and the ROS 2 port is a dev branch.
#
# Run stage A first and verify OctoMap; only then attempt stage B. Needs sudo (apt) + internet.
#   ./scripts/setup_3d_mapping.sh octomap      # stage A only (recommended first)
#   ./scripts/setup_3d_mapping.sh elevation    # stage B only
#   ./scripts/setup_3d_mapping.sh all          # both
set -eo pipefail
WS="${WS_DIR:-$HOME/unige_ws}"
EMC_BRANCH="${EMC_BRANCH:-codex/elevation-despike-jazzy}"   # the Jazzy ROS 2 branch
ELEV_VENV="${ELEV_VENV:-$HOME/elevation_venv}"
STAGE="${1:-all}"

octomap() {
  echo "=== [A] OctoMap 3D voxel + grid_map + depth_image_proc (apt) ==="
  sudo apt-get update
  sudo apt-get install -y \
    ros-jazzy-depth-image-proc \
    ros-jazzy-octomap ros-jazzy-octomap-msgs ros-jazzy-octomap-ros \
    ros-jazzy-octomap-server ros-jazzy-octomap-rviz-plugins \
    ros-jazzy-grid-map ros-jazzy-grid-map-rviz-plugin ros-jazzy-grid-map-msgs \
    ros-jazzy-grid-map-ros ros-jazzy-grid-map-octomap ros-jazzy-grid-map-visualization
  echo "[A] done. Test:  ros2 launch spot_sar_bringup mapping3d.launch.py   (with a --building sim up)"
}

elevation() {
  echo "=== [B] elevation_mapping_cupy (source build + CuPy GPU) ==="
  # 1) CuPy + numpy PINNED to the ROS ABI (1.26; cv_bridge is built against numpy 1.x), in its own venv.
  if [ ! -d "$ELEV_VENV" ]; then
    /usr/bin/python3.12 -m venv --system-site-packages "$ELEV_VENV"
  fi
  # shellcheck disable=SC1091
  source "$ELEV_VENV/bin/activate"
  python -m pip install --upgrade pip
  python -m pip install "numpy==1.26.4" "cupy-cuda12x" scipy shapely
  #   (cupy-cuda12x bundles the CUDA runtime; only the NVIDIA driver is required, not the CUDA toolkit.)
  deactivate

  # 2) build deps for the ROS 2 packages (grid_map etc.) via apt + rosdep.
  sudo apt-get install -y ros-jazzy-grid-map ros-jazzy-pybind11-vendor python3-colcon-common-extensions

  # 3) clone the ROS 2 (Jazzy) dev branch into the workspace and build.
  cd "$WS/src"
  if [ ! -d elevation_mapping_cupy ]; then
    git clone --branch "$EMC_BRANCH" --depth 1 \
      https://github.com/leggedrobotics/elevation_mapping_cupy.git
  fi
  cd "$WS"
  source /opt/ros/jazzy/setup.bash
  rosdep install --from-paths src/elevation_mapping_cupy --ignore-src -r -y || true
  colcon build --packages-up-to elevation_mapping_cupy --symlink-install
  echo "[B] done. NOTE: elevation_mapping_node.py imports cupy — run it under $ELEV_VENV (the node is"
  echo "    launched by elevation.launch.py; if it can't import cupy, prepend $ELEV_VENV/bin to PATH or"
  echo "    run the node with that interpreter). Then:  ros2 launch spot_sar_bringup mapping3d.launch.py elevation:=true"
  echo "    WATCH VRAM (nvidia-smi): elevation_mapping_cupy + Isaac RTX both want the 8 GB GPU."
}

case "$STAGE" in
  octomap) octomap ;;
  elevation) elevation ;;
  all) octomap; elevation ;;
  *) echo "usage: $0 {octomap|elevation|all}"; exit 2 ;;
esac
echo "setup_3d_mapping.sh: '$STAGE' complete."
