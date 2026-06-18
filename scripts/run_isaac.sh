#!/usr/bin/env bash
# run_isaac.sh — launch an Isaac Sim standalone Python script with the right environment.
#
# Handles the three recurring footguns on this machine:
#   1. conda `base` auto-activates (Python 3.13) and shadows everything → deactivate it.
#   2. Isaac Sim must be run with its OWN bundled Python via python.sh (3.12, matches Jazzy).
#   3. ROS 2 Jazzy + the colcon overlay must be sourced BEFORE launch so the in-process
#      rclpy node and the isaacsim.ros2.bridge can resolve std + spot_sar_msgs messages.
#
# Usage:  run_isaac.sh <script.py> [args...]
# NOTE: no `set -u` — ROS 2 setup.bash references unbound vars (AMENT_TRACE_SETUP_FILES).
set -eo pipefail

ISAAC_DIR="${ISAAC_DIR:-$HOME/isaacsim}"
WS_DIR="${WS_DIR:-$HOME/unige_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Real local asset root (the persistent Isaac config points at a stale /home/isaac path).
export ISAAC_ASSETS="${ISAAC_ASSETS:-$HOME/isaacsim_assets}"

# 1. Drop out of any conda env.
if command -v conda >/dev/null 2>&1; then
  source "$(conda info --base)/etc/profile.d/conda.sh" 2>/dev/null || true
  conda deactivate 2>/dev/null || true
  conda deactivate 2>/dev/null || true
fi

# 2. Source ROS 2 Jazzy + workspace overlay (if already built).
source /opt/ros/jazzy/setup.bash
if [ -f "$WS_DIR/install/setup.bash" ]; then
  source "$WS_DIR/install/setup.bash"
fi

if [ $# -lt 1 ]; then
  echo "usage: run_isaac.sh <script.py> [args...]" >&2
  exit 2
fi

# Resolve the script to an absolute path BEFORE cd'ing into the Isaac dir, so relative
# paths (e.g. spot_sar_sim/.../foo.py) still work from wherever the user invoked us.
SCRIPT="$1"; shift
SCRIPT_ABS="$(readlink -f -- "$SCRIPT")"
if [ ! -f "$SCRIPT_ABS" ]; then
  echo "run_isaac.sh: script not found: $SCRIPT" >&2
  exit 2
fi

cd "$ISAAC_DIR"
exec ./python.sh "$SCRIPT_ABS" "$@"
