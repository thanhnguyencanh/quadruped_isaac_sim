#!/usr/bin/env bash
# run_isaac.sh — launch an Isaac Sim standalone Python script with the right environment.

set -eo pipefail

ISAAC_DIR="${ISAAC_DIR:-$HOME/isaacsim}"
WS_DIR="${WS_DIR:-$HOME/unige_ws}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

# Asset root: prefer a local pack (~/isaacsim_assets) for speed; fall back to NVIDIA's cloud S3
# (Isaac 6.0) if it's absent, so a fresh machine with no downloaded assets still runs by streaming
# USDs. An explicit ISAAC_ASSETS always wins. (get_assets_root_path() accepts either a local dir or
# a URL, as long as it has /Isaac + /NVIDIA under it.)
if [ -z "${ISAAC_ASSETS:-}" ]; then
  if [ -d "$HOME/isaacsim_assets" ]; then
    ISAAC_ASSETS="$HOME/isaacsim_assets"
  else
    ISAAC_ASSETS="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0"
    echo "run_isaac.sh: ~/isaacsim_assets not found -> streaming assets from NVIDIA cloud (Isaac 6.0)" >&2
  fi
fi
export ISAAC_ASSETS

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
