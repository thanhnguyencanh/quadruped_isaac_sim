#!/usr/bin/env bash
# make_transfer_bundle.sh — package the SMALL, must-copy files for migrating this project to a new
# machine (full procedure: ../MIGRATION.md). Everything large (Isaac Sim, the ~276 GB asset pack, ROS 2,
# the venvs, the docker image) is RE-DOWNLOADED, not bundled. Output: ~/spot_sar_transfer.tar.gz (~10 MB).
#
#   ./scripts/make_transfer_bundle.sh [output.tar.gz]
set -eo pipefail

OUT="${1:-$HOME/spot_sar_transfer.tar.gz}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"                 # the quadruped_isaac_sim checkout
# Claude Code project memory is keyed by the project path (slashes -> dashes). Override via MEMORY_DIR.
MEMORY_DIR="${MEMORY_DIR:-$HOME/.claude/projects/-home-thanhnc-unige-ws-src-quadruped-isaac-sim/memory}"
YOLO_PT="${YOLO_PT:-$HOME/yolo_venv/yolov8n.pt}"
DATASET="${DATASET:-$HOME/unige_ws/datasets/sar_victims}"
OMNI_CACHE="${OMNI_CACHE:-$HOME/.cache/isaacsim_omni_cache}"

STAGE="$(mktemp -d)"
trap 'rm -rf "$STAGE"' EXIT

# Only bundle the pieces that actually exist (each is a `tar -C <dir> <name>` pair).
args=()
add() {  # add <label> <path>
  if [ -e "$2" ]; then args+=( -C "$(dirname "$2")" "$(basename "$2")" ); echo "  + $2"
  else echo "  - (missing, skipped) $2"; fi
}
echo "collecting must-copy files:"
add docs        "$REPO/docs"
add memory      "$MEMORY_DIR"
add weights     "$YOLO_PT"
add dataset     "$DATASET"
add omni_cache  "$OMNI_CACHE"

cat > "$STAGE/MANIFEST.txt" <<EOF
spot_sar_transfer.tar.gz — SMALL, must-copy files for the Spot SAR migration.
Full procedure: MIGRATION.md in the repo. Unpack on the new desktop AFTER cloning the repo, then move:
  docs/                -> ~/unige_ws/src/quadruped_isaac_sim/docs   (gitignored; private)
  memory/              -> ~/.claude/projects/<project-path-slug>/memory   (Claude Code project memory)
  yolov8n.pt           -> ~/yolo_venv/yolov8n.pt                    (pinned YOLO weights)
  sar_victims/         -> ~/unige_ws/datasets/sar_victims           (optional; regenerable)
  isaacsim_omni_cache/ -> ~/.cache/isaacsim_omni_cache              (optional; speeds first boot)
NOT included (re-obtain on the new machine): Isaac Sim + the ~276 GB asset pack, ROS 2 Jazzy + apt
deps, ~/sar_planning_venv + ~/yolo_venv (rebuild via pip with the pins), the docker image
(docker pull thanhnc19/unige_legged), and credentials (re-auth git + docker).
Generated $(date -u +%Y-%m-%dT%H:%M:%SZ) on $(hostname).
EOF
args+=( -C "$STAGE" MANIFEST.txt )

tar czf "$OUT" "${args[@]}"
echo
echo "wrote $OUT  ($(du -h "$OUT" | cut -f1))"
echo "transfer that single file to the new desktop, then follow MIGRATION.md."
