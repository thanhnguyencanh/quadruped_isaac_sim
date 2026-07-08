# Migrating this project to a new machine

Everything needed to reproduce the Spot SAR Isaac Sim project on a fresh desktop. Almost nothing needs
a raw byte-transfer: the heavy pieces (Isaac Sim, the ~276 GB asset pack, ROS 2, the venvs, the 7.35 GB
docker image) **re-download or rebuild**. You copy one small bundle, re-auth credentials, and rebuild.

*Verified 2026-07-08 against commit `b2898c4`. For what's implemented + what to do next once you're back
up, see `docs/STATUS_AND_NEXT_STEPS.md` (it lives in the gitignored `docs/`, so it arrives via the
transfer bundle in §0, not the clone).*

> **Context that simplifies things:** on the original box, Isaac Sim + the assets lived in a *root-owned,
> shared* `/home/isaac/` (a lab machine shared with `IsaacLab` and other users), symlinked into the home
> dir. On a **personal** desktop you install Isaac under your **own** home — so the root-owned-symlink
> complexity goes away entirely.

## 0. The one small file to copy

Regenerate the transfer bundle any time with:

```bash
./scripts/make_transfer_bundle.sh          # -> ~/spot_sar_transfer.tar.gz  (~10 MB)
```

It contains only the small, hard-to-reproduce or private files:

| In the bundle | Size | Why |
|---|---|---|
| `docs/` | ~850 KB | **Private, gitignored** (grant RTF, implementation guide, Isaac6 cookbook, slides) — not on GitHub |
| `memory/` | ~50 KB | Claude Code project memory (only auto-loads if the new machine uses the same project path + user) |
| `yolov8n.pt` | 6.5 MB | Pinned YOLO weights (else ultralytics re-fetches) |
| `sar_victims/` | 3.8 MB | Replicator dataset — regenerable, optional |
| `isaacsim_omni_cache/` | 1.8 MB | RTX/OGN cache — optional, speeds the first cold boot |

## 1. RE-DOWNLOAD / REINSTALL (do NOT copy the bytes)

| Thing | Notes |
|---|---|
| **NVIDIA driver** | ≥535 (580 known-good on the RTX 5060 Blackwell). Reboot, verify `nvidia-smi`. |
| **OS** | Ubuntu 24.04. |
| **ROS 2 Jazzy + tools** | `ros-jazzy-desktop`, `ros-dev-tools`, `python3-colcon-common-extensions`, `python3-rosdep`, `build-essential`, `cmake`, `default-jre`. |
| **Nav2 / SLAM / sensors** | Pulled by `rosdep install` from the package.xml files: navigation2, nav2-bringup, slam_toolbox, depthimage_to_laserscan, depth_image_proc, cv_bridge, robot_state_publisher, xacro, tf2, vision_msgs, … |
| **3D-mapping apt deps** | Not part of the core stack — installed later by `scripts/setup_3d_mapping.sh octomap` (octomap-server, octomap-rviz-plugins, grid_map*). |
| **Isaac Sim 6.0** | Re-download the workstation package (this project used `6.0.0-rc.59`) → `~/isaacsim`. |
| **Isaac assets** | Re-download the NVIDIA asset pack → `~/isaacsim_assets`. The apps pin the asset root to `$ISAAC_ASSETS` (default `~/isaacsim_assets`). See the note below — you likely don't need all 276 GB. |
| **Planning venv** | `~/sar_planning_venv` — rebuild via pip (Fast Downward compiles during install). |
| **YOLO venv** | `~/yolo_venv` — rebuild via pip, **keep the numpy/opencv pins**. |
| **Docker image** | `docker pull thanhnc19/unige_legged` (don't copy the 7.35 GB). |
| **Credentials** | Re-auth git (PAT) + `docker login`. Never copy tokens in clear. |

## 2. REGENERATE (rebuild on the new machine)
- `~/unige_ws/{build,install,log}` → `colcon build`.
- The RTX/OGN cache rebuilds on first Isaac run (or use the bundled copy).

## 3. SKIP (don't transfer)
- `.vscode/browse.vc.db` (this is the "1.2 GB" — VS Code IntelliSense junk).
- The repo dir itself → **clone it** from GitHub (fully pushed).
- The docker image bytes (pull from Hub), any `/tmp/...` scratch, and all secrets.

## 4. Ordered restore procedure

```bash
# 1. NVIDIA driver (>=535) + reboot ;  verify:  nvidia-smi

# 2. ROS 2 Jazzy + build tools
sudo apt install ros-jazzy-desktop ros-dev-tools python3-colcon-common-extensions \
                 python3-rosdep build-essential cmake default-jre
sudo rosdep init 2>/dev/null; rosdep update

# 3. Isaac Sim 6.0 -> ~/isaacsim ;  NVIDIA asset pack -> ~/isaacsim_assets   (from NVIDIA)

# 4. Clone the repo + restore the small bundle
mkdir -p ~/unige_ws/src && cd ~/unige_ws/src
git clone https://github.com/thanhnguyencanh/quadruped_isaac_sim.git
mkdir -p /tmp/restore && tar xzf ~/spot_sar_transfer.tar.gz -C /tmp/restore
cp -r /tmp/restore/docs        ~/unige_ws/src/quadruped_isaac_sim/           # gitignored docs
cp -r /tmp/restore/sar_victims ~/unige_ws/datasets/ 2>/dev/null || true
# memory -> the path-hashed dir under ~/.claude/projects/ (see the bundle's MANIFEST.txt)

# 5. ROS deps declared in the package.xml files
cd ~/unige_ws && rosdep install --from-paths src --ignore-src -r -y

# 6. Planning venv (unified_planning + Fast Downward + ENHSP; Fast Downward builds during pip)
python3.12 -m venv --system-site-packages ~/sar_planning_venv
~/sar_planning_venv/bin/pip install unified-planning up-fast-downward up-enhsp networkx

# 7. YOLO venv — the numpy/opencv PINS are load-bearing (ROS cv_bridge ABI)
python3.12 -m venv --system-site-packages ~/yolo_venv
~/yolo_venv/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
~/yolo_venv/bin/pip install ultralytics "numpy==1.26.4" "opencv-python==4.9.0.80"
cp /tmp/restore/yolov8n.pt ~/yolo_venv/           # or let it auto-download on first run

# 8. Build + verify
cd ~/unige_ws && colcon build --symlink-install && source install/setup.bash
./src/quadruped_isaac_sim/scripts/run_isaac.sh \
   src/quadruped_isaac_sim/spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py --building --gui

# 9. Re-auth git (PAT) + docker login ;  docker pull thanhnc19/unige_legged   (optional)
# 10. (Optional) 3D mapping deps:  ./src/quadruped_isaac_sim/scripts/setup_3d_mapping.sh octomap
```

## 5. Gotchas

- **The ~276 GB assets** — you almost certainly don't need all of it. This project only references
  `Isaac/Environments/Grid/default_environment.usd` and `Isaac/People/Characters/*` (Spot itself ships
  inside Isaac Sim). Easiest = the NVIDIA asset-pack download; leanest = copy just those two subtrees
  (a few GB) into `~/isaacsim_assets/Isaac/...`.
- **`numpy==1.26.4`** in *both* venvs — ROS `cv_bridge` is built against numpy 1.x; torch/opencv would
  otherwise pull numpy 2.x → `numpy.core.multiarray failed to import`. Same reason for `opencv-python==4.9.0.80`.
- **`default-jre`** (Java) is required for ENHSP (`up-enhsp`); `up-fast-downward` needs `cmake` + `build-essential`.
- **`ROS_DOMAIN_ID=42`** — `run_isaac.sh` sets it internally, but export it in any *other* shell that runs
  `ros2` CLI, or topics won't be visible.
- **Memory continuity** only if the new machine uses the identical project path
  (`~/unige_ws/src/quadruped_isaac_sim`) and user; otherwise unpack `memory/` into the new
  path-hashed dir under `~/.claude/projects/`.
- **conda** (if you install miniconda) auto-activates its base env and shadows ROS/Isaac — `run_isaac.sh`
  already `conda deactivate`s defensively, so this is handled.

## 6. Verify your restore

Cheap checks, fastest first — each isolates one layer:

```bash
# a) GPU + driver
nvidia-smi

# b) pure-python + PDDL layer (no Isaac): the two-floor geometry self-test
python3 ~/unige_ws/src/quadruped_isaac_sim/spot_sar_planning/spot_sar_planning/sar_building.py
#    expect: "[sar_building] OK: 8 rooms, 6 portals ... landing stacked at (12.0, 0.0)."

# c) workspace built + sourced
cd ~/unige_ws && source install/setup.bash && ros2 pkg list | grep spot_sar   # 8 packages

# d) Isaac + the two-floor scene renders (GUI), Spot stands
./src/quadruped_isaac_sim/scripts/run_isaac.sh \
   src/quadruped_isaac_sim/spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py --building --gui

# e) end-to-end sanity: /cmd_vel + camera + YOLO (headless), then teleport the stairs by hand
ros2 launch spot_sar_bringup perception.launch.py building:=true      # shell 1
ros2 topic pub --once /stairs_cmd std_msgs/msg/String '{data: "stair_main up"}'   # shell 2 (near the landing)
ros2 topic echo /floor_state   # -> "f2"
```

If (b) passes you know the symbolic layer transferred intact; if (d) renders the building with Spot
standing, the Isaac install + assets + venvs are all wired correctly.
