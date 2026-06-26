# quadruped_isaac_sim — Spot SAR in NVIDIA Isaac Sim + PDDL planning

A Boston Dynamics **Spot** quadruped, simulated in **NVIDIA Isaac Sim**, explores an
unknown disaster-like indoor environment, detects victims, and reports them — with a
**PDDL planner** (unified-planning + Fast Downward / ENHSP) autonomously selecting the
next action inside a closed **SENSE → GROUND → PLAN → ACT → MONITOR → REPLAN** loop.

University of Genoa (UNIGE) / DIBRIS research grant **D.R. 2237/26** —
supervisor Prof. Antonio Sgorbissa.

## This machine's stack (verified)

| Layer | Version |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| Simulator | Isaac Sim **6.0.0-rc.59** (workstation install at `~/isaacsim`) |
| GPU | NVIDIA RTX 5060, 8 GB VRAM (Blackwell sm_120), driver 580 / CUDA 12.9 |
| ROS 2 | **Jazzy** (Python 3.12) |
| Nav2 / SLAM | `ros-jazzy-navigation2`, `ros-jazzy-slam-toolbox`, `depthimage_to_laserscan` (installed) |
| Planner venv | `~/sar_planning_venv` — unified-planning 1.3.0 + Fast Downward + ENHSP (verified) |

## Status

| Phase | State |
|---|---|
| **Phase 0** — workspace + env | ✅ Done (commit `5a8c2d3`). 8 packages build; Spot loads its flat-terrain policy and **walks ~2.76 m** headless (physics-only smoke test). |
| **Phase 1** — Spot + ROS 2 bridge | ✅ Working. `spot_cmd_vel_app.py`: Spot **walks ~11 m on a ROS 2 `/cmd_vel`** Twist and stands stably at zero command; the bridge publishes `/clock`, `/joint_states`, `/odom`, `/tf` (all visible to system ROS 2 Jazzy on `ROS_DOMAIN_ID=42`). Verified end-to-end via `ros2 launch spot_sar_bringup sim.launch.py`. |
| **Phase 2** — RGB-D perception | ✅ Camera bridge working. `spot_perception_app.py` adds a body-mounted RGB-D camera; the bridge publishes `/camera/rgb/image_raw` (~25 Hz), `/camera/depth/image_raw`, `/camera/rgb/camera_info` (640×480, `camera_optical_frame`) + the `base_link→camera_link→camera_optical_frame` TF. `detector_node` (HSV + depth back-projection) publishes `spot_sar_msgs/VictimArray` on `/victims`. One launch: `ros2 launch spot_sar_bringup perception.launch.py`. |
| **Phase 3** — SLAM + world model | ✅ Done. Depth→`/scan` (`depthimage_to_laserscan`, ~48 Hz, `camera_link` frame); `world_model_node` (symbol grounding → `spot_sar_msgs/WorldModel` on `/world_model`); `slam_toolbox` publishes **`/map`** (verified: 204×187 @ 0.05 m of the walled room) + `map→odom`. Fix: slam_toolbox is a **lifecycle node** — `slam.launch.py` includes slam_toolbox's `online_async_launch.py` so it gets CONFIGURE+ACTIVATE (a bare `Node` stays unconfigured and never subscribes `/scan`). `ros2 launch spot_sar_bringup mapping.launch.py`. |
| Planning env | ✅ `~/sar_planning_venv` solves with Fast Downward / ENHSP; `rclpy` + `unified_planning` coexist. |
| **Phase 4** — Nav2 + skills | ✅ Done & verified. `nav2.launch.py` (minimal bringup, controller→Twist `/cmd_vel`) + a **rolling robot-centred global costmap** (fixes "robot out of bounds of costmap"). **Verified: Nav2 drives Spot to goals (`Reached the goal!`).** `skill_server` hosts the `/skill` action — `go_to_location` (→Nav2), `explore` (→frontier), `observe`, `report` — all verified executing. `frontier_explorer` frontier detection unit-tested. |
| **Phase 5** — PDDL planning | ✅ Domain + problem generation verified. `spot_sar_planning/pddl/domain.pddl` (move/explore/detect/report) solves via Fast Downward; `planner.problem_pddl_from_worldmodel()` turns a `WorldModel` into a problem that also solves (move→…→explore→detect→report). |
| **Phase 6** — Task executive | ✅ Done & verified **end-to-end in sim**. `task_executive` runs SENSE→GROUND→PLAN→ACT→MONITOR→REPLAN (MultiThreadedExecutor): reads `/world_model`, solves a PDDL problem, dispatches the first action as a `/skill` (move→`go_to_location`, explore/detect→`observe`, report→`report`), tracks `found`/`reported` state and replans. **Demonstrated: the executive autonomously detected, navigated to (via Nav2), and reported multiple victims** (e.g. victims 3 & 5) in one run. |
| Phase 7 | 🟡 Evaluation plan in [docs/Phase7_Evaluation.md](docs/); metrics ready to fill from mission runs. |
| Full stack | **`ros2 launch spot_sar_bringup mission.launch.py`** — one command: Isaac + camera + detector + SLAM + Nav2 + skill server + world model + executive. Verified: the closed loop ran stably (no OOM) and reported victims autonomously. (Heavy on 8 GB VRAM; run on a freshly booted machine.) |

## Layout

Colcon workspace is `~/unige_ws` (run `colcon build` there); this repo is `src/quadruped_isaac_sim/`.

```
unige_ws/                      # colcon workspace (build/ install/ log/ live here, gitignored)
└── src/quadruped_isaac_sim/   # this repo
    ├── docs/                  # grant call, interview prep, implementation guide, bridge cookbook
    ├── scripts/run_isaac.sh   # launcher: conda-deactivate + ROS source + asset/cache env + python.sh
    ├── spot_sar_msgs/         # Victim, VictimArray (pinned /victims type), WorldModel  (ament_cmake)
    ├── spot_sar_description/  # URDF/USD references, TF, sensor frames
    ├── spot_sar_bringup/      # launch files + params composing the system
    ├── spot_sar_sim/          # Isaac Sim standalone apps + ROS 2 bridge (see standalone/)
    ├── spot_sar_perception/   # victim detection + localization → /victims
    ├── spot_sar_nav/          # Nav2 + slam_toolbox config, frontier exploration
    ├── spot_sar_planning/     # PDDL domain/problem + unified-planning glue
    └── spot_sar_executive/    # sense-plan-act-replan orchestrator
```

`spot_sar_sim/spot_sar_sim/standalone/`:
- `spot_view_scene.py` — **minimal viewer (no ROS):** boots the Isaac Sim GUI, loads the SAR environment (walled room + victim markers) and Spot (standing), and renders. The simplest way to *see* the scene and the first step-by-step bring-up check before adding ROS. GUI by default; `--headless` to drop the window.
- `spot_smoke_test.py` — fast physics-only validation (Spot walks ~2.76 m, no rendering). The go-to health check.
- `spot_smoke_render.py` — RTX render-path validation (slow first run; for camera-sensor work later).
- `spot_cmd_vel_app.py` — Phase 1 app: drive Spot from ROS 2 `/cmd_vel`; bridge publishes `/clock`, `/joint_states`, `/odom`, `/tf`.
- `spot_perception_app.py` — Phase 2 app: superset of the Phase 1 app + a body-mounted RGB-D camera (`/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/rgb/camera_info`). **Rendering on** (the camera needs the RTX render path), unlike the physics-only Phase 1 app.
- `sar_scene.py` — shared SAR environment builder (walled room + multiple orange "victim" markers + a distractor, with `UsdSemantics` class labels); used by the live sim **and** the dataset generator.
- `replicator_sar_sdg.py` — Phase 2 synthetic-data generator: NVIDIA Replicator renders randomized viewpoints → a labelled dataset (RGB + 2D bbox + semantic segmentation, classes `victim`/`distractor`) under `~/unige_ws/datasets/sar_victims/`. Trains a learned detector (YOLO) to replace the HSV first cut.

## Helper scripts

Two wrappers hide the environment footguns so you never invoke Isaac's `python.sh` or a raw
`docker run` by hand.

### `scripts/run_isaac.sh` — launch an Isaac Sim standalone app

```bash
./scripts/run_isaac.sh <script.py> [args...]
# e.g.
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_smoke_test.py
ROS_DOMAIN_ID=42 ./scripts/run_isaac.sh \
    spot_sar_sim/spot_sar_sim/standalone/replicator_sar_sdg.py --frames 200
```

What it does, in order: (1) `conda deactivate` (twice — the auto-activated `base` env's Python 3.13
shadows ROS/Isaac); (2) source ROS 2 Jazzy + the `~/unige_ws` colcon overlay (so the in-process
`rclpy` node and the `isaacsim.ros2.bridge` resolve `std_msgs` + `spot_sar_msgs`); (3) pin the real
asset root and `ROS_DOMAIN_ID`; (4) resolve the script to an absolute path **before** `cd`-ing into
`~/isaacsim`, then `exec ./python.sh` (Isaac's bundled Python 3.12, which matches Jazzy).

| Env var | Default | Purpose |
|---|---|---|
| `ISAAC_DIR` | `~/isaacsim` | Isaac Sim workstation install (where `python.sh` lives). |
| `WS_DIR` | `~/unige_ws` | colcon workspace whose `install/setup.bash` is overlaid. |
| `ROS_DOMAIN_ID` | `42` | DDS domain — must match every `ros2` CLI shell. |
| `ISAAC_ASSETS` | `~/isaacsim_assets` | Real local asset root (the persistent config points at a stale `/home/isaac` path). |

### `docker/run_unige_docker.sh` — start the ROS-side container

Isaac Sim stays on the **host**; this runs the `thanhnc19/unige_legged` image (Nav2, slam_toolbox,
perception, planner) with this repo bind-mounted and host networking, so its nodes discover the
Isaac ROS 2 bridge over DDS.

```bash
./docker/run_unige_docker.sh                                   # interactive shell in the container
CMD="ros2 launch spot_sar_bringup perception.launch.py" \
    ./docker/run_unige_docker.sh                               # run one command, then exit
```

It mounts the repo at `/opt/unige_ws/src/quadruped_isaac_sim` (build/install stay container-side),
adds `--gpus all` when `nvidia-smi` is present, and shares `--net=host --ipc=host` + `ROS_DOMAIN_ID`
+ the X11 socket. Override `IMAGE`, `TAG`, `ROS_DOMAIN_ID`, or `CMD` via env vars. Build/push the
image with `docker/build_docker.sh` / `docker/push_docker.sh`; see [docker/README.md](docker/README.md).

## Installation (prerequisites)

This machine is already provisioned (✓ = present here). The steps below document how to
reproduce the setup on a fresh **Ubuntu 24.04** box and what each piece is for; run them in order.

```bash
# 1. NVIDIA driver  (✓ 580)  — RTX GPU + driver >= 535.129.03 (Isaac Sim 6.0 floor)
nvidia-smi                                   # confirm GPU + driver are live

# 2. Isaac Sim 6.0  (✓ ~/isaacsim, assets ✓ ~/isaacsim_assets)
#    Download the Isaac Sim 6.0 *workstation* package + asset pack from NVIDIA, unzip to those dirs.
cat ~/isaacsim/VERSION                       # expect 6.0.0-...
~/isaacsim/python.sh -c "import isaacsim; print('isaacsim import OK')"

# 3. ROS 2 Jazzy + dev tools  (✓)  — ros-dev-tools provides colcon, rosdep, vcstool
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 4. Navigation / SLAM / sensor bridges  (✓ except pointcloud_to_laserscan)
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
                    ros-jazzy-depthimage-to-laserscan
# (later, only if /scan is synthesized from a 3D cloud:)
# sudo apt install -y ros-jazzy-pointcloud-to-laserscan

# 5. Miniconda  (✓ ~/miniconda3) — OPTIONAL and not used by this project. Its base env
#    (Python 3.13) shadows ROS/Isaac, so always `conda deactivate` first (run_isaac.sh does this).

# 6. Planning venv  (✓ ~/sar_planning_venv) — PDDL solving deps.
#    Fast Downward compiles from source (needs cmake/g++); ENHSP is JVM-based (needs a JRE).
sudo apt install -y python3.12-venv python3.12-dev build-essential cmake default-jre
python3.12 -m venv --system-site-packages ~/sar_planning_venv   # --system-site-packages => rclpy importable
source ~/sar_planning_venv/bin/activate
pip install --upgrade pip
pip install unified-planning up-fast-downward up-enhsp
python -c "from unified_planning.shortcuts import get_environment; print(list(get_environment().factory.engines)[:5])"
deactivate
```

> `ROS_DOMAIN_ID` is used to isolate DDS traffic; this project standardizes on **42**. Export the
> same value in every shell (Isaac app and any `ros2` CLI) or they won't discover each other:
> `export ROS_DOMAIN_ID=42`.

## Quick start

```bash
# 1. Build the ROS 2 workspace (conda MUST be deactivated; Jazzy uses Python 3.12)
conda deactivate
source /opt/ros/jazzy/setup.bash
cd ~/unige_ws && colcon build --symlink-install && source install/setup.bash

# 2. Health check — Spot walks in Isaac Sim (headless, physics-only, ~25 s)
cd ~/unige_ws/src/quadruped_isaac_sim
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_smoke_test.py
#   expect: "[smoke] SUCCESS: Spot loaded its policy and walked forward."

# 3. Phase 1 — Spot driven by ROS 2 /cmd_vel (headless)
ROS_DOMAIN_ID=42 ./scripts/run_isaac.sh \
    spot_sar_sim/spot_sar_sim/standalone/spot_cmd_vel_app.py
# In another shell (conda deactivated, Jazzy sourced, SAME ROS_DOMAIN_ID=42):
#   ros2 topic echo /clock --once
#   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.6}}'

# 4. Phase 2 — Spot + RGB-D camera + victim detector (headless, rendering on)
ros2 launch spot_sar_bringup perception.launch.py
# In another shell (SAME ROS_DOMAIN_ID=42):
#   ros2 topic hz /camera/rgb/image_raw          # ~25 Hz
#   ros2 topic echo /camera/rgb/camera_info --once
#   ros2 topic echo /victims                     # spot_sar_msgs/VictimArray
# First launch triggers a slow RTX shader compile (cold cache) before frames appear.
```

### Watch it live — Isaac Sim GUI window

The standalone apps run **headless by default** (no window — best for the 8 GB VRAM budget and for
data generation). Pass **`--gui`** to open the Isaac Sim viewport and *see* the environment + Spot.
The perception app is the one to watch: it builds the full **SAR room (walls + victim markers)** and
spawns Spot, so the viewport shows the whole scene.

```bash
cd ~/unige_ws/src/quadruped_isaac_sim
export DISPLAY=:0                       # your X display (run `echo $DISPLAY` in a desktop terminal)

# Simplest: just the scene — SAR environment + Spot, no ROS (recommended first step)
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py

# …or the perception app: same SAR environment + Spot, plus the RGB-D camera + ROS bridge
ROS_DOMAIN_ID=42 ./scripts/run_isaac.sh \
    spot_sar_sim/spot_sar_sim/standalone/spot_perception_app.py --gui

# …or just Spot on a ground plane (Phase 1 locomotion app) with the window:
ROS_DOMAIN_ID=42 ./scripts/run_isaac.sh \
    spot_sar_sim/spot_sar_sim/standalone/spot_cmd_vel_app.py --gui

# Then drive Spot from another shell (SAME ROS_DOMAIN_ID=42) and watch it move:
#   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear:  {x: 0.6}}'   # walk forward
#   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.5}}'   # turn in place
```

### Drive Spot from the keyboard

Instead of one-shot `ros2 topic pub`, use the interactive keyboard teleop — it streams `/cmd_vel`
while you steer with **WASD** (`w/s` fwd/back, `a/d` turn, `q/e` strafe, **space** = stop, `+/-` speed):

```bash
export ROS_DOMAIN_ID=42        # MUST match the sim's domain (see the footgun below)
ros2 run spot_sar_sim teleop_keyboard
# focus this terminal and press keys; Ctrl-C sends a final stop and quits
# (zero-code alternative, also installed: `ros2 run teleop_twist_keyboard teleop_twist_keyboard`)
```

> **`ROS_DOMAIN_ID` must match.** DDS only connects nodes on the *same* domain. If the sim is on
> domain 42 but your teleop/publisher shell is on a different domain (e.g. an unset/`0` default), the
> command never reaches Spot and it just stands still. Run `echo $ROS_DOMAIN_ID` in **both** the sim
> terminal and the teleop terminal — the numbers must be equal. This project standardizes on **42**.

> **GUI notes:** the window opens on the X server named by `DISPLAY` (a desktop session, e.g. `:0`/`:1`).
> The first GUI launch compiles RTX shaders (cold Blackwell cache) and can take a minute before the
> viewport appears — subsequent launches are fast. `--gui` adds the window + extra render load on top
> of the already-on camera render path, so keep it to one app at a time on 8 GB VRAM.

### Visualize in RViz (recommended for the full stack)

Rather than the heavy Isaac GUI, run Isaac **headless** and watch everything in **RViz2** over
ROS 2 — lighter on 8 GB VRAM and no shader-compile freeze. Everything in the stack is already a
ROS topic, so RViz shows the raw camera, the **detection overlay**, the lidar scan, the SLAM map,
Nav2 costmaps/paths, victim markers, and TF.

```bash
# Terminal A — bring the stack up HEADLESS (no --gui), on domain 42
export ROS_DOMAIN_ID=42
ros2 launch spot_sar_bringup perception.launch.py     # camera + detector
#   or mapping.launch.py (adds /scan + SLAM /map), or mission.launch.py (full autonomy)

# Terminal B — open RViz preloaded with the SAR config, SAME domain
export ROS_DOMAIN_ID=42
ros2 launch spot_sar_bringup rviz.launch.py
```

Preloaded displays (`spot_sar_bringup/rviz/sar.rviz`, fixed frame `odom`):

| Display | Topic | Shows |
|---|---|---|
| Camera RAW | `/camera/rgb/image_raw` | the raw RGB stream |
| Detections | `/camera/rgb/detections` | RGB **+ green detection boxes** + confidence/range labels |
| Victims | `/victims/markers` | 3D spheres at detected victim positions (in `odom`/`map`) |
| LaserScan | `/scan` | the depth-derived lidar (mapping/mission) |
| Map | `/map` | the slam_toolbox occupancy grid (mapping/mission) |
| Global/Local Costmap | `/global_costmap/costmap`, `/local_costmap/costmap` | Nav2 costmaps (off by default — tick to enable) |
| Nav2 Path | `/plan` | the planned path |
| TF | — | the `odom→base_link→camera_*` frames |

The detector publishes `/camera/rgb/detections` (annotated image) and `/victims/markers` **only when
RViz subscribes**, so they add no overhead during headless autonomy runs. RViz must be on the same
`ROS_DOMAIN_ID` as the sim — see the footgun note above.

> **Docker:** a ROS 2 environment image (`thanhnc19/unige_legged`) is provided under
> [docker/](docker/) for the ROS-side stack (Nav2, slam_toolbox, perception, planner).
> Isaac Sim stays a host install; the container talks to it over DDS. See [docker/README.md](docker/README.md).

## Machine-specific footguns (handled by `scripts/run_isaac.sh` + the apps)

1. **conda `base` auto-activates** (Python 3.13) and shadows ROS/Isaac → always `conda deactivate` first.
2. **Stale asset root**: Isaac's persistent config points at a Docker path `/home/isaac/...`. Real
   assets are `~/isaacsim_assets`; the apps pin `persistent.isaac.asset_root.default` at startup.
3. **Stale Kit cache** (`${omni_cache}` → `/home/isaac/...`, not writable here): the apps redirect it
   to `~/.cache/isaacsim_omni_cache` so OGN node generation persists (else the ROS 2 bridge
   intermittently hangs at startup).
4. **8 GB VRAM** is the *minimum* tier — keep render resolution low and add sensors one at a time.
5. **First-run RTX shader compilation is slow** (cold Blackwell cache); locomotion dev uses the
   physics-only path (`world.step(render=False)`) to skip it entirely.
6. **Camera needs rendering ON** — `spot_perception_app.py` must run the RTX render path (a render
   product produces no image in the physics-only mode used for locomotion). Keep one camera at
   640×480 with the render decoupled (~25 Hz) to fit the 8 GB VRAM budget.
7. **Isaac camera optical frame** — Isaac's USD camera looks down its local −Z, but the bridge
   publishes pixels + intrinsics in the ROS optical convention (z-fwd, x-right, y-down). Images
   are stamped `camera_optical_frame`; the `camera_link→camera_optical_frame` leg is a REP-103
   static TF (launch). Validate with `ros2 run tf2_ros tf2_echo base_link camera_optical_frame`.
