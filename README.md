# quadruped_isaac_sim — Spot SAR in NVIDIA Isaac Sim + PDDL planning

A Boston Dynamics **Spot** quadruped, simulated in **NVIDIA Isaac Sim**, explores an
unknown disaster-like indoor environment, detects victims, and reports them — with a
**PDDL planner** (unified-planning + Fast Downward / ENHSP) autonomously selecting the
next action inside a closed **SENSE → GROUND → PLAN → ACT → MONITOR → REPLAN** loop.

University of Genoa (UNIGE) / DIBRIS research grant **D.R. 2237/26** —
supervisor Prof. Antonio Sgorbissa. Simulation-first, 3 months, remote-capable.

## This machine's stack (verified)

| Layer | Version |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| Simulator | Isaac Sim **6.0.0-rc.59** (workstation install at `~/isaacsim`) |
| GPU | NVIDIA RTX 5060, 8 GB VRAM (Blackwell sm_120), driver 580 / CUDA 12.9 |
| ROS 2 | **Jazzy** (Python 3.12) |
| Nav2 / SLAM | `ros-jazzy-navigation2`, `ros-jazzy-slam-toolbox`, `depthimage_to_laserscan` (installed) |
| Planner venv | `~/sar_planning_venv` — unified-planning 1.3.0 + Fast Downward + ENHSP (verified) |

> The implementation guide in [docs/](docs/) was originally written for Ubuntu 22.04 / ROS 2
> Humble / Python 3.10; it has been patched in place to this stack (see the "STACK ADAPTATION"
> callout at its top). The port is clean because Isaac Sim 6.0 bundles Python **3.12** (matches
> Jazzy) and its `isaacsim.ros2.bridge` ships Jazzy libraries. The binding constraint is **8 GB VRAM**.

## Status

| Phase | State |
|---|---|
| **Phase 0** — workspace + env | ✅ Done (commit `5a8c2d3`). 8 packages build; Spot loads its flat-terrain policy and **walks ~2.76 m** headless (physics-only smoke test). |
| **Phase 1** — Spot + ROS 2 bridge | ✅ Working. `spot_cmd_vel_app.py`: Spot **walks ~11 m on a ROS 2 `/cmd_vel`** Twist and stands stably at zero command; the bridge publishes `/clock`, `/joint_states`, `/odom`, `/tf` (all visible to system ROS 2 Jazzy on `ROS_DOMAIN_ID=42`). Verified end-to-end via `ros2 launch spot_sar_bringup sim.launch.py`. |
| **Phase 2** — RGB-D perception | ✅ Camera bridge working. `spot_perception_app.py` adds a body-mounted RGB-D camera; the bridge publishes `/camera/rgb/image_raw` (~25 Hz), `/camera/depth/image_raw`, `/camera/rgb/camera_info` (640×480, `camera_optical_frame`) + the `base_link→camera_link→camera_optical_frame` TF. `detector_node` (HSV + depth back-projection) publishes `spot_sar_msgs/VictimArray` on `/victims`. One launch: `ros2 launch spot_sar_bringup perception.launch.py`. |
| **Phase 3** — SLAM + world model | ✅ Done. Depth→`/scan` (`depthimage_to_laserscan`, ~48 Hz, `camera_link` frame); `world_model_node` (symbol grounding → `spot_sar_msgs/WorldModel` on `/world_model`); `slam_toolbox` publishes **`/map`** (verified: 204×187 @ 0.05 m of the walled room) + `map→odom`. Fix: slam_toolbox is a **lifecycle node** — `slam.launch.py` includes slam_toolbox's `online_async_launch.py` so it gets CONFIGURE+ACTIVATE (a bare `Node` stays unconfigured and never subscribes `/scan`). `ros2 launch spot_sar_bringup mapping.launch.py`. |
| Planning env | ✅ `~/sar_planning_venv` solves with Fast Downward / ENHSP; `rclpy` + `unified_planning` coexist. |
| **Phase 4** — Nav2 + exploration | 🟡 Implemented (verification needs a fresh boot). `spot_sar_nav`: `nav2_params.yaml` + `nav2.launch.py` (RPP controller, NavFn planner, `/scan` costmaps, Twist `/cmd_vel`); `frontier_explorer` node (frontier detection unit-tested) drives coverage via `NavigateToPose`. |
| **Phase 5** — PDDL planning | ✅ Domain + problem generation verified. `spot_sar_planning/pddl/domain.pddl` (move/explore/detect/report) solves via Fast Downward; `planner.problem_pddl_from_worldmodel()` turns a `WorldModel` into a problem that also solves (move→…→explore→detect→report). |
| **Phase 6** — Task executive | 🟡 Implemented (verification needs Nav2 up). `spot_sar_executive/task_executive`: the SENSE→GROUND→PLAN→ACT→MONITOR→REPLAN loop — reads `/world_model`, builds+solves a PDDL problem, dispatches the first action (move/explore→`NavigateToPose`, report→logged), replans each cycle. Loop logic verified end-to-end (incl. replan shortening on arrival). |
| Phase 7 | ⬜ Evaluation & deliverables. |
| Full stack | `ros2 launch spot_sar_bringup sar_system.launch.py` (mapping+Nav2) + `world_model_node` + `task_executive` (in the planning venv). **Heavy — run on a freshly booted machine.** |

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
- `spot_smoke_test.py` — fast physics-only validation (Spot walks ~2.76 m, no rendering). The go-to health check.
- `spot_smoke_render.py` — RTX render-path validation (slow first run; for camera-sensor work later).
- `spot_cmd_vel_app.py` — Phase 1 app: drive Spot from ROS 2 `/cmd_vel`; bridge publishes `/clock`, `/joint_states`, `/odom`, `/tf`.
- `spot_perception_app.py` — Phase 2 app: superset of the Phase 1 app + a body-mounted RGB-D camera (`/camera/rgb/image_raw`, `/camera/depth/image_raw`, `/camera/rgb/camera_info`) and an orange "victim" marker. **Rendering on** (the camera needs the RTX render path), unlike the physics-only Phase 1 app.

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
