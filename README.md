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
- `spot_view_scene.py` — **minimal viewer (no ROS):** boots the Isaac Sim GUI, loads the SAR environment and Spot (standing), and renders. The simplest way to *see* the scene and the first step-by-step bring-up check before adding ROS. GUI by default; `--headless` to drop the window; **`--floor` to view the 3-room floor + doors** (doors shown closed — opening them needs the ROS door bus in `spot_perception_app.py --floor`).
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

# 5. Planning venv  (✓ ~/sar_planning_venv) — PDDL solving deps.
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
# 0. Once per shell: drop conda, source ROS + the workspace, pick the DDS domain.
conda deactivate && source /opt/ros/jazzy/setup.bash
cd ~/unige_ws && colcon build --symlink-install && source install/setup.bash
export ROS_DOMAIN_ID=42      # SAME value in EVERY shell (sim + every ros2 CLI) or they can't see each other
cd ~/unige_ws/src/quadruped_isaac_sim

# 1. Health check — Spot walks (headless, physics-only, ~25 s)
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_smoke_test.py
#   expect: "[smoke] SUCCESS: Spot loaded its policy and walked forward."

# 2. Just LOOK at the environment + Spot (no ROS; GUI viewer — orbit with the mouse)
export DISPLAY=:0            # your desktop X display — run `echo $DISPLAY` in a desktop terminal
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py            # single room
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py --floor    # 3-room floor + doors

# 3. Phase 1 — drive Spot from ROS 2 /cmd_vel (headless; append --gui for the Isaac window)
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_cmd_vel_app.py
#   another shell, SAME ROS_DOMAIN_ID=42 — drive Spot:
#     ros2 run spot_sar_sim teleop_keyboard                                       # WASD teleop (w/s a/d q/e, space=stop)
#     ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.6}}'  # or a one-shot publish

# 4. Phase 2 — RGB-D camera + victim detector (headless)
ros2 launch spot_sar_bringup perception.launch.py
#   visualize EVERYTHING in RViz (another shell, SAME domain) — raw + detection image, /victims, /scan, /map:
#     ros2 launch spot_sar_bringup rviz.launch.py

# 5. Full autonomy mission — Isaac + SLAM + Nav2 + PDDL executive, one command
ros2 launch spot_sar_bringup mission.launch.py          # single-room SAR mission
ros2 launch spot_sar_bringup floor_mission.launch.py    # multi-room floor + openable doors (PDDL open-door)
```

> **Two recurring gotchas.** (1) Every shell — the Isaac app **and** every `ros2` command — must export the
> **same `ROS_DOMAIN_ID`** (this project uses **42**), or DDS can't connect them and Spot just stands still
> (`echo $ROS_DOMAIN_ID` in both shells must match). (2) The first **`--gui`** launch compiles RTX shaders
> (cold Blackwell cache) and may sit on a black *"not responding"* window for ~1 min — click **Wait**, not
> Force Quit. On 8 GB VRAM, prefer **headless + RViz** over the Isaac GUI.

### Visualize in RViz

`ros2 launch spot_sar_bringup rviz.launch.py` (Quick start step 4) opens RViz preloaded with
`spot_sar_bringup/rviz/sar.rviz` (fixed frame `odom`) — run Isaac **headless** and watch the whole
stack over ROS 2, lighter than the Isaac GUI and with no shader-compile freeze. Preloaded displays:

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

The detector publishes `/camera/rgb/detections` + `/victims/markers` **only when RViz subscribes**, so
they cost nothing during headless autonomy runs. (RViz must share the sim's `ROS_DOMAIN_ID`.)

## Multi-room floor with openable doors (PDDL `open-door`)

A more complex environment: a floor of **3 rooms (A–B–C)** separated by walls and connected by
**doors**, with victims behind them. The PDDL planner must dispatch an **`open-door`** action — which
**physically slides the door slab open in Isaac** — before it can `move` into the next room. It's an
**opt-in parallel path**: the single-room mission (`mission.launch.py`) is unchanged.

The **full closed loop** is `ros2 launch spot_sar_bringup floor_mission.launch.py` (Quick start step 5):
the executive plans `open-door` → the slab slides open → Spot moves through → detect → report. To
drive/inspect the floor by hand, run the app with `--floor` and command doors on `/door_cmd`
(ids: `door_ab` = rooms A–B, `door_bc` = rooms B–C):

```bash
ROS_DOMAIN_ID=42 ./scripts/run_isaac.sh \
    spot_sar_sim/spot_sar_sim/standalone/spot_perception_app.py --floor       # add --gui to watch
ros2 topic pub --once /door_cmd std_msgs/msg/String '{data: door_bc}'          # open  (bare id = open)
ros2 topic pub --once /door_cmd std_msgs/msg/String '{data: "door_bc close"}'  # close
ros2 topic echo /door_states   # "<id> open" | "<id> closed" once the slab arrives (latched)
```

How it fits together (single source of truth: `spot_sar_planning/spot_sar_planning/sar_floor.py` —
rooms, doors, victims, `room_of`):

| Layer | What it does |
|---|---|
| **Scene** | `sar_scene.build_floor_scene()` — perimeter + divider walls (door gaps) + a sliding **door slab** per door, all with colliders |
| **Door bus** | the `--floor` app hosts a `DoorNode`: `/door_cmd` (`"<id>"`/`"<id> open"`/`"<id> close"`) slides the slab open/closed over ~1 s; `/door_states` (`"<id> open/closed"`) announces it (latched) |
| **PDDL** | `domain_doors.pddl` — `move ?from ?to ?d` requires `(door-open ?d)`; only `open-door` opens a door ⇒ the planner is **forced to open before traversing** (verified with Fast Downward) |
| **Grounding** | `floor_world_model_node` — a **room-graph** `/world_model` (rooms + doors + live `door_open`) |
| **Executive / skill** | run with `domain_profile:=doors`; dispatches the **`open_door`** skill, which publishes `/door_cmd` and blocks until `/door_states` confirms |

Verified component-by-component: the PDDL gating (`open-door` before `move`), the executive's
doors-profile planning, the slab physically opening on `/door_cmd`, and the grounding reflecting the
live door state. *(Phase 2, planned: swap the HSV detector for a pretrained YOLO person/object
detector — no custom training.)*

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
