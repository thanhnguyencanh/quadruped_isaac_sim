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
| YOLO venv | `~/yolo_venv` — ultralytics 8.4 + CPU torch 2.12 (numpy pinned 1.26 to match ROS), YOLOv8n (verified detecting Isaac humans) |

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

## Installation

Steps 1–2 (host) are required either way:

```bash
# 1. NVIDIA driver — RTX GPU + driver (Isaac Sim 6.0)
nvidia-smi                                   # confirm GPU + driver are live

# 2. Isaac Sim 6.0 — download the *workstation* package + asset pack from NVIDIA, unzip to these dirs
cat ~/isaacsim/VERSION                       # expect 6.0.0-...  (assets in ~/isaacsim_assets)
~/isaacsim/python.sh -c "import isaacsim; print('isaacsim import OK')"
```

### Docker (recommended, ROS side only) — the `unige_legged` image

The `thanhnc19/unige_legged` image ships the entire ROS 2 side already built: **Jazzy + Nav2 +
slam_toolbox + RGB-D perception incl. the YOLO detector + OctoMap/grid_map 3D mapping + the PDDL
planner** (`/opt/sar_planning_venv`). 

```bash
docker pull thanhnc19/unige_legged            # or ./docker/build_docker.sh to build it locally
./docker/run_unige_docker.sh                  # repo mounted, host net + GPU, ROS_DOMAIN_ID=42
#   then inside the container:
#     cd /opt/unige_ws && colcon build --symlink-install && source install/setup.bash
#     ros2 topic list          # should show the host Isaac bridge topics on domain 42
```

Isaac Sim stays on the host; it and the container share `ROS_DOMAIN_ID=42` + host networking, so the
container's nodes discover the Isaac ROS 2 bridge over DDS. See [docker/README.md](docker/README.md).

### Native install — ROS side on the host

Install the ROS stack + the two Python venvs directly (this is what this machine runs):

```bash
# 3. ROS 2 Jazzy + dev tools — ros-dev-tools provides colcon, rosdep, vcstool
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc

# 4. Navigation / SLAM / sensor bridges
sudo apt install -y ros-jazzy-navigation2 ros-jazzy-nav2-bringup ros-jazzy-slam-toolbox \
                    ros-jazzy-depthimage-to-laserscan
# (later, only if /scan is synthesized from a 3D cloud:)
# sudo apt install -y ros-jazzy-pointcloud-to-laserscan

# 5. Planning venv (~/sar_planning_venv) — PDDL solving deps.
#    Fast Downward compiles from source (needs cmake/g++); ENHSP is JVM-based (needs a JRE).
sudo apt install -y python3.12-venv python3.12-dev build-essential cmake default-jre
python3.12 -m venv --system-site-packages ~/sar_planning_venv   # --system-site-packages => rclpy importable
source ~/sar_planning_venv/bin/activate
pip install --upgrade pip
pip install unified-planning up-fast-downward up-enhsp
python -c "from unified_planning.shortcuts import get_environment; print(list(get_environment().factory.engines)[:5])"
deactivate

# 6. YOLO venv (~/yolo_venv) — the learned victim detector (pretrained YOLOv8, no training).
#    --system-site-packages => rclpy/cv_bridge import; numpy PINNED to 1.26 to match ROS's ABI
#    (cv_bridge is built against numpy 1.x; torch/opencv would otherwise pull numpy 2.x and clash).
python3.12 -m venv --system-site-packages ~/yolo_venv
source ~/yolo_venv/bin/activate && pip install --upgrade pip
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu   # CPU wheel (~200 MB)
pip install ultralytics
pip install "numpy==1.26.4" "opencv-python==4.9.0.80"     # MUST pin to the system numpy ABI
yolo predict model=yolov8n.pt source=https://ultralytics.com/images/bus.jpg      # fetch weights once
cp yolov8n.pt ~/yolo_venv/yolov8n.pt                      # pin absolute path (no re-download at launch)
deactivate
```

> **`ROS_DOMAIN_ID=42`** must be the same in every shell (the Isaac app + each `ros2` CLI) or they
> can't discover each other; `run_unige_docker.sh` sets it inside the container for you. (Why it
> matters, and the failure symptom, are in the Quick-start gotchas below.)

## Quick start

```bash
# 0. Once per shell: drop conda, source ROS + the workspace, pick the DDS domain.
source /opt/ros/jazzy/setup.bash
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

# 3. Phase 1 — drive Spot from ROS 2 /cmd_vel (GUI by default; append --headless to drop the window)
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_cmd_vel_app.py
#   another shell, SAME ROS_DOMAIN_ID=42 — drive Spot:
#     ros2 run spot_sar_sim teleop_keyboard                                       # WASD teleop (w/s a/d q/e, space=stop)
#     ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{linear: {x: 0.6}}'  # or a one-shot publish

# 4. Phase 2 — RGB-D camera + victim detector (GUI by default; gui:=false for headless)
ros2 launch spot_sar_bringup perception.launch.py
#   visualize EVERYTHING in RViz (another shell, SAME domain) — raw + detection image, /victims, /scan, /map:
#     ros2 launch spot_sar_bringup rviz.launch.py

# 5. Full autonomy mission — Isaac + SLAM + Nav2 + PDDL executive, one command
ros2 launch spot_sar_bringup mission.launch.py          # single-room SAR mission
ros2 launch spot_sar_bringup floor_mission.launch.py    # multi-room floor + openable doors (PDDL open-door)
```

> **Two recurring gotchas.** (1) Every shell — the Isaac app **and** every `ros2` command — must export the
> **same `ROS_DOMAIN_ID`** (this project uses **42**), or DDS can't connect them and Spot just stands still
> (`echo $ROS_DOMAIN_ID` in both shells must match). (2) The first GUI launch (the default) compiles RTX
> shaders (cold cache) and may sit on a black *"not responding"* window for ~1 min — click **Wait**, not
> Force Quit. On small (≤8 GB) GPUs, prefer **headless + RViz** (`--headless` / `gui:=false`) over the Isaac GUI.

## Details

### Helper scripts

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

| Env var | Default | Purpose |
|---|---|---|
| `ISAAC_DIR` | `~/isaacsim` | Isaac Sim workstation install (where `python.sh` lives). |
| `WS_DIR` | `~/unige_ws` | colcon workspace whose `install/setup.bash` is overlaid. |
| `ROS_DOMAIN_ID` | `42` | DDS domain — must match every `ros2` CLI shell. |
| `ISAAC_ASSETS` | `~/isaacsim_assets` | Real local asset root (the persistent config points at a stale `/home/isaac` path). |

### Isaac Sim environments

Three SAR scenes ship in `spot_sar_sim/.../standalone/sar_scene.py`, picked by a flag on the
standalone apps (all share the same victim markers / Isaac People + `UsdSemantics` labels):

| Environment | Flag | Scene builder | What's in it |
|---|---|---|---|
| **Single room** (default) | *(none)* | `build_sar_scene()` | a 20×20 m collapsed interior: outer walls + 2 m interior wall segments (lidar structure + occlusion frontiers) + victims (all beyond the lidar's 5 m first scan, so exploration is required) + a distractor — the Phase 0–6 mission |
| **3-room floor + doors** | `--floor` | `build_floor_scene()` | rooms A–B–C, divider walls with door gaps + sliding door slabs (PDDL `open-door`) |
| **Two-floor building + stairwell** | `--building` | `build_two_floor_scene()` | two x-offset floors, floor-1 doors + a stairwell landing (PDDL `use-stairs`) |

**Check a sim environment** — the no-ROS viewer (`spot_view_scene.py`) boots Isaac, loads the scene +
Spot (standing) and renders it, so you can eyeball geometry/lighting before wiring any ROS. GUI on by
default (needs `export DISPLAY=:0`); `--headless` just validates that the scene loads. **Run these on
the HOST, not in the `unige_legged` container** — Isaac Sim is a host install, so `run_isaac.sh` inside
the container fails with `cd: /root/isaacsim: No such file or directory`.

```bash
# quickest visual check of each environment (orbit with the mouse; Ctrl-C to quit):
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py             # single room
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py --floor     # 3-room floor + doors
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py --building  # two-floor building
./scripts/run_isaac.sh spot_sar_sim/spot_sar_sim/standalone/spot_view_scene.py --headless  # load-only check, no window
```

The viewer is **static** — doors stay closed and the stair teleport is idle (it hosts no ROS bus). To
*drive* the doors/stairs, use the ROS apps in the sections below (`spot_perception_app.py --floor` /
`--building`).

#### Multi-room floor with openable doors (PDDL `open-door`)

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
    spot_sar_sim/spot_sar_sim/standalone/spot_perception_app.py --floor       # GUI by default (+--headless to drop it)
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
live door state.

#### Two-floor building with a stairwell (PDDL `use-stairs`)

The third environment: a **two-storey building** — three rooms per floor, victims on **both floors**,
joined by a **stairwell**. The planner must dispatch a **`use-stairs`** action (the floor analogue of
`open-door`) to change levels. Another **opt-in parallel path**; the single-room and floor missions
are unchanged.

**The design in one idea — "the door pattern, applied vertically."** Spot's `SpotFlatTerrainPolicy`
is trained on flat ground and *cannot* climb real steps, and slam_toolbox keeps a single 2D map that a
vertically-stacked building would corrupt. So:

- **Floors are x-offset** (`f1` rooms at x∈[-9, 9], `f2` rooms at x∈[15, 33]) → their footprints are
  **disjoint in the one 2D map**, no per-floor maps or map-switching needed. Nav2's rolling,
  scan-driven costmap serves both wings unchanged.
- **…except the stair landing**, which is **vertically stacked**: `f1_stair` and `f2_stair` share the
  exact same `(x,y)=(10.5, 0)`, differing only in z (0.0 vs 3.0). So the floor change is a **pure-z
  teleport** `(10.5,0,0.8)↔(10.5,0,3.8)`: odom x,y stay constant while z jumps ±3 m, so slam's
  **planar `map→odom` sees ~no discontinuity** (verified: odom `x:0.12, y:-0.02, z:2.68` after a climb).
  A real staircase is rendered at the landing for the camera / YOLO / 3D-mapping (its steps rise to
  meet the floor-2 slab at z=3.0 so it reads as connected); **Spot never drives onto it** — the
  teleport does the level change.

> **Why teleport instead of real climbing?** `SpotFlatTerrainPolicy` is trained on flat ground and
> **cannot physically walk up steps**, so the staircase is decorative and the level change is the
> pure-z teleport. **TODO / future work — real stair-climbing:** swap in a stairs-capable locomotion
> policy, give the steps true collision-stepping, and retire the teleport (which also means going
> beyond the single 2D map). Substantial work; the teleport is the deliberate abstraction for now.

Run the **full closed loop**: `ros2 launch spot_sar_bringup building_mission.launch.py` — the executive
plans `open-door` (floor 1) → reaches + reports the floor-1 victim → `move` to the landing →
`use-stairs` → **StairsNode teleports Spot up** → grounds on floor 2 → traverses → reports the
floor-2 victim. To drive/inspect by hand, run the app with `--building` and command the stairs on
`/stairs_cmd` (`--spawn 10.5 0 0.8` starts Spot on the landing):

```bash
ROS_DOMAIN_ID=42 ./scripts/run_isaac.sh \
    spot_sar_sim/spot_sar_sim/standalone/spot_perception_app.py --building --spawn 10.5 0 0.8   # GUI by default
ros2 topic pub --once /stairs_cmd std_msgs/msg/String '{data: "stair_main up"}'    # teleport to floor 2
ros2 topic pub --once /stairs_cmd std_msgs/msg/String '{data: "stair_main down"}'  # back to floor 1
ros2 topic echo /floor_state   # "f1" | "f2" once Spot arrives (latched)
```

How it fits together (single source of truth: `spot_sar_planning/spot_sar_planning/sar_building.py` —
rooms tagged by floor, portals, stairs, `room_of(x, y, floor)`):

| Layer | What it does |
|---|---|
| **Scene** | `sar_scene.build_two_floor_scene()` — two x-offset floor plates + per-floor walls + floor-1 doors + a decorative staircase + humans on both floors |
| **Stair bus** | the `--building` app hosts a `StairsNode` (the `DoorNode` of floors): `/stairs_cmd` (`"<id> up/down"`) pure-z-teleports Spot at the stacked landing; `/floor_state` (`"f1"/"f2"`, latched) announces the floor |
| **PDDL** | `domain_building.pddl` — a `stair` is the ONLY edge between the (x-disjoint) wings, so STRIPS is **forced to `use-stairs`** to reach floor 2 (verified with Fast Downward) |
| **Grounding** | `building_world_model_node` — room graph across both floors; the floor comes from the latched `/floor_state` (the two landings share `(x,y)`, so it is the authoritative tie-break) |
| **Executive / skill** | run with `domain_profile:=building`; dispatches the **`climb_stairs`** skill, which publishes `/stairs_cmd` and blocks until `/floor_state` confirms |

Verified component-by-component (feasibility-gated before the heavy run): PDDL solves the full
two-floor mission (`open-door` + `use-stairs` + reports both floors); the **teleport keeps odom x,y
constant while z jumps ±3 m** and Spot stands stably on floor 2; and the executive's building profile
dispatches `climb_stairs("stair_main up")`.

### Check perception and world model

**Victims: human figures + YOLO (default), or boxes + HSV (opt-out).** By default the victims are
**realistic Isaac People humans** detected by a **pretrained YOLOv8** (`person` class, no finetuning)
running on CPU from `~/yolo_venv`. Both detectors publish the same `/victims` (+ `/camera/rgb/detections`
overlay + `/victims/markers`); pick via launch args — `humans:=…` / `detector:=…` thread through
`mapping` → `sar_system` → `mission`/`floor_mission`:

```bash
ros2 launch spot_sar_bringup perception.launch.py                          # humans + YOLO (default)
ros2 launch spot_sar_bringup perception.launch.py humans:=false detector:=hsv   # orange boxes + HSV (no venv needed)
#   + building:=true for the two-floor scene
```
Perception (each in its own `ROS_DOMAIN_ID=42` shell):
```bash
ros2 topic hz  /camera/rgb/image_raw     # ~25 Hz (640×480, camera_optical_frame)
ros2 topic echo /victims                 # spot_sar_msgs/VictimArray — detections with range
# watch what the detector sees (green boxes + confidence) — lighter than full RViz:
ros2 run rqt_image_view rqt_image_view /camera/rgb/detections
# drive Spot around to face a victim (WASD: w/s fwd/back, a/d strafe, q/e turn, space = stop):
ros2 run spot_sar_sim teleop_keyboard
```
(The detector publishes the `/camera/rgb/detections` overlay only while something subscribes —
opening it in `rqt_image_view` is what turns it on.)
World model (symbol grounding — comes up with `mapping.launch.py` / `mission.launch.py`):
```bash
ros2 topic echo --once /world_model      # world_model_node: rooms + grounded victims (spot_sar_msgs/WorldModel)
```
**Pass:** `/victims` populates when Spot faces a victim (YOLO detects the Isaac human as `person` —
verified at **0.94** confidence, ~8 Hz) and `/world_model` reflects the grounded rooms/victims. In
`rviz.launch.py` the **Detections** overlay (green boxes) + **Victims** markers appear.

The YOLO node (`spot_sar_perception/yolo_detector_node.py`) **subclasses** the HSV `detector_node`,
reusing its depth back-projection + tf2 + overlay/markers — only the detection stage differs.

### Check SLAM (2D + 3D mapping), path planning and navigation

**Sensor split: the LIDAR maps and navigates; the camera only detects victims.** By default `/scan`
comes from a **stabilized virtual 360° lidar** in the sim app: PhysX rays cast horizontally from a
mount that follows the robot's position but *never its roll/pitch* — every scan is level by
construction (no tilt corruption, no floor strikes) with 360° coverage instead of the camera's 67°.
Range is capped at **5 m** — deliberately smaller than the 20 m room, so a single sweep cannot
finish the map and the frontier explorer always has unknown space to chase. No-return beams are
`inf` per REP-117 — **never republish them as finite values**: karto's scan matcher uses its
unfiltered readings, and a finite "miss ring" is rotation-invariant, so it out-correlates the
real walls and collapses every scan pose onto the previous one (the map degenerates into a 5 m
free disc that follows the robot). Division of labor: **slam's `/map` maps geometry** (walls,
carved free space along hit rays only), while the **nav2 costmaps carry "explored free space"**
(`inf_is_valid: true` raytrace-clears the inf beams) — the frontier explorer and the `explore`
skill therefore read `/global_costmap/costmap` (odom frame), not `/map`.
Walls, doors and **human victims are solid** (collision-verified: Spot driven full-speed into a wall
and a person stops at their face), so the lidar sees them and costmaps avoid them. Sim-idealized on
purpose — a real robot would need a gimbal or scan motion-compensation (report limitation).
`lidar:=false` restores the legacy camera-depth scan + tilt gate (for comparison/ablation).

`sar_system.launch.py` is the full nav stack **without** the executive (Isaac + lidar + camera +
`slam_toolbox` + Nav2), so you can send a goal by hand and watch Spot drive to it:

```bash
ros2 launch spot_sar_bringup sar_system.launch.py     # Isaac + SLAM + Nav2 (heavy; run on a fresh boot)
```
Then, in another shell (`export ROS_DOMAIN_ID=42`):
```bash
ros2 topic hz  /scan                     # ~12-15 Hz, 360 beams, frame base_scan
ros2 topic echo --once /map              # slam_toolbox occupancy grid → SLAM works
ros2 run tf2_ros tf2_echo map odom       # the map→odom transform is live
# path planning + navigation — send a Nav2 goal, watch Spot walk there
# (navigation runs in the ODOM frame — decoupled from SLAM latency; map-framed goals also work,
#  transformed once by bt_navigator):
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: odom}, pose: {position: {x: 2.0, y: 0.0}, orientation: {w: 1.0}}}}"
ros2 topic echo --once /plan             # the planned path
```
**Pass:** Nav2 logs **`Reached the goal!`** and Spot reaches the target. Easiest visually:
`ros2 launch spot_sar_bringup rviz.launch.py`, then drop a **2D Goal Pose** and watch `/map`, `/plan`, and the robot move.

> **Legacy camera-scan mode (`lidar:=false`).** The depth-derived scan tilts with the body, so
> `scan_tilt_gate` republishes `/scan_raw`→`/scan` only while |roll|/|pitch| ≤ `max_tilt_deg`
> (default **4°** — the floor-strike bound `atan(cam height / range_max)`), logging the pass rate
> every 5 s. Kept for the sim-to-real ablation story; the stabilized lidar makes it unnecessary.

**Exploration + skills (Phase 4).** Neither node is auto-started by `sar_system.launch.py`, so each
can be checked in isolation on top of it (every shell on `ROS_DOMAIN_ID=42`):

```bash
# frontier EXPLORATION — fully autonomous coverage, no goals from you:
ros2 run spot_sar_nav frontier_explorer --ros-args -p use_sim_time:=true
#   pass: Spot drives itself frontier-to-frontier, /map grows in RViz, log ends with "no frontiers"

# SKILLS — the /skill action the executive dispatches (schema: {skill, target} -> {success, message}):
ros2 run spot_sar_nav skill_server --ros-args -p use_sim_time:=true      # its own shell
ros2 action send_goal /skill spot_sar_msgs/action/Skill "{skill: explore, target: ''}" --feedback
ros2 action send_goal /skill spot_sar_msgs/action/Skill "{skill: observe, target: ''}"
ros2 action send_goal /skill spot_sar_msgs/action/Skill "{skill: go_to_location, target: L_1_0}"
```

`explore`/`observe` are self-contained (`/map` frontiers, `/victims` dwell). **`go_to_location` needs
`/world_model` running** (it resolves the target's centroid there) — echo `/world_model` and pick a real
id from `locations`. `open_door` / `climb_stairs` need the `--floor` / `--building` sim.

**3D mapping — OctoMap voxels + elevation map for the stairs.** The 2D SLAM map is complemented by
two **3D** maps, run *alongside* a live sim (they consume the RGB-D camera). Both are in
`mapping3d.launch.py`; the shared input is a camera **point cloud** (`depth_image_proc` →
`/camera/points`; `setup_3d_mapping.sh octomap` installs it along with the OctoMap stack):

| Map | Node | Output | RViz |
|---|---|---|---|
| **3D voxel (OctoMap)** | `octomap_server` (apt) | `/octomap_full` + `/octomap_point_cloud_centers` + projected `/projected_map` | voxel boxes coloured by height — the two floors show up at z≈0 and z≈3 |
| **3D elevation (stairs)** | leggedrobotics `elevation_mapping_cupy` (GPU/CuPy) | `/elevation_mapping_node/elevation_map` (`grid_map_msgs/GridMap`) | `grid_map_rviz_plugin` renders the `elevation` layer as a 3D surface — the steps as a height field |

```bash
# one-time install (needs sudo + internet; the unige_legged Docker image ships the OctoMap stack):
./scripts/setup_3d_mapping.sh octomap      # stage A: OctoMap + grid_map (apt) — GPU-free, low risk
./scripts/setup_3d_mapping.sh elevation    # stage B: elevation_mapping_cupy (source build + CuPy)

# then, with a --building sim running (e.g. building_mission.launch.py or perception.launch.py building:=true):
ros2 launch spot_sar_bringup mapping3d.launch.py                     # point cloud + OctoMap
ros2 launch spot_sar_bringup mapping3d.launch.py elevation:=true     # + elevation_mapping_cupy
ros2 launch spot_sar_bringup rviz.launch.py rviz_config:=$(ros2 pkg prefix spot_sar_bringup)/share/spot_sar_bringup/rviz/sar_3d.rviz
```
**Pass:** `/camera/points` streams and `/octomap_point_cloud_centers` fills in as Spot's camera looks
around — the two storeys appear as separate voxel bands at z≈0 and z≈3.

- **OctoMap** is the reliable, GPU-free 3D map — apt install and go. Point Spot's camera around and the
  octree fills in (the pure-z teleport keeps odom coherent, so the octree stays consistent across floors).
- **elevation_mapping_cupy** is the faithful leggedrobotics stack for the stairs, but it is a ROS 2
  **dev-branch source build** + **CuPy on the GPU**. It shares the VRAM with the Isaac RTX renderer →
  real OOM risk on small GPUs; if it won't co-run, keep OctoMap (which already captures the stairs as
  voxels) and run elevation on a lighter scene / lower resolution. `setup_3d_mapping.sh` builds it into
  a `~/elevation_venv` (CuPy + NumPy pinned to the ROS ABI) and clones the Jazzy branch.



### Check PDDL planning and task executive

The full closed loop — the planner solving + the executive driving Nav2:

```bash
ros2 launch spot_sar_bringup mission.launch.py        # single-room (also floor_/building_mission.launch.py)
```
**Pass:** the executive log runs the **SENSE→GROUND→PLAN→ACT→MONITOR→REPLAN** loop — Fast Downward
solves `spot_sar_planning/pddl/domain.pddl` from a `/world_model`-derived problem, the executive
dispatches skills (`go_to_location`→Nav2, `observe`, `report`), and autonomously **detects → navigates
to → REPORTS** victims, replanning each cycle. (The executive runs under `~/sar_planning_venv`; the
`unige_legged` container ships it.) Offline PDDL sanity (no Isaac): the planner solves `domain.pddl` +
a generated problem inside the planning venv.

### Visualize in RViz

`ros2 launch spot_sar_bringup rviz.launch.py` (Quick start step 4) opens RViz preloaded with
`spot_sar_bringup/rviz/sar.rviz` (fixed frame `odom`) — optionally run Isaac **headless**
(`--headless` / `gui:=false`) and watch the whole stack over ROS 2, lighter than the Isaac GUI and
with no shader-compile freeze. Preloaded displays:

| Display | Topic | Shows |
|---|---|---|
| Camera RAW | `/camera/rgb/image_raw` | the raw RGB stream |
| Detections | `/camera/rgb/detections` | RGB **+ green detection boxes** + confidence/range labels |
| Victims | `/victims/markers` | 3D spheres at detected victim positions (in `odom`/`map`) |
| LaserScan | `/scan` | the stabilized 360° lidar (mapping/mission) |
| Map | `/map` | the slam_toolbox occupancy grid (mapping/mission) |
| Global/Local Costmap | `/global_costmap/costmap`, `/local_costmap/costmap` | Nav2 costmaps (off by default — tick to enable) |
| Nav2 Path | `/plan` | the planned path |
| TF | — | the `odom→base_link→camera_*` frames |

The detector publishes `/camera/rgb/detections` + `/victims/markers` **only when RViz subscribes**, so
they cost nothing during headless autonomy runs. (RViz must share the sim's `ROS_DOMAIN_ID`.)

