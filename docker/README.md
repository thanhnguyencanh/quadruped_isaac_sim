# `unige_legged` — Spot SAR ROS 2 environment image

A Docker image (`thanhnc19/unige_legged`) that ships the **ROS 2 side** of the Spot SAR
stack: ROS 2 **Jazzy** + Nav2 + slam_toolbox + the RGB-D perception dependencies + the PDDL
planner (unified-planning + Fast Downward + ENHSP). Modeled on the
[`hesfm` docker](../../../hesfm_ws/src/hesfm/docker): the image is an **environment only** —
your source is **not** baked in; mount the colcon workspace at run time and build it inside.

## What this image is / isn't

- ✅ Runs the **perception / nav / planning / executive** ROS 2 nodes.
- ❌ Does **not** contain NVIDIA Isaac Sim. Isaac Sim 6.0 is a **host** workstation install
  (GPU + RTX driver bound to the machine). The simulator runs on the host and talks to the
  containerized nodes over DDS — same `ROS_DOMAIN_ID` (42), host networking.

```
 ┌─────────── host ───────────┐        ┌──── unige_legged container ────┐
 │ Isaac Sim 6.0 + ROS 2 bridge│  DDS   │ Nav2 / slam_toolbox / detector │
 │ /clock /odom /tf /camera/*  │◄──────►│ planner / executive            │
 └─────────────────────────────┘ domain └────────────────────────────────┘
                                   42
```

## Usage

```bash
# 1. Build (~ROS 2 Jazzy desktop + Nav2 + planner; first build is slow)
./docker/build_docker.sh

# 2. Push to Docker Hub (needs `docker login -u thanhnc19`)
./docker/push_docker.sh

# 3. Run — mounts this repo at /opt/unige_ws/src/quadruped_isaac_sim, host net + GPU
./docker/run_unige_docker.sh
#   then, inside the container:
#     cd /opt/unige_ws && colcon build --symlink-install && source install/setup.bash
#     ros2 topic list          # should see the host Isaac bridge topics on domain 42
```

Override the name/tag with `IMAGE=` / `TAG=`; run a one-shot command with
`CMD="ros2 launch spot_sar_bringup perception.launch.py" ./docker/run_unige_docker.sh`.

## Notes

- The planner lives in a venv at `/opt/sar_planning_venv` (built with
  `--system-site-packages` so `rclpy` stays importable alongside `unified_planning`).
  `source /opt/sar_planning_venv/bin/activate` to use Fast Downward / ENHSP.
- GPU flags (`--gpus all`) are added automatically when `nvidia-smi` is present (only needed
  if you run GPU ROS nodes in the container; Isaac itself is on the host).
