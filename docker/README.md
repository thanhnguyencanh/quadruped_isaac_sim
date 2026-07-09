# `unige_legged` — Spot SAR ROS 2 environment image

A Docker image (`thanhnc19/unige_legged`) that ships the **ROS 2 side** of the Spot SAR
stack: ROS 2 **Jazzy** + Nav2 + slam_toolbox + the RGB-D perception (incl. the YOLO detector)
+ OctoMap/grid_map 3D mapping + the PDDL planner (unified-planning + Fast Downward + ENHSP). Modeled on the
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

## Prerequisite — Docker daemon access

`docker` must be usable without `sudo`. If you see
`permission denied ... /var/run/docker.sock`, add yourself to the `docker` group once:

```bash
sudo usermod -aG docker $USER
newgrp docker          # apply to the current shell (or log out/in)
```

(Or prefix each command below with `sudo`.)

## Usage

```bash
# 1. Build. The published image is built WITH_YOLO=1 (OctoMap/grid_map 3D mapping is default);
#    a bare ./docker/build_docker.sh is HSV-only. Add the GPU elevation map with WITH_ELEVATION=1.
WITH_YOLO=1 ./docker/build_docker.sh                    # reproduce the published image
WITH_YOLO=1 WITH_ELEVATION=1 ./docker/build_docker.sh   # + elevation_mapping_cupy (CuPy/GPU)

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
- **YOLO + OctoMap are included** in the published image: the pretrained YOLOv8 detector at
  `~/yolo_venv` (`/root/yolo_venv`, where `perception.launch.py` looks — CPU torch + `yolov8n.pt`,
  numpy pinned 1.26.4) and the OctoMap/grid_map 3D-mapping stack (`mapping3d.launch.py`). A bare local
  `docker build` (ARG default) is HSV-only — pass `WITH_YOLO=1` to match the published image, or run
  perception with `humans:=false detector:=hsv` to skip YOLO.
- **elevation_mapping_cupy is opt-in** (`WITH_ELEVATION=1`): a CuPy/GPU source build, needs the NVIDIA
  GPU at runtime. OctoMap already gives a GPU-free 3D voxel map without it.
