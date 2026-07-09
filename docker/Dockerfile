# =============================================================================
# thanhnc19/unige_legged  —  ROS 2 environment image for the Spot SAR project
#
# Like the HESFM image, this ships ONLY the runtime environment — the ROS 2 side
# of the Spot SAR stack (Jazzy + Nav2 + slam_toolbox + perception deps + the PDDL
# planner). Your source is NOT baked in: mount the colcon workspace at run time
# and build it inside the container with `colcon build` (see run_unige_docker.sh).
#
# WHAT THIS IMAGE IS NOT: it does not contain NVIDIA Isaac Sim. Isaac Sim 6.0 is a
# host workstation install (GPU + RTX driver bound to the machine); the simulator
# runs on the host and talks to containerised ROS 2 nodes over DDS (same
# ROS_DOMAIN_ID, host networking). This image is for the perception / nav /
# planning / executive nodes — everything downstream of the Isaac ROS 2 bridge.
#
# Build:   ./docker/build_docker.sh
# Push:    ./docker/push_docker.sh
# Run:     ./docker/run_unige_docker.sh
# =============================================================================
FROM osrf/ros:jazzy-desktop

LABEL org.opencontainers.image.title="unige_legged" \
      org.opencontainers.image.description="Spot SAR ROS 2 environment (Jazzy + Nav2 + slam_toolbox + RGB-D perception + unified-planning/Fast Downward/ENHSP)" \
      org.opencontainers.image.source="https://github.com/thanhnguyencanh/quadruped_isaac_sim" \
      maintainer="thanhnguyencanh <canhthanhlt@gmail.com>"

ENV DEBIAN_FRONTEND=noninteractive \
    ROS_DISTRO=jazzy \
    LANG=C.UTF-8 \
    LC_ALL=C.UTF-8

SHELL ["/bin/bash", "-c"]

# -----------------------------------------------------------------------------
# 1. Build tooling + colcon / rosdep (ros-dev-tools brings colcon, vcstool, rosdep)
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        build-essential cmake git wget curl ca-certificates \
        ros-dev-tools python3-colcon-common-extensions python3-rosdep python3-vcstool && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 2. ROS 2 runtime dependencies for the Spot SAR packages
#    - Nav2 + slam_toolbox             : Phases 3-4 mapping / navigation
#    - depthimage/pointcloud->laserscan: synthesize /scan for slam_toolbox / Nav2
#    - cv_bridge + vision_msgs + tf2   : Phase 2 perception (detector_node)
#    - image_transport / rqt_image_view: inspecting the camera streams
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-navigation2 ros-jazzy-nav2-bringup \
        ros-jazzy-slam-toolbox \
        ros-jazzy-depthimage-to-laserscan ros-jazzy-pointcloud-to-laserscan \
        ros-jazzy-cv-bridge ros-jazzy-vision-msgs ros-jazzy-image-transport \
        ros-jazzy-tf2-ros ros-jazzy-tf2-geometry-msgs ros-jazzy-tf2-sensor-msgs \
        ros-jazzy-rqt-image-view ros-jazzy-rviz2 \
        python3-opencv python3-numpy && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 3. PDDL planning environment — unified-planning + Fast Downward + ENHSP.
#    Mirrors the host ~/sar_planning_venv: a venv with --system-site-packages so
#    rclpy (from /opt/ros) stays importable alongside the planner. Fast Downward
#    compiles from source (cmake/g++); ENHSP is JVM-based (needs a JRE).
# -----------------------------------------------------------------------------
ENV SAR_PLANNING_VENV=/opt/sar_planning_venv
RUN apt-get update && apt-get install -y --no-install-recommends \
        python3-venv python3-dev default-jre && \
    rm -rf /var/lib/apt/lists/* && \
    python3 -m venv --system-site-packages "${SAR_PLANNING_VENV}" && \
    source "${SAR_PLANNING_VENV}/bin/activate" && \
    pip install --no-cache-dir --upgrade pip && \
    pip install --no-cache-dir unified-planning up-fast-downward up-enhsp && \
    python -c "from unified_planning.shortcuts import get_environment; print('UP engines:', list(get_environment().factory.engines)[:5])" && \
    deactivate

# -----------------------------------------------------------------------------
# 3b. Editors / small CLI conveniences. Late, cheap layer so editing this list
#     never invalidates the expensive Nav2 / planner cache above.
# -----------------------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
        nano vim less htop tmux iputils-ping && \
    rm -rf /var/lib/apt/lists/*

# -----------------------------------------------------------------------------
# 3c. OPTIONAL learned detector — YOLOv8 venv at /root/yolo_venv (= ~/yolo_venv,
#     exactly where perception.launch.py looks; the container runs as root).
#     OFF by default — CPU torch + ultralytics add ~2-3 GB. Enable at build time:
#         WITH_YOLO=1 ./docker/build_docker.sh
#         docker build --build-arg WITH_YOLO=1 -f docker/Dockerfile docker/
#     Mirrors the host ~/yolo_venv: numpy PINNED to 1.26.4 so cv_bridge's ABI
#     holds against torch/opencv (else numpy 2.x → "multiarray failed to import").
# -----------------------------------------------------------------------------
ARG WITH_YOLO=0
ENV YOLO_VENV=/root/yolo_venv
RUN if [ "$WITH_YOLO" = "1" ]; then \
        echo ">> WITH_YOLO=1 — building ${YOLO_VENV} (CPU torch + ultralytics + YOLOv8n weights)" && \
        python3 -m venv --system-site-packages "${YOLO_VENV}" && \
        source "${YOLO_VENV}/bin/activate" && \
        pip install --no-cache-dir --upgrade pip && \
        pip install --no-cache-dir torch torchvision --index-url https://download.pytorch.org/whl/cpu && \
        pip install --no-cache-dir ultralytics && \
        pip install --no-cache-dir "numpy==1.26.4" "opencv-python==4.9.0.80" && \
        ( cd "${YOLO_VENV}" && python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')" ) && \
        python -c "import torch, ultralytics, numpy; print('YOLO venv OK: torch', torch.__version__, '| numpy', numpy.__version__)" && \
        deactivate ; \
    else \
        echo ">> WITH_YOLO=0 — skipping YOLO venv; run perception with detector:=hsv inside the container" ; \
    fi

# -----------------------------------------------------------------------------
# 4. Workspace mount point + shell setup. The colcon workspace is mounted here at
#    run time; sourcing install/setup.bash is a no-op until you build it once.
# -----------------------------------------------------------------------------
ENV UNIGE_WS=/opt/unige_ws
RUN mkdir -p ${UNIGE_WS}/src && \
    printf '%s\n' \
        'source /opt/ros/jazzy/setup.bash' \
        "[ -f ${UNIGE_WS}/install/setup.bash ] && source ${UNIGE_WS}/install/setup.bash" \
        'export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-42}' \
        >> /root/.bashrc

WORKDIR ${UNIGE_WS}

# Fail the build early if the planner stack did not resolve. ROS must be sourced FIRST so
# rclpy is on PYTHONPATH (it lives in /opt/ros, not system site-packages); the venv was made
# with --system-site-packages so it inherits that PYTHONPATH. This mirrors runtime usage:
# `source /opt/ros/jazzy/setup.bash && source /opt/sar_planning_venv/bin/activate`.
RUN source /opt/ros/jazzy/setup.bash && \
    source "${SAR_PLANNING_VENV}/bin/activate" && \
    python -c "import unified_planning, rclpy; print('planning venv OK: unified_planning + rclpy import')" && \
    deactivate

CMD ["bash"]
