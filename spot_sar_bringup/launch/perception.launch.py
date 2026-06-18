"""Phase 2 — bring up Spot with the RGB-D camera + the victim detector.

`ros2 launch spot_sar_bringup perception.launch.py` runs:
  * spot_perception_app.py via scripts/run_isaac.sh — Spot + /cmd_vel + the locomotion
    bridge (/clock /joint_states /odom /tf) PLUS the RGB-D camera
    (/camera/rgb/image_raw, /camera/depth/image_raw, /camera/rgb/camera_info);
  * two static TFs that connect the camera into the tree:
        base_link -> camera_link            (the rigid mount: 0.35 m fwd, 0.10 m up)
        camera_link -> camera_optical_frame (REP-103 optical rotation; image + K live here)
  * detector_node — HSV+depth victim detection publishing /victims (use_sim_time:=true).

Args:
  gui:=true            show the Isaac Sim window (default headless)
  domain_id:=42        ROS_DOMAIN_ID for the whole system
  repo:=<path>         path to this repo (default: the workspace src checkout)
  run_detector:=false  bring up the sim + TFs only, skip the detector

NOTE: the FIRST launch triggers a long RTX shader compile (cold Blackwell cache) before any
camera frame appears — this is expected, not a hang.
"""
import os
import math

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

DEFAULT_REPO = os.path.expanduser("~/unige_ws/src/quadruped_isaac_sim")
HALF_PI = math.pi / 2.0


def _launch(context, *args, **kwargs):
    repo = LaunchConfiguration("repo").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")
    domain_id = LaunchConfiguration("domain_id").perform(context)
    run_detector = LaunchConfiguration("run_detector").perform(context).lower() in ("1", "true", "yes")

    run_isaac = os.path.join(repo, "scripts", "run_isaac.sh")
    app = os.path.join(repo, "spot_sar_sim", "spot_sar_sim", "standalone", "spot_perception_app.py")
    cmd = [run_isaac, app]
    if gui:
        cmd.append("--gui")

    actions = [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            additional_env={
                "ROS_DOMAIN_ID": domain_id,
                "ISAAC_ASSETS": os.path.expanduser("~/isaacsim_assets"),
            },
        ),
        # base_link -> camera_link : rigid mount (matches the prim translate in the app)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_camera_link",
            arguments=["--x", "0.35", "--y", "0.0", "--z", "0.10",
                       "--frame-id", "base_link", "--child-frame-id", "camera_link"],
        ),
        # camera_link -> camera_optical_frame : REP-103 (x-right, y-down, z-fwd)
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_link_to_optical",
            arguments=["--roll", f"{-HALF_PI}", "--pitch", "0.0", "--yaw", f"{-HALF_PI}",
                       "--frame-id", "camera_link", "--child-frame-id", "camera_optical_frame"],
        ),
    ]

    if run_detector:
        actions.append(
            Node(
                package="spot_sar_perception",
                executable="detector_node",
                name="victim_detector",
                output="screen",
                parameters=[{"use_sim_time": True}],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="false"),
            DeclareLaunchArgument("domain_id", default_value="42"),
            DeclareLaunchArgument("repo", default_value=DEFAULT_REPO),
            DeclareLaunchArgument("run_detector", default_value="true"),
            OpaqueFunction(function=_launch),
        ]
    )
