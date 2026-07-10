"""Bring up the Spot Isaac Sim + ROS 2 bridge.

`ros2 launch spot_sar_bringup sim.launch.py` runs the spot_sar_sim standalone app via
scripts/run_isaac.sh (which deactivates conda, sources ROS 2, pins the local asset/cache
dirs, and launches Isaac Sim's bundled Python). Later phases add perception / nav / planner
nodes to this file so one launch composes the whole system.

Args:
  gui:=false       hide the Isaac Sim window (default: GUI shown; headless for CI/low-VRAM runs)
  domain_id:=42    ROS_DOMAIN_ID for the sim (match it in your ros2 CLI shells)
  repo:=<path>     path to this repo (default: the workspace src checkout)
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration

DEFAULT_REPO = os.path.expanduser("~/unige_ws/src/quadruped_isaac_sim")


def _launch_sim(context, *args, **kwargs):
    repo = LaunchConfiguration("repo").perform(context)
    gui = LaunchConfiguration("gui").perform(context).lower() in ("1", "true", "yes")
    domain_id = LaunchConfiguration("domain_id").perform(context)

    run_isaac = os.path.join(repo, "scripts", "run_isaac.sh")
    app = os.path.join(repo, "spot_sar_sim", "spot_sar_sim", "standalone", "spot_cmd_vel_app.py")
    cmd = [run_isaac, app]
    if not gui:
        cmd.append("--headless")

    # NOTE: do NOT pin ISAAC_ASSETS here — run_isaac.sh picks the local pack if present and
    # falls back to NVIDIA's cloud assets otherwise (hardcoding the local dir here would crash
    # machines without a downloaded asset pack).
    return [
        ExecuteProcess(
            cmd=cmd,
            output="screen",
            additional_env={
                "ROS_DOMAIN_ID": domain_id,
            },
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("domain_id", default_value="42"),
            DeclareLaunchArgument("repo", default_value=DEFAULT_REPO),
            OpaqueFunction(function=_launch_sim),
        ]
    )
