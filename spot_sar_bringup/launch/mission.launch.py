"""Phase 6 — the FULL SAR mission in one command.

Composes the whole autonomy stack:
  * sar_system.launch.py : Isaac + Spot + RGB-D camera + victim detector + depth->/scan + SLAM + Nav2
  * skills.launch.py     : the /skill action server (go_to_location | explore | observe | report)
  * world_model_node     : symbol grounding /victims+TF -> /world_model (fixed_frame=odom,
        matching nav + skills: goals go out in odom, so grounded centroids must be odom too)
  * task_executive       : the SENSE->GROUND->PLAN->ACT->MONITOR->REPLAN loop (run via the planning venv)

  ros2 launch spot_sar_bringup mission.launch.py

NOTES:
  * The executive runs under the planning venv (unified_planning); it is launched as an
    ExecuteProcess invoking the venv python on the installed task_executive entry point. The
    launching shell must have ROS 2 + the workspace install sourced (so the venv inherits PYTHONPATH).
  * HEAVY: Isaac RTX + SLAM + Nav2 + detector + brain together are memory-intensive on 8 GB VRAM.
    Run on a freshly booted machine; for a leaner run, start the brain separately after Nav2 is up.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

VENV_PY = os.path.expanduser("~/sar_planning_venv/bin/python")
EXEC_BIN = os.path.expanduser("~/unige_ws/install/spot_sar_executive/lib/spot_sar_executive/task_executive")


def generate_launch_description():
    sar_system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("spot_sar_bringup"), "launch", "sar_system.launch.py"])
        ),
        launch_arguments={"gui": LaunchConfiguration("gui")}.items(),
    )
    skills = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("spot_sar_nav"), "launch", "skills.launch.py"])
        )
    )
    world_model = Node(
        package="spot_sar_planning",
        executable="world_model_node",
        name="world_model",
        output="screen",
        parameters=[{"use_sim_time": True, "fixed_frame": "odom"}],
    )
    # The executive needs the planning venv (unified_planning). Delay it so SLAM/Nav2 come up first.
    executive = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=[VENV_PY, EXEC_BIN, "--ros-args",
                     "-p", "use_sim_time:=true", "-p", "planner_frame:=odom", "-p", "cycle_period:=3.0"],
                output="screen",
            )
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        sar_system, skills, world_model, executive,
    ])
