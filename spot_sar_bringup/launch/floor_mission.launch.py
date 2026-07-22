"""Multi-room FLOOR mission with openable doors — the door demo in one command.

Same autonomy stack as mission.launch.py, but switched to the room-graph "doors" profile:
  * sar_system.launch.py (floor:=true) : Isaac + Spot + camera + detector + SLAM + Nav2, building
        the multi-room floor (walls + sliding door slabs) instead of the single room;
  * skills.launch.py                   : the /skill action server (now incl. open_door);
  * floor_world_model_node             : room-graph grounding (rooms + doors) -> /world_model;
  * task_executive (domain_profile:=doors) : plans with domain_doors.pddl, so it must dispatch
        open_door (which physically slides the slab open in Isaac) before move-ing between rooms.

  ros2 launch spot_sar_bringup floor_mission.launch.py

Expected closed loop: PLAN open-door -> skill open_door publishes /door_cmd -> slab slides open,
/door_states -> grounding flips door_open -> next PLAN move through the door -> Nav2 drives into
the room -> detect -> REPORT. See mission.launch.py for the heavy-stack / planning-venv notes.
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
        launch_arguments={"floor": "true", "gui": LaunchConfiguration("gui")}.items(),
    )
    skills = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("spot_sar_nav"), "launch", "skills.launch.py"])
        )
    )
    floor_world_model = Node(
        package="spot_sar_planning",
        executable="floor_world_model_node",
        name="floor_world_model",
        output="screen",
        parameters=[{"use_sim_time": True, "fixed_frame": "map"}],
    )
    # The executive needs the planning venv (unified_planning). Delay it so SLAM/Nav2 come up first.
    executive = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=[VENV_PY, EXEC_BIN, "--ros-args", "-p", "use_sim_time:=true",
                     "-p", "planner_frame:=map", "-p", "cycle_period:=3.0",
                     "-p", "domain_profile:=doors"],
                output="screen",
            )
        ],
    )
    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true"),
        sar_system, skills, floor_world_model, executive,
    ])
