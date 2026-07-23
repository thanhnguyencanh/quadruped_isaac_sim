"""TWO-FLOOR BUILDING mission with a stairwell — the stairs demo in one command.

Same autonomy stack as floor_mission.launch.py, but switched to the two-floor "building" profile:
  * sar_system.launch.py (building:=true) : Isaac + Spot + camera + detector + SLAM + Nav2, building
        the TWO-FLOOR building (x-offset wings sharing a vertically-stacked stair landing) instead of
        the single-storey floor;
  * skills.launch.py                      : the /skill action server (now incl. climb_stairs);
  * building_world_model_node             : room-graph grounding across BOTH floors (floor from the
        latched /floor_state) -> /world_model;
  * task_executive (domain_profile:=building) : plans with domain_building.pddl, so it must dispatch
        open_door (floor-1 doors) and climb_stairs (which pure-z-teleports Spot between floors in
        Isaac) to reach the floor-2 victim.

  ros2 launch spot_sar_bringup building_mission.launch.py

Expected closed loop: PLAN open-door -> reach floor-1 victim -> report -> move to the landing ->
PLAN use-stairs -> climb_stairs publishes /stairs_cmd -> StairsNode teleports Spot up -> /floor_state
flips to f2 -> grounding grounds the robot on floor 2 -> move through the floor-2 rooms -> detect ->
REPORT. See mission.launch.py for the heavy-stack / planning-venv notes.

WHY the stairs are a teleport-assist: Spot's flat-terrain policy cannot climb real steps, so the
level change is performed in-app by StairsNode (a pure-z teleport at the stacked landing, which keeps
odom x,y ~constant so slam_toolbox's planar map->odom is undisturbed). The staircase is rendered for
realism / perception; Spot never drives onto it.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, TimerAction, SetEnvironmentVariable
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
        launch_arguments={"building": "true", "gui": LaunchConfiguration("gui")}.items(),
    )
    skills = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("spot_sar_nav"), "launch", "skills.launch.py"])
        )
    )
    building_world_model = Node(
        package="spot_sar_planning",
        executable="building_world_model_node",
        name="building_world_model",
        output="screen",
        parameters=[{"use_sim_time": True, "fixed_frame": "odom"}],
    )
    # The executive needs the planning venv (unified_planning). Delay it so SLAM/Nav2 come up first.
    executive = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=[VENV_PY, EXEC_BIN, "--ros-args", "-p", "use_sim_time:=true",
                     "-p", "planner_frame:=odom", "-p", "cycle_period:=3.0",
                     "-p", "domain_profile:=building"],
                output="screen",
            )
        ],
    )
    return LaunchDescription([
        # rcutils disables ANSI colors when stdout is a PIPE (always true under ros2
        # launch), so WARN mission-tracking lines printed white. Force colors: WARN's
        # yellow is the whole point of the [STATUS]/world_model/REPORTED highlighting.
        SetEnvironmentVariable("RCUTILS_COLORIZED_OUTPUT", "1"),
        DeclareLaunchArgument("gui", default_value="true"),
        sar_system, skills, building_world_model, executive,
    ])
