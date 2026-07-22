"""Full SAR autonomy infrastructure (Phases 2-4) in one launch.

Composes:
  * mapping.launch.py  -> Isaac + Spot + RGB-D camera + victim detector + depth->/scan + slam_toolbox
  * nav2.launch.py     -> Nav2 navigation servers (controller/planner/behaviors/bt)

The "brain" is started SEPARATELY because it needs the planning venv (unified_planning):
  source /opt/ros/jazzy/setup.bash && source ~/unige_ws/install/setup.bash
  source ~/sar_planning_venv/bin/activate
  ros2 run spot_sar_planning world_model_node --ros-args -p use_sim_time:=true -p fixed_frame:=map
  ros2 run spot_sar_executive task_executive   --ros-args -p use_sim_time:=true
For pure coverage instead of the SAR mission:
  ros2 run spot_sar_nav frontier_explorer --ros-args -p use_sim_time:=true

  ros2 launch spot_sar_bringup sar_system.launch.py

WARNING: this is the FULL heavy stack (Isaac+RTX render + slam_toolbox + Nav2). It needs real
memory headroom — run it on a freshly booted machine (clear swap) or it will OOM-kill nodes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    floor = LaunchConfiguration("floor")        # floor:=true -> multi-room + openable doors
    building = LaunchConfiguration("building")  # building:=true -> two-floor building (stairs)
    humans = LaunchConfiguration("humans")      # humans:=false -> orange box victims
    detector = LaunchConfiguration("detector")  # yolo | hsv
    gui = LaunchConfiguration("gui")            # gui:=false -> headless Isaac (lighter)
    mapping = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("spot_sar_bringup"), "launch", "mapping.launch.py"])
        ),
        launch_arguments={"floor": floor, "building": building,
                          "humans": humans, "detector": detector, "gui": gui}.items(),
    )
    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("spot_sar_nav"), "launch", "nav2.launch.py"])
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("floor", default_value="false"),
        DeclareLaunchArgument("building", default_value="false"),
        DeclareLaunchArgument("humans", default_value="true"),
        DeclareLaunchArgument("detector", default_value="yolo"),
        DeclareLaunchArgument("gui", default_value="true"),
        mapping, nav2,
    ])
