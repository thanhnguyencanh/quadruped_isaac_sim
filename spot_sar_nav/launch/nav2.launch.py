"""Phase 4 — Nav2 navigation servers for Spot SAR.

Brings up the Nav2 stack (controller, planner, behaviors, bt_navigator, velocity_smoother,
lifecycle manager) via nav2_bringup's navigation_launch.py with spot_sar_nav's nav2_params.yaml.
The map comes from slam_toolbox (run slam.launch.py / mapping.launch.py alongside), obstacles
from /scan, and the controller drives Spot through /cmd_vel (Twist).

  ros2 launch spot_sar_nav nav2.launch.py
Full autonomy stack (heavy): mapping.launch.py + nav2.launch.py + frontier_explorer / executive.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = PathJoinSubstitution([FindPackageShare("spot_sar_nav"), "config", "nav2_params.yaml"])

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("nav2_bringup"), "launch", "navigation_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "params_file": params,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            nav2,
        ]
    )
