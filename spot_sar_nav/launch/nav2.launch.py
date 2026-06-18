"""Phase 4 — minimal Nav2 navigation servers for Spot SAR.

Brings up ONLY the nodes we configure, managed by our own lifecycle_manager:
  controller_server, planner_server, behavior_server, bt_navigator.

WHY NOT nav2_bringup/navigation_launch.py: that launch also manages collision_monitor and
velocity_smoother and remaps cmd_vel through a chain (controller -> cmd_vel_nav ->
velocity_smoother -> cmd_vel_smoothed -> collision_monitor -> cmd_vel). If collision_monitor
isn't configured it fails to configure and the lifecycle_manager aborts the WHOLE bringup
(no /cmd_vel). It also spawns more processes — heavier on an 8 GB-VRAM box. Here the
controller_server publishes a plain Twist straight to /cmd_vel (enable_stamped_cmd_vel:false in
nav2_params.yaml), which Spot's locomotion policy subscribes to directly.

The map comes from slam_toolbox (run mapping.launch.py alongside), obstacles from /scan.
  ros2 launch spot_sar_nav nav2.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

LIFECYCLE_NODES = ["controller_server", "planner_server", "behavior_server", "bt_navigator"]


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    params = PathJoinSubstitution([FindPackageShare("spot_sar_nav"), "config", "nav2_params.yaml"])

    def srv(pkg, exe, name=None):
        return Node(package=pkg, executable=exe, name=name or exe, output="screen",
                    parameters=[params, {"use_sim_time": use_sim_time}])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            srv("nav2_controller", "controller_server"),
            srv("nav2_planner", "planner_server"),
            srv("nav2_behaviors", "behavior_server"),
            srv("nav2_bt_navigator", "bt_navigator"),
            Node(
                package="nav2_lifecycle_manager",
                executable="lifecycle_manager",
                name="lifecycle_manager_navigation",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "autostart": True, "node_names": LIFECYCLE_NODES}],
            ),
        ]
    )
