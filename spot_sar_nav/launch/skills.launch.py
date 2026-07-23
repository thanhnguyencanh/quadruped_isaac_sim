"""Phase 4 — SAR skill server.

Hosts the /skill action (go_to_location | explore | observe | report) that the Task Executive
dispatches. Run alongside Nav2 (nav2.launch.py) + mapping.

  ros2 launch spot_sar_nav skills.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    planner_frame = LaunchConfiguration("planner_frame")
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("planner_frame", default_value="odom"),
            Node(
                package="spot_sar_nav",
                executable="skill_server",
                name="skill_server",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "planner_frame": planner_frame}],
            ),
        ]
    )
