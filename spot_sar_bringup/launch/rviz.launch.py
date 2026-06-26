"""rviz.launch.py — open RViz2 preloaded with the SAR visualization config.

Pairs with a HEADLESS Isaac run: bring the sim up without --gui (lighter on 8 GB VRAM, no
shader-compile freeze) and visualize everything here over ROS 2 — raw camera, the detection
overlay, /scan, /map, Nav2 costmaps/path, victim markers, and TF.

  ros2 launch spot_sar_bringup rviz.launch.py            # default config (config/sar.rviz)
  ros2 launch spot_sar_bringup rviz.launch.py rviz_config:=/path/to/other.rviz

Run on the SAME ROS_DOMAIN_ID as the sim (this project standardizes on 42).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_rviz = os.path.join(
        get_package_share_directory("spot_sar_bringup"), "rviz", "sar.rviz"
    )

    rviz_config = LaunchConfiguration("rviz_config")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("rviz_config", default_value=default_rviz,
                              description="Path to the .rviz config to load"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Use /clock from Isaac Sim"),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
