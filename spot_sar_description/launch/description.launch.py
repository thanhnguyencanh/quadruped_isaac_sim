"""Publish Spot's static sensor frames via robot_state_publisher.

Loads spot_sensors.urdf and publishes base_link -> camera_link -> camera_optical_frame on
/tf_static. Use this INSTEAD of the inline static_transform_publisher nodes in
perception.launch.py for a single canonical frame source.

  ros2 launch spot_sar_description description.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    urdf = PathJoinSubstitution([FindPackageShare("spot_sar_description"), "urdf", "spot_sensors.urdf"])
    robot_description = ParameterValue(Command(["cat ", urdf]), value_type=str)

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                name="robot_state_publisher",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time, "robot_description": robot_description}],
            ),
        ]
    )
