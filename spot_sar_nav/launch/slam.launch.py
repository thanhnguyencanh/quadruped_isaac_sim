"""Phase 3 — SLAM from Spot's RGB-D depth camera.

Two nodes:
  * depthimage_to_laserscan : /camera/depth/image_raw (+ camera_info) -> /scan, published in
    the horizontal `camera_link` frame (x-fwd, z-up). depthimage_to_laserscan relabels the
    scan to `output_frame` and assumes that frame is horizontal, so camera_link (not the
    z-forward optical frame) is the correct choice — otherwise the scan plane is vertical.
  * slam_toolbox (async)     : /scan + odom->base_link TF -> /map and the map->odom transform.

Assumes the perception app is already publishing the camera + TF (run perception.launch.py,
or use spot_sar_bringup/mapping.launch.py which composes both).

  ros2 launch spot_sar_nav slam.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = PathJoinSubstitution([FindPackageShare("spot_sar_nav"), "config", "slam_toolbox.yaml"])

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            Node(
                package="depthimage_to_laserscan",
                executable="depthimage_to_laserscan_node",
                name="depth_to_scan",
                output="screen",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "output_frame": "camera_link",   # horizontal frame, NOT camera_optical_frame
                    "range_min": 0.3,
                    "range_max": 8.0,
                    "scan_height": 100,              # rows of the depth image to collapse
                    "scan_time": 0.033,
                }],
                remappings=[
                    ("depth", "/camera/depth/image_raw"),
                    ("depth_camera_info", "/camera/rgb/camera_info"),
                    ("scan", "/scan"),
                ],
            ),
            Node(
                package="slam_toolbox",
                executable="async_slam_toolbox_node",
                name="slam_toolbox",
                output="screen",
                parameters=[slam_params, {"use_sim_time": use_sim_time}],
            ),
        ]
    )
