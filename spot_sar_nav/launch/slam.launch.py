"""Phase 3 — SLAM from Spot's RGB-D depth camera.

  * depthimage_to_laserscan : /camera/depth/image_raw (+ camera_info) -> /scan in the horizontal
    `camera_link` frame (depthimage_to_laserscan relabels the scan to output_frame and assumes it
    is horizontal — so camera_link, NOT the z-forward optical frame).
  * slam_toolbox (async)     : /scan + odom->base_link TF -> /map and map->odom.

IMPORTANT: in Jazzy, async_slam_toolbox_node is a LIFECYCLE node — it must be CONFIGURED and
ACTIVATED or it never subscribes to /scan / publishes /map (it sits in the unconfigured state
with only /clock + /parameter_events). So we include slam_toolbox's own online_async_launch.py
(which emits the configure+activate transitions) with our params, rather than launching the node
bare.

Assumes the perception app is publishing the camera + TF (run perception.launch.py / mapping.launch.py).
  ros2 launch spot_sar_nav slam.launch.py
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    slam_params = PathJoinSubstitution([FindPackageShare("spot_sar_nav"), "config", "slam_toolbox.yaml"])

    depth_to_scan = Node(
        package="depthimage_to_laserscan",
        executable="depthimage_to_laserscan_node",
        name="depth_to_scan",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "output_frame": "camera_link",   # horizontal frame, NOT camera_optical_frame
            "range_min": 0.3,
            "range_max": 8.0,
            "scan_height": 100,
            "scan_time": 0.033,
        }],
        remappings=[
            ("depth", "/camera/depth/image_raw"),
            ("depth_camera_info", "/camera/rgb/camera_info"),
            ("scan", "/scan"),
        ],
    )

    # slam_toolbox's launch handles the lifecycle configure+activate (autostart=true).
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("slam_toolbox"), "launch", "online_async_launch.py"])
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "slam_params_file": slam_params,
            "autostart": "true",
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            depth_to_scan,
            slam,
        ]
    )
