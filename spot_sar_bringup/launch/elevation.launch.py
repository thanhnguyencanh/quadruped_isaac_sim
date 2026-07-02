"""leggedrobotics elevation_mapping_cupy for the two-floor building's stairs (3D elevation / height map).

Included by mapping3d.launch.py when `elevation:=true`. Consumes the camera point cloud
(/camera/points from mapping3d) and publishes a robot-centric 2.5D height GridMap
(/elevation_mapping_node/elevation_map, grid_map_msgs/GridMap) whose `elevation` layer renders as a
3D coloured surface in RViz via grid_map_rviz_plugin — the stairs appear as a stepped height field.

REQUIRES the elevation_mapping_cupy stack BUILT in the workspace (a ROS 2 dev branch + CuPy + the apt
grid_map packages). Run `scripts/setup_3d_mapping.sh` first. GPU/CuPy: heavy on 8 GB VRAM next to the
Isaac RTX renderer — if it OOMs, fall back to the OctoMap voxel map (which is GPU-free). See README.

It loads elevation_mapping_cupy's own core_param.yaml + plugin_config.yaml (defaults) and overlays our
setup (config/elevation_spot_building.yaml: frames + the /camera/points input + the output layers)."""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")

    from ament_index_python.packages import get_package_share_directory, PackageNotFoundError
    try:
        emc_share = get_package_share_directory("elevation_mapping_cupy")
    except PackageNotFoundError as e:
        raise RuntimeError(
            "elevation_mapping_cupy is not installed/built. Run scripts/setup_3d_mapping.sh first "
            "(apt grid_map + CuPy + colcon build the ROS 2 branch), then source the workspace. "
            f"({e})")

    core_param = os.path.join(emc_share, "config", "core", "core_param.yaml")
    plugin_cfg = os.path.join(emc_share, "config", "core", "plugin_config.yaml")
    our_share = get_package_share_directory("spot_sar_bringup")
    setup = os.path.join(our_share, "config", "elevation_spot_building.yaml")

    params = [p for p in (core_param, plugin_cfg) if os.path.exists(p)] + [
        setup, {"use_sim_time": use_sim_time}]

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="elevation_mapping_cupy",
            executable="elevation_mapping_node.py",
            name="elevation_mapping_node",
            output="screen",
            parameters=params,
        ),
    ])
