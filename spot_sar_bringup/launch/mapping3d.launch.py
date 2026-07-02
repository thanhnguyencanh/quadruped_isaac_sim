"""3D mapping add-ons — run ALONGSIDE a live sim (perception.launch.py / building_mission.launch.py).

Turns the RGB-D camera into a 3D map, two complementary ways:
  * a camera POINT CLOUD          : depth_image_proc/point_cloud_xyz_node
        /camera/depth/image_raw (+ camera_info) -> /camera/points (sensor_msgs/PointCloud2)
  * a 3D VOXEL (OctoMap) map       : octomap_server — /camera/points -> an octree, published as
        /octomap_full + /octomap_binary + occupied-voxel markers (/occupied_cells_vis_array) +
        /octomap_point_cloud_centers + a projected 2D /projected_map. Two stacked floors show up as
        voxels at z~0 and z~3 (the teleport keeps odom x,y continuous, so the octree stays coherent).
  * (optional) 3D ELEVATION map    : leggedrobotics elevation_mapping_cupy for the stairs
        (elevation:=true) -> /elevation_mapping/elevation_map (grid_map_msgs/GridMap). GPU/CuPy —
        heavy on 8 GB VRAM next to Isaac; see README + elevation.launch.py.

  ros2 launch spot_sar_bringup mapping3d.launch.py                        # point cloud + octomap
  ros2 launch spot_sar_bringup mapping3d.launch.py fixed_frame:=map       # (with SLAM up)
  ros2 launch spot_sar_bringup mapping3d.launch.py elevation:=true        # + elevation_mapping_cupy

REQUIRES (apt): ros-jazzy-octomap-server ros-jazzy-octomap-rviz-plugins ros-jazzy-octomap-msgs
  (depth_image_proc is already installed). For elevation:=true see the elevation_mapping_cupy notes.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    fixed_frame = LaunchConfiguration("fixed_frame")   # odom (always up) | map (needs SLAM)
    resolution = LaunchConfiguration("resolution")     # octree voxel size (m)
    elevation = LaunchConfiguration("elevation")

    # depth image -> organized XYZ point cloud (32FC1 metres depth from Isaac)
    point_cloud = Node(
        package="depth_image_proc",
        executable="point_cloud_xyz_node",
        name="point_cloud_xyz",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
        remappings=[
            ("image_rect", "/camera/depth/image_raw"),
            ("camera_info", "/camera/rgb/camera_info"),
            ("points", "/camera/points"),
        ],
    )

    # OctoMap 3D voxel map from the cloud (frame_id = the persistent fixed frame)
    octomap = Node(
        package="octomap_server",
        executable="octomap_server_node",
        name="octomap_server",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "frame_id": fixed_frame,          # accumulate the octree in this frame
            "base_frame_id": "base_link",
            "resolution": resolution,
            "sensor_model.max_range": 6.0,    # matches the RGB-D useful range
            "sensor_model.hit": 0.7,
            "sensor_model.miss": 0.4,
            "pointcloud_min_z": -0.5,         # keep the full two-floor height band (0..~5 m)
            "pointcloud_max_z": 6.0,
            "occupancy_min_z": -0.5,
            "occupancy_max_z": 6.0,
            "filter_ground": False,           # keep the floor slabs (they separate the two storeys)
            "latch": False,
        }],
        remappings=[("cloud_in", "/camera/points")],
    )

    # optional leggedrobotics elevation_mapping_cupy (GPU) for the stairs
    elevation_group = GroupAction(
        condition=IfCondition(elevation),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([FindPackageShare("spot_sar_bringup"), "launch", "elevation.launch.py"])
            ),
            launch_arguments={"use_sim_time": use_sim_time}.items(),
        )],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("fixed_frame", default_value="odom",
                              description="octomap accumulation frame: odom (always up) | map (needs SLAM)"),
        DeclareLaunchArgument("resolution", default_value="0.10"),
        DeclareLaunchArgument("elevation", default_value="false",
                              description="also run elevation_mapping_cupy (GPU; heavy on 8 GB VRAM)"),
        point_cloud,
        octomap,
        elevation_group,
    ])
