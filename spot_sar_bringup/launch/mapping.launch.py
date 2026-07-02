"""Phase 3 — full mapping stack in one command.

Composes:
  * spot_sar_bringup/perception.launch.py  (Isaac + Spot + RGB-D camera + victim detector)
  * spot_sar_nav/slam.launch.py            (depth->/scan + slam_toolbox -> /map, map->odom)

  ros2 launch spot_sar_bringup mapping.launch.py
Then drive Spot to build the map:
  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist '{angular: {z: 0.3}}'
  ros2 topic echo /map --once
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # run_detector:=false drops the victim detector to save memory during heavy SLAM runs.
    run_detector = LaunchConfiguration("run_detector")
    floor = LaunchConfiguration("floor")          # floor:=true -> multi-room + doors environment
    building = LaunchConfiguration("building")    # building:=true -> two-floor building (stairs)
    humans = LaunchConfiguration("humans")        # humans:=false -> orange box victims
    detector = LaunchConfiguration("detector")    # yolo | hsv
    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("spot_sar_bringup"), "launch", "perception.launch.py"])
        ),
        launch_arguments={"run_detector": run_detector, "floor": floor, "building": building,
                          "humans": humans, "detector": detector}.items(),
    )
    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare("spot_sar_nav"), "launch", "slam.launch.py"])
        )
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("run_detector", default_value="true"),
            DeclareLaunchArgument("floor", default_value="false"),
            DeclareLaunchArgument("building", default_value="false"),
            DeclareLaunchArgument("humans", default_value="true"),
            DeclareLaunchArgument("detector", default_value="yolo"),
            perception,
            slam,
        ]
    )
