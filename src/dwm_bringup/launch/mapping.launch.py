"""SLAM Toolbox mapping using the existing lidar and odometry bridge."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def launch_mapping(context):
    if LaunchConfiguration("start_robot").perform(context).lower() != "false":
        raise RuntimeError(
            "Start robot.launch.py separately; start_robot must be false"
        )
    source = (
        Path(get_package_share_directory("slam_toolbox"))
        / "launch/online_async_launch.py"
    )
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(source)),
            launch_arguments={
                "use_sim_time": "false",
                "autostart": "true",
                "slam_params_file": LaunchConfiguration("params_file"),
            }.items(),
        )
    ]


def generate_launch_description():
    share = Path(get_package_share_directory("dwm_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("start_robot", default_value="false"),
            DeclareLaunchArgument(
                "params_file", default_value=str(share / "config/slam.yaml")
            ),
            OpaqueFunction(function=launch_mapping),
        ]
    )
