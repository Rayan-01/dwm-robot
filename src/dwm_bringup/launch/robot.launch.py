"""Start the dashboard, hardware owner, sensor bridge and optional lidar driver."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_server(context):
    enabled = LaunchConfiguration("hardware").perform(context).lower()
    if enabled not in ("true", "false"):
        raise RuntimeError("hardware must be true or false")
    command = [
        LaunchConfiguration("robot_python").perform(context),
        "-u",
        "-m",
        "dwm_robot.robot",
        "--hardware" if enabled == "true" else "--dry-run",
        "--camera",
        "oak",
        "--serial",
        LaunchConfiguration("serial_port").perform(context),
        "--device",
        LaunchConfiguration("device").perform(context),
    ]
    return [ExecuteProcess(cmd=command, output="screen")]


def generate_launch_description():
    share = Path(get_package_share_directory("dwm_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("hardware", default_value="false"),
            DeclareLaunchArgument("robot_python", default_value="/usr/bin/python3"),
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyACM0"),
            DeclareLaunchArgument("lidar_port", default_value="/dev/ttyUSB0"),
            DeclareLaunchArgument("device", default_value="0"),
            DeclareLaunchArgument("start_lidar", default_value="true"),
            DeclareLaunchArgument("ros_domain_id", default_value="42"),
            SetEnvironmentVariable(
                "ROS_DOMAIN_ID", LaunchConfiguration("ros_domain_id")
            ),
            OpaqueFunction(function=launch_server),
            Node(
                package="dwm_navigation",
                executable="navigation_bridge",
                output="screen",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(share / "launch/lidar.launch.py")),
                condition=IfCondition(LaunchConfiguration("start_lidar")),
                launch_arguments={
                    "serial_port": LaunchConfiguration("lidar_port")
                }.items(),
            ),
        ]
    )
