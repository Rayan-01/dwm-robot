"""RPLidar S2 driver; serial_port can be a stable /dev/serial/by-id path."""

from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    source = (
        Path(get_package_share_directory("rplidar_ros")) / "launch/rplidar_s2_launch.py"
    )
    return LaunchDescription(
        [
            DeclareLaunchArgument("serial_port", default_value="/dev/ttyUSB0"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(source)),
                launch_arguments={
                    "serial_port": LaunchConfiguration("serial_port"),
                    "serial_baudrate": "1000000",
                    "frame_id": "laser",
                    "scan_mode": "DenseBoost",
                }.items(),
            ),
        ]
    )
