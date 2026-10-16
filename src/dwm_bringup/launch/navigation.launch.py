"""Map localization and Nav2; the running robot server owns the serial port."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_navigation(context):
    if LaunchConfiguration("start_robot").perform(context).lower() != "false":
        raise RuntimeError(
            "Start robot.launch.py separately; start_robot must be false"
        )
    map_path = Path(LaunchConfiguration("map").perform(context)).expanduser().resolve()
    if not map_path.is_file():
        raise RuntimeError(f"Map YAML does not exist: {map_path}")
    parameters = LaunchConfiguration("params_file")
    components = [
        ("nav2_map_server", "map_server"),
        ("nav2_amcl", "amcl"),
        ("nav2_controller", "controller_server"),
        ("nav2_smoother", "smoother_server"),
        ("nav2_planner", "planner_server"),
        ("nav2_behaviors", "behavior_server"),
        ("nav2_velocity_smoother", "velocity_smoother"),
        ("nav2_bt_navigator", "bt_navigator"),
        ("nav2_waypoint_follower", "waypoint_follower"),
        ("nav2_collision_monitor", "collision_monitor"),
    ]
    nodes = []
    for package, name in components:
        overrides = {"use_sim_time": False}
        remappings = []
        if name == "map_server":
            overrides["yaml_filename"] = str(map_path)
        if name in ("controller_server", "behavior_server", "velocity_smoother"):
            overrides["enable_stamped_cmd_vel"] = False
            remappings.append(("cmd_vel", "cmd_vel_nav"))
        if name == "velocity_smoother":
            remappings.append(("cmd_vel_smoothed", "cmd_vel"))
        nodes.append(
            Node(
                package=package,
                executable=name,
                name=name,
                parameters=[parameters, overrides],
                remappings=remappings,
                output="screen",
            )
        )
    nodes.append(
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": False,
                    "autostart": True,
                    "node_names": [name for _, name in components],
                }
            ],
        )
    )
    return nodes


def generate_launch_description():
    share = Path(get_package_share_directory("dwm_bringup"))
    return LaunchDescription(
        [
            DeclareLaunchArgument("map", description="Absolute path to saved map YAML"),
            DeclareLaunchArgument("start_robot", default_value="false"),
            DeclareLaunchArgument(
                "params_file",
                default_value=str(share / "config/navigation_params.yaml"),
            ),
            OpaqueFunction(function=launch_navigation),
        ]
    )
