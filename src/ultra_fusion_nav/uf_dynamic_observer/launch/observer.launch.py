from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("uf_dynamic_observer"))
    default_config = package_share / "config" / "observer.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(default_config)),
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument("input_mode", default_value="livox_custom"),
            Node(
                package="uf_dynamic_observer",
                executable="dynamic_observer_node",
                name="dynamic_static_map_observer",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "enabled": LaunchConfiguration("enabled"),
                        "input_mode": LaunchConfiguration("input_mode"),
                    },
                ],
            ),
        ]
    )
