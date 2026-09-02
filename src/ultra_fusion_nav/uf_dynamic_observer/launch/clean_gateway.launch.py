from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_share = Path(get_package_share_directory("uf_dynamic_observer"))
    default_config = package_share / "config" / "clean_gateway.yaml"
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=str(default_config)),
            DeclareLaunchArgument("enabled", default_value="false"),
            DeclareLaunchArgument(
                "raw_topic", default_value="/livox/lidar"
            ),
            DeclareLaunchArgument(
                "clean_topic", default_value="/dynamic_observer/clean/livox"
            ),
            DeclareLaunchArgument(
                "previous_state_topic",
                default_value="/clean_fast_lio/previous_state",
            ),
            Node(
                package="uf_dynamic_observer",
                executable="clean_scan_gateway_node",
                name="clean_scan_gateway",
                output="screen",
                parameters=[
                    LaunchConfiguration("config"),
                    {
                        "enabled": LaunchConfiguration("enabled"),
                        "raw_topic": LaunchConfiguration("raw_topic"),
                        "clean_topic": LaunchConfiguration("clean_topic"),
                        "previous_state_topic": LaunchConfiguration(
                            "previous_state_topic"
                        ),
                    },
                ],
            ),
        ]
    )
