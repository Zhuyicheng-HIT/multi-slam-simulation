from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory("hybridfusion_map_fusion"))
    enabled = LaunchConfiguration("enabled")
    config = LaunchConfiguration("config")
    output_root = LaunchConfiguration("output_root")
    return LaunchDescription([
        DeclareLaunchArgument(
            "enabled", default_value="false",
            description="Opt-in switch. False creates no subscriptions or exporters."),
        DeclareLaunchArgument(
            "config", default_value=str(share / "config" / "hybridfusion.yaml")),
        DeclareLaunchArgument(
            "output_root", default_value="~/.ros/hybridfusion_export"),
        LogInfo(
            condition=IfCondition(enabled),
            msg="HybridFusion map exporters enabled; outputs are independent files only."),
        Node(
            package="hybridfusion_map_fusion",
            executable="rgbd_map_exporter",
            name="hybridfusion_rgbd_map_exporter",
            condition=IfCondition(enabled),
            parameters=[config, {
                "enabled": True,
                "output_dir": [output_root, "/visual"],
            }],
            output="screen",
        ),
        Node(
            package="hybridfusion_map_fusion",
            executable="lidar_map_exporter",
            name="hybridfusion_lidar_map_exporter",
            condition=IfCondition(enabled),
            parameters=[config, {
                "enabled": True,
                "output_dir": [output_root, "/lidar"],
            }],
            output="screen",
        ),
    ])
