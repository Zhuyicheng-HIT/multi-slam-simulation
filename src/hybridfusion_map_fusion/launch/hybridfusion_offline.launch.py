from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = Path(get_package_share_directory("hybridfusion_map_fusion"))
    enabled = LaunchConfiguration("enabled")
    return LaunchDescription([
        DeclareLaunchArgument(
            "enabled", default_value="false",
            description="Explicit opt-in; offline fusion is disabled by default."),
        DeclareLaunchArgument("dataset", default_value=""),
        DeclareLaunchArgument(
            "config", default_value=str(share / "config" / "hybridfusion.yaml")),
        DeclareLaunchArgument("method", default_value="hybrid"),
        DeclareLaunchArgument("output", default_value="~/.ros/hybridfusion_result"),
        ExecuteProcess(
            condition=IfCondition(enabled),
            cmd=[
                "ros2", "run", "hybridfusion_map_fusion", "hybridfusion_offline",
                "--dataset", LaunchConfiguration("dataset"),
                "--config", LaunchConfiguration("config"),
                "--method", LaunchConfiguration("method"),
                "--output", LaunchConfiguration("output"),
            ],
            output="screen",
        ),
    ])
