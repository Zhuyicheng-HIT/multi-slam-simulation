from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = str(
        Path(get_package_share_directory("uf_safety_supervisor"))
        / "config"
        / "safety_slice.yaml"
    )
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    raw_lidar_topic = LaunchConfiguration("raw_lidar_topic")
    return LaunchDescription(
        [
            DeclareLaunchArgument("config", default_value=default_config),
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("raw_lidar_topic", default_value="/livox/lidar"),
            Node(
                package="uf_relocalization",
                executable="active_relocalization_controller",
                parameters=[config, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
            Node(
                package="uf_safety_supervisor",
                executable="raw_obstacle_safety_monitor",
                parameters=[
                    config,
                    {
                        "use_sim_time": use_sim_time,
                        "raw_lidar_topic": raw_lidar_topic,
                    },
                ],
                output="screen",
            ),
            Node(
                package="uf_safety_supervisor",
                executable="local_avoidance_planner",
                parameters=[
                    config,
                    {
                        "use_sim_time": use_sim_time,
                        "raw_lidar_topic": raw_lidar_topic,
                    },
                ],
                output="screen",
            ),
            Node(
                package="uf_safety_supervisor",
                executable="flight_command_arbiter",
                parameters=[config, {"use_sim_time": use_sim_time}],
                output="screen",
            ),
        ]
    )
