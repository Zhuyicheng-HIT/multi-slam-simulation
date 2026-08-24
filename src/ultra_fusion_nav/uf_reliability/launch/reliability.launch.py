from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory("uf_reliability"), "config", "reliability.yaml")
    scheduler_config = os.path.join(
        get_package_share_directory("uf_reliability"),
        "config",
        "scheduler_config.yaml",
    )
    arbiter_config = os.path.join(
        get_package_share_directory("uf_reliability"),
        "config",
        "relocalization_request_arbiter.yaml",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        Node(
            package="uf_reliability",
            executable="relocalization_request_arbiter",
            name="relocalization_request_arbiter",
            parameters=[arbiter_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="uf_reliability",
            executable="reliability_monitor",
            name="reliability_monitor",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="uf_reliability",
            executable="reliability_scheduler",
            name="reliability_scheduler",
            parameters=[scheduler_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
    ])
