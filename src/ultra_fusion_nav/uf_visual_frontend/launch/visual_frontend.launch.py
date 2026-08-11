from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os


def generate_launch_description():
    config = os.path.join(get_package_share_directory(
        "uf_visual_frontend"), "config", "visual_frontend.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        Node(
            package="uf_visual_frontend",
            executable="rgbd_feature_frontend",
            name="uf_rgbd_feature_frontend",
            parameters=[config],
            condition=IfCondition(LaunchConfiguration("enabled")),
            output="screen",
        ),
    ])
