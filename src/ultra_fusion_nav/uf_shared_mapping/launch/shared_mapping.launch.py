import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    config = os.path.join(get_package_share_directory("uf_shared_mapping"),
                          "config", "shared_mapping.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("lidar_enabled", default_value="true"),
        DeclareLaunchArgument("rgbd_enabled", default_value="true"),
        DeclareLaunchArgument("output_directory", default_value="shared_map_output"),
        Node(package="uf_shared_mapping", executable="shared_mapping",
             parameters=[config, {"enabled": True,
                                  "use_sim_time": LaunchConfiguration("use_sim_time"),
                                  "lidar_enabled": LaunchConfiguration("lidar_enabled"),
                                  "rgbd_enabled": LaunchConfiguration("rgbd_enabled"),
                                  "output_directory": LaunchConfiguration("output_directory")}],
             condition=IfCondition(LaunchConfiguration("enabled")), output="screen"),
    ])
