import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    config = os.path.join(get_package_share_directory("uf_shared_mapping"),
                          "config", "shared_mapping.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("lidar_enabled", default_value="true"),
        DeclareLaunchArgument("lidar_topic", default_value="/cloud_registered"),
        DeclareLaunchArgument("rgbd_enabled", default_value="true"),
        DeclareLaunchArgument("rgbd_minimum_depth_m", default_value="0.30"),
        DeclareLaunchArgument("rgbd_maximum_depth_m", default_value="6.0"),
        DeclareLaunchArgument("output_directory", default_value="shared_map_output"),
        DeclareLaunchArgument(
            "performance_profiling_enabled", default_value="false"
        ),
        Node(package="uf_shared_mapping", executable="shared_mapping",
             parameters=[config, {"enabled": True,
                                  "use_sim_time": LaunchConfiguration("use_sim_time"),
                                  "lidar_enabled": LaunchConfiguration("lidar_enabled"),
                                  "lidar_topic": LaunchConfiguration("lidar_topic"),
                                  "rgbd_enabled": LaunchConfiguration("rgbd_enabled"),
                                  "minimum_depth_m": ParameterValue(
                                      LaunchConfiguration("rgbd_minimum_depth_m"),
                                      value_type=float),
                                  "maximum_depth_m": ParameterValue(
                                      LaunchConfiguration("rgbd_maximum_depth_m"),
                                      value_type=float),
                                  "performance_profiling_enabled": LaunchConfiguration(
                                      "performance_profiling_enabled"),
                                  "output_directory": LaunchConfiguration("output_directory")}],
             condition=IfCondition(LaunchConfiguration("enabled")), output="screen"),
    ])
