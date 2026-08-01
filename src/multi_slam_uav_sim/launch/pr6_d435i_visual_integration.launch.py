from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include(package, launch_file, arguments=None, condition=None):
    source = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(source)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    sim_share = Path(get_package_share_directory("multi_slam_uav_sim"))
    rtab_config = LaunchConfiguration("rtab_config")
    database_path = LaunchConfiguration("database_path")
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_backend = LaunchConfiguration("start_backend")
    start_rtabmap = LaunchConfiguration("start_rtabmap")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_backend", default_value="true"),
        DeclareLaunchArgument("start_rtabmap", default_value="true"),
        DeclareLaunchArgument(
            "rtab_config",
            default_value=str(
                sim_share / "config" / "d435i_rtabmap_feature_aligned.yaml"),
        ),
        DeclareLaunchArgument(
            "database_path",
            default_value="~/.ros/pr6_d435i_visual_integration.db",
        ),
        Node(
            package="d435i_rgbd_bridge_cpp",
            executable="d435i_rgbd_bridge",
            name="d435i_rgbd_bridge_cpp",
            parameters=[{
                "gz_prefix": "/front/d435i/gz",
                "ros_prefix": "/front/d435i",
                "camera_link_frame": "d435i_link",
                "depth_encoding": "16UC1",
                "sync_queue_depth": 2,
                "qos_depth": 1,
                "qos_reliability": "best_effort",
                "enable_pointcloud": False,
            }],
            output="screen",
        ),
        include(
            "uf_sensor_pipeline",
            "sensor_pipeline.launch.py",
            {
                "enable_vision": "true",
                "use_sim_time": use_sim_time,
                "d435_color_input_topic": "/front/d435i/color/image_raw",
                "d435_depth_input_topic": (
                    "/front/d435i/aligned_depth_to_color/image_raw"),
                "d435_parent_frame": "base_link",
                "d435_child_frame": "d435i_link",
            },
        ),
        include(
            "multi_slam_uav_sim",
            "d435i_rtabmap.launch.py",
            {
                "config_file": rtab_config,
                "database_path": database_path,
                "rgb_topic": "/sensors/rgbd/color",
                "depth_topic": "/sensors/rgbd/depth",
                "camera_info_topic": "/front/d435i/color/camera_info",
            },
            condition=IfCondition(start_rtabmap),
        ),
        Node(
            package="multi_slam_uav_sim",
            executable="d435i_visual_reliability",
            name="d435i_visual_reliability",
            parameters=[
                str(sim_share / "config" / "d435i_visual_reliability.yaml"),
                {
                    "use_sim_time": use_sim_time,
                    "rgb_topic": "/sensors/rgbd/color",
                    "depth_topic": "/sensors/rgbd/depth",
                    "odom_topic": "/rtabmap/odom",
                },
            ],
            output="screen",
        ),
        include(
            "uf_lio_adapter",
            "lio_adapter.launch.py",
            {"use_sim_time": use_sim_time},
            condition=IfCondition(start_backend),
        ),
        include(
            "uf_backend_fusion",
            "online_backend_visual.launch.py",
            {"preserve_lio_anchor": "true"},
            condition=IfCondition(start_backend),
        ),
    ])
