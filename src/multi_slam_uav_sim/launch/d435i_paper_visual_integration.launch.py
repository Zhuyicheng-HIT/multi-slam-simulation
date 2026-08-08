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
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rtabmap = LaunchConfiguration("start_rtabmap")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_rtabmap", default_value="true"),
        DeclareLaunchArgument("database_path", default_value="paper_visual.db"),
        DeclareLaunchArgument("camera_time_offset_s", default_value="0.0"),
        DeclareLaunchArgument(
            "visual_factor_mode", default_value="paper_reprojection"
        ),
        DeclareLaunchArgument("visual_keyframe_profile", default_value="balanced"),
        DeclareLaunchArgument(
            "visual_candidate_quality_enabled", default_value="true"
        ),
        DeclareLaunchArgument("visual_pending_enabled", default_value="true"),
        DeclareLaunchArgument(
            "performance_profiling_enabled", default_value="false"
        ),
        DeclareLaunchArgument("performance_trace_path", default_value=""),
        DeclareLaunchArgument("shared_mapping_enabled", default_value="false"),
        DeclareLaunchArgument("shared_mapping_rgbd_enabled", default_value="true"),
        DeclareLaunchArgument(
            "shared_mapping_output_directory", default_value="shared_map_output"
        ),
        Node(
            package="d435i_rgbd_bridge_cpp",
            executable="d435i_rgbd_bridge",
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
        include("uf_sensor_pipeline", "sensor_pipeline.launch.py", {
            "enable_vision": "true",
            "use_sim_time": use_sim_time,
            "d435_color_input_topic": "/front/d435i/color/image_raw",
            "d435_depth_input_topic": "/front/d435i/aligned_depth_to_color/image_raw",
            "d435_parent_frame": "base_link",
            "d435_child_frame": "d435i_link",
        }),
        include("multi_slam_uav_sim", "d435i_rtabmap.launch.py", {
            "config_file": str(sim_share / "config" / "d435i_rtabmap_feature_aligned.yaml"),
            "database_path": LaunchConfiguration("database_path"),
            "rgb_topic": "/sensors/rgbd/color",
            "depth_topic": "/sensors/rgbd/depth",
            "camera_info_topic": "/front/d435i/color/camera_info",
        }, condition=IfCondition(start_rtabmap)),
        include("uf_lio_adapter", "lio_adapter.launch.py", {
            "use_sim_time": use_sim_time,
        }),
        include("uf_visual_frontend", "visual_tight_coupling.launch.py", {
            "use_sim_time": use_sim_time,
            "enabled": "true",
            "start_fusion_stack": "true",
            "visual_factor_mode": LaunchConfiguration("visual_factor_mode"),
            "visual_keyframe_profile": LaunchConfiguration(
                "visual_keyframe_profile"
            ),
            "visual_candidate_quality_enabled": LaunchConfiguration(
                "visual_candidate_quality_enabled"
            ),
            "visual_pending_enabled": LaunchConfiguration(
                "visual_pending_enabled"
            ),
            "performance_profiling_enabled": LaunchConfiguration(
                "performance_profiling_enabled"
            ),
            "performance_trace_path": LaunchConfiguration(
                "performance_trace_path"
            ),
            "camera_time_offset_s": LaunchConfiguration("camera_time_offset_s"),
        }),
        include("uf_shared_mapping", "shared_mapping.launch.py", {
            "enabled": LaunchConfiguration("shared_mapping_enabled"),
            "use_sim_time": use_sim_time,
            "lidar_enabled": "true",
            "rgbd_enabled": LaunchConfiguration("shared_mapping_rgbd_enabled"),
            "performance_profiling_enabled": LaunchConfiguration(
                "performance_profiling_enabled"
            ),
            "output_directory": LaunchConfiguration(
                "shared_mapping_output_directory"
            ),
        }),
    ])
