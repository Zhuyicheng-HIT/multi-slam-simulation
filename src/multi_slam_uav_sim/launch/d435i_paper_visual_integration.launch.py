from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def include(package, launch_file, arguments=None, condition=None):
    source = Path(get_package_share_directory(package)) / "launch" / launch_file
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(source)),
        launch_arguments=(arguments or {}).items(),
        condition=condition,
    )


def generate_launch_description():
    sim_share = Path(get_package_share_directory("multi_slam_uav_sim"))
    sensor_share = Path(get_package_share_directory("uf_sensor_pipeline"))
    reliability_share = Path(get_package_share_directory("uf_reliability"))
    relocalization_share = Path(get_package_share_directory("uf_relocalization"))
    use_sim_time = LaunchConfiguration("use_sim_time")
    start_rtabmap = LaunchConfiguration("start_rtabmap")
    start_rgbd_bridge = LaunchConfiguration("start_rgbd_bridge")
    start_visual_frontend = LaunchConfiguration("start_visual_frontend")
    enable_vision = LaunchConfiguration("enable_vision")
    robustness_enabled = LaunchConfiguration("robustness_enabled")
    robustness_profile_path = LaunchConfiguration("robustness_profile_path")
    robustness_profile = LaunchConfiguration("robustness_profile")
    robustness_channels = LaunchConfiguration("robustness_channels")
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_rgbd_bridge", default_value="true"),
        DeclareLaunchArgument("start_visual_frontend", default_value="true"),
        DeclareLaunchArgument("enable_vision", default_value="true"),
        DeclareLaunchArgument("robustness_enabled", default_value="false"),
        DeclareLaunchArgument(
            "robustness_profile_path",
            default_value=str(sensor_share / "config" / "robustness_v3_profiles.yaml"),
        ),
        DeclareLaunchArgument("robustness_profile", default_value="nominal"),
        DeclareLaunchArgument("robustness_channels", default_value="[gnss]"),
        DeclareLaunchArgument(
            "active_modalities",
            default_value="[lidar, gnss, imu, optical_flow]",
            description="Health modalities enabled for this runtime profile",
        ),
        DeclareLaunchArgument("start_rtabmap", default_value="true"),
        DeclareLaunchArgument("database_path", default_value="paper_visual.db"),
        DeclareLaunchArgument("relocalization_database_path", default_value=""),
        DeclareLaunchArgument(
            "relocalization_database_save_on_insert", default_value="true"
        ),
        DeclareLaunchArgument(
            "relocalization_source_lio_pose_topic",
            default_value="/fusion/unified/frontend_activation_odom",
            description="Timestamp-aligned local pose for keyframe synchronization",
        ),
        DeclareLaunchArgument("camera_time_offset_s", default_value="0.0"),
        DeclareLaunchArgument(
            "camera_time_calibration_enabled", default_value="true"
        ),
        DeclareLaunchArgument(
            "visual_initialization_require_time_lock", default_value="false"
        ),
        DeclareLaunchArgument("external_nav_enabled", default_value="true"),
        DeclareLaunchArgument(
            "visual_factor_mode", default_value="paper_reprojection"
        ),
        DeclareLaunchArgument("visual_keyframe_profile", default_value="balanced"),
        DeclareLaunchArgument(
            "visual_candidate_quality_enabled", default_value="true"
        ),
        DeclareLaunchArgument("visual_pending_enabled", default_value="true"),
        DeclareLaunchArgument("rgbd_minimum_depth_m", default_value="0.30"),
        # Gazebo renders idealized depth farther than a real D435i. Keep this
        # simulation profile configurable and leave hardware-facing node
        # defaults conservative.
        DeclareLaunchArgument("rgbd_maximum_depth_m", default_value="10.0"),
        DeclareLaunchArgument(
            "performance_profiling_enabled", default_value="false"
        ),
        DeclareLaunchArgument("performance_trace_path", default_value=""),
        DeclareLaunchArgument("backend_process_prefix", default_value=""),
        DeclareLaunchArgument("backend_numeric_threads", default_value="1"),
        DeclareLaunchArgument(
            "barometer_topic", default_value="/sim/barometer/pressure"
        ),
        DeclareLaunchArgument("shared_mapping_enabled", default_value="false"),
        DeclareLaunchArgument("shared_mapping_rgbd_enabled", default_value="true"),
        DeclareLaunchArgument(
            "shared_mapping_lidar_topic",
            default_value="/cloud_registered_filtered",
        ),
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
                "min_depth_m": ParameterValue(
                    LaunchConfiguration("rgbd_minimum_depth_m"),
                    value_type=float,
                ),
                "max_depth_m": ParameterValue(
                    LaunchConfiguration("rgbd_maximum_depth_m"),
                    value_type=float,
                ),
                "sync_queue_depth": 2,
                "qos_depth": 1,
                "qos_reliability": "best_effort",
                "enable_pointcloud": False,
            }],
            condition=IfCondition(start_rgbd_bridge),
            output="screen",
        ),
        include("uf_sensor_pipeline", "sensor_pipeline.launch.py", {
            "enable_vision": enable_vision,
            "use_sim_time": use_sim_time,
            "d435_color_input_topic": "/front/d435i/color/image_raw",
            "d435_depth_input_topic": "/front/d435i/aligned_depth_to_color/image_raw",
            "d435_parent_frame": "base_link",
            "d435_child_frame": "d435i_link",
            "active_modalities": LaunchConfiguration("active_modalities"),
        }),
        Node(
            package="uf_sensor_pipeline", executable="robustness_fault_injector",
            name="robustness_v3_fault_injector",
            parameters=[{
                "use_sim_time": use_sim_time,
                "profile_path": robustness_profile_path,
                "profile": robustness_profile,
                "channels": robustness_channels,
                "native_lidar_input_topic": "/robustness/raw/native_lidar_factor",
                "native_lidar_output_topic": "/fast_lio/native_lidar_factor",
                "gnss_input_topic": "/mavros/global_position/raw/fix",
                "gnss_output_topic": "/sensors/gnss/fix",
            }],
            condition=IfCondition(robustness_enabled), output="screen",
        ),
        # The upstream RTAB launch declares a broad set of generic launch
        # configurations. Keep them scoped so they cannot reset arguments of
        # the visual backend or shared-map includes that follow.
        GroupAction(scoped=True, actions=[
            include("multi_slam_uav_sim", "d435i_rtabmap.launch.py", {
                "config_file": str(
                    sim_share / "config" / "d435i_rtabmap_feature_aligned.yaml"
                ),
                "database_path": LaunchConfiguration("database_path"),
                "rgb_topic": "/sensors/rgbd/color",
                "depth_topic": "/sensors/rgbd/depth",
                "camera_info_topic": "/front/d435i/color/camera_info",
            }, condition=IfCondition(start_rtabmap)),
        ]),
        include("uf_lio_adapter", "lio_adapter.launch.py", {
            "use_sim_time": use_sim_time,
        }),
        Node(
            package="uf_reliability", executable="relocalization_request_arbiter",
            name="relocalization_request_arbiter",
            parameters=[str(reliability_share / "config" / "relocalization_request_arbiter.yaml"),
                        {"use_sim_time": use_sim_time}], output="screen"),
        Node(
            package="uf_reliability", executable="manual_relocalization_control",
            name="manual_relocalization_control",
            parameters=[str(reliability_share / "config" / "manual_relocalization_control.yaml"),
                        {"use_sim_time": use_sim_time}], output="screen"),
        Node(
            package="uf_relocalization", executable="relocalization_node",
            name="relocalization_node",
            parameters=[str(relocalization_share / "config" / "relocalization.yaml"), {
                "use_sim_time": use_sim_time,
                "database_path": LaunchConfiguration("relocalization_database_path"),
                "source_lio_pose_topic": LaunchConfiguration(
                    "relocalization_source_lio_pose_topic"
                ),
                "database_save_on_insert": ParameterValue(
                    LaunchConfiguration("relocalization_database_save_on_insert"),
                    value_type=bool,
                ),
            }], output="screen"),
        Node(
            package="uf_relocalization", executable="active_relocalization_controller",
            name="active_relocalization_controller",
            parameters=[{"use_sim_time": use_sim_time}], output="screen"),
        include("uf_visual_frontend", "visual_tight_coupling.launch.py", {
            "use_sim_time": use_sim_time,
            "enabled": start_visual_frontend,
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
            "backend_process_prefix": LaunchConfiguration(
                "backend_process_prefix"
            ),
            "backend_numeric_threads": LaunchConfiguration(
                "backend_numeric_threads"
            ),
            "camera_time_offset_s": LaunchConfiguration("camera_time_offset_s"),
            "camera_time_calibration_enabled": LaunchConfiguration(
                "camera_time_calibration_enabled"
            ),
            "visual_initialization_require_time_lock": LaunchConfiguration(
                "visual_initialization_require_time_lock"
            ),
            "external_nav_enabled": LaunchConfiguration(
                "external_nav_enabled"
            ),
            "rgbd_minimum_depth_m": LaunchConfiguration(
                "rgbd_minimum_depth_m"
            ),
            "rgbd_maximum_depth_m": LaunchConfiguration(
                "rgbd_maximum_depth_m"
            ),
            "barometer_topic": LaunchConfiguration("barometer_topic"),
            "active_modalities": LaunchConfiguration("active_modalities"),
        }),
        include("uf_shared_mapping", "shared_mapping.launch.py", {
            "enabled": LaunchConfiguration("shared_mapping_enabled"),
            "use_sim_time": use_sim_time,
            "lidar_enabled": "true",
            "lidar_topic": LaunchConfiguration("shared_mapping_lidar_topic"),
            "rgbd_enabled": LaunchConfiguration("shared_mapping_rgbd_enabled"),
            "rgbd_minimum_depth_m": LaunchConfiguration(
                "rgbd_minimum_depth_m"
            ),
            "rgbd_maximum_depth_m": LaunchConfiguration(
                "rgbd_maximum_depth_m"
            ),
            "performance_profiling_enabled": LaunchConfiguration(
                "performance_profiling_enabled"
            ),
            "output_directory": LaunchConfiguration(
                "shared_mapping_output_directory"
            ),
        }),
    ])
