import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    frontend_config = os.path.join(
        get_package_share_directory(
            "uf_visual_frontend"), "config", "visual_frontend.yaml"
    )
    backend_config = os.path.join(
        get_package_share_directory(
            "uf_backend_fusion"), "config", "online_backend.yaml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("start_fusion_stack", default_value="false"),
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
        DeclareLaunchArgument(
            "external_nav_output_topic",
            default_value="/fusion/runtime_external_nav",
        ),
        DeclareLaunchArgument("camera_time_offset_s", default_value="0.0"),
        DeclareLaunchArgument(
            "camera_time_calibration_enabled", default_value="true"
        ),
        DeclareLaunchArgument(
            "visual_initialization_require_time_lock", default_value="true"
        ),
        Node(
            package="uf_visual_frontend", executable="rgbd_feature_frontend",
            parameters=[frontend_config, {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "keyframe_profile": LaunchConfiguration(
                    "visual_keyframe_profile"
                ),
                "candidate_quality_enabled": LaunchConfiguration(
                    "visual_candidate_quality_enabled"
                ),
            }], output="screen",
            condition=IfCondition(LaunchConfiguration("enabled")),
        ),
        Node(
            package="uf_reliability", executable="reliability_monitor",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
            }],
            condition=IfCondition(LaunchConfiguration("start_fusion_stack")),
            output="screen",
        ),
        Node(
            package="uf_reliability", executable="reliability_scheduler",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "active_modalities": ["lidar", "gnss", "imu", "optical_flow", "vision"],
                "required_modalities": ["imu"],
                "minimum_usable_modalities": 2,
            }],
            condition=IfCondition(LaunchConfiguration("start_fusion_stack")),
            output="screen",
        ),
        Node(
            package="uf_backend_fusion", executable="online_backend_fusion",
            name="unified_backend_fusion",
            parameters=[backend_config, {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "visual_factor_mode": LaunchConfiguration("visual_factor_mode"),
                "visual_pending_enabled": LaunchConfiguration(
                    "visual_pending_enabled"
                ),
                "performance_profiling_enabled": LaunchConfiguration(
                    "performance_profiling_enabled"
                ),
                "performance_trace_path": LaunchConfiguration(
                    "performance_trace_path"
                ),
                "visual_time_offset_s": LaunchConfiguration("camera_time_offset_s"),
                "visual_time_calibration_enabled": LaunchConfiguration(
                    "camera_time_calibration_enabled"
                ),
                "visual_initialization_require_time_lock": LaunchConfiguration(
                    "visual_initialization_require_time_lock"
                ),
                # Paper mode requires the Stage3 native-factor contract. Keep
                # this explicit so a stale FAST-LIO overlay cannot silently
                # fall back to paired /Odometry poses.
                "native_lidar_factor_enabled": True,
                "input_trigger_mode": "native_factor",
                "frontend_scan_prediction_enabled": True,
                "allow_lio_pose_fallback": False,
                "imu_factor_enabled": True,
            }],
            condition=IfCondition(LaunchConfiguration("start_fusion_stack")),
            output="screen",
        ),
        Node(
            package="uf_sensor_pipeline", executable="external_nav_gate",
            name="unified_external_nav_gate",
            parameters=[{
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "input_topic": "/fusion/unified/odom",
                "output_topic": LaunchConfiguration("external_nav_output_topic"),
                "expected_map_frame": "camera_init",
                "expected_body_frame": "body",
                "maximum_input_age_s": 0.65,
                "minimum_rate_hz": 4.0,
                "output_rate_hz": 20.0,
                "enabled": True,
                "require_scheduler_health": True,
                "scheduler_topic": "/reliability/scheduler_state",
                "fusion_epoch_topic": "/fusion/unified/epoch",
                "scheduler_timeout_s": 0.5,
                "allowed_scheduler_states": [
                    "NORMAL", "RECOVERED", "DEGRADED", "RISK",
                    "RELOCALIZING",
                ],
                "require_capability_support": False,
                "maximum_propagation_age_s": 0.65,
                "maximum_position_variance_m2": 25.0,
                "maximum_orientation_variance_rad2": 1.0,
                "maximum_position_step_m": 1.0,
                "maximum_linear_speed_mps": 10.0,
                "maximum_orientation_step_rad": 0.5,
                "maximum_angular_speed_radps": 5.0,
            }],
            condition=IfCondition(LaunchConfiguration("start_fusion_stack")),
            output="screen",
        ),
    ])
