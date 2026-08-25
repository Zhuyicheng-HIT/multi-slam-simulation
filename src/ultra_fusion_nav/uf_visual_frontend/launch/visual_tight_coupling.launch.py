import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    start_fusion_stack = LaunchConfiguration("start_fusion_stack")
    external_nav_enabled = LaunchConfiguration("external_nav_enabled")
    frontend_config = os.path.join(
        get_package_share_directory(
            "uf_visual_frontend"), "config", "visual_frontend.yaml"
    )
    backend_config = os.path.join(
        get_package_share_directory(
            "uf_backend_fusion"), "config", "online_backend.yaml"
    )
    reliability_config = os.path.join(
        get_package_share_directory("uf_reliability"),
        "config", "reliability.yaml",
    )
    scheduler_config = os.path.join(
        get_package_share_directory("uf_reliability"),
        "config", "scheduler_config.yaml",
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
        DeclareLaunchArgument("rgbd_minimum_depth_m", default_value="0.30"),
        DeclareLaunchArgument("rgbd_maximum_depth_m", default_value="6.0"),
        DeclareLaunchArgument(
            "performance_profiling_enabled", default_value="false"
        ),
        DeclareLaunchArgument("performance_trace_path", default_value=""),
        DeclareLaunchArgument("backend_process_prefix", default_value=""),
        DeclareLaunchArgument("backend_numeric_threads", default_value="1"),
        DeclareLaunchArgument(
            "external_nav_output_topic",
            default_value="/fusion/runtime_external_nav",
        ),
        DeclareLaunchArgument("external_nav_enabled", default_value="true"),
        DeclareLaunchArgument("camera_time_offset_s", default_value="0.0"),
        DeclareLaunchArgument(
            "camera_time_calibration_enabled", default_value="true"
        ),
        DeclareLaunchArgument(
            "visual_initialization_require_time_lock", default_value="false"
        ),
        DeclareLaunchArgument(
            "barometer_topic", default_value="/mavros/imu/static_pressure"
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
                "candidate_require_pnp": ParameterValue(
                    PythonExpression([
                        "'", LaunchConfiguration("visual_factor_mode"),
                        "' != 'rgbd_direct'",
                    ]),
                    value_type=bool,
                ),
                "minimum_depth_m": ParameterValue(
                    LaunchConfiguration("rgbd_minimum_depth_m"),
                    value_type=float,
                ),
                "maximum_depth_m": ParameterValue(
                    LaunchConfiguration("rgbd_maximum_depth_m"),
                    value_type=float,
                ),
            }], output="screen",
            condition=IfCondition(LaunchConfiguration("enabled")),
        ),
        Node(
            package="uf_reliability", executable="reliability_monitor",
            parameters=[reliability_config, {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "vision.minimum_depth_m": ParameterValue(
                    LaunchConfiguration("rgbd_minimum_depth_m"),
                    value_type=float,
                ),
                "vision.maximum_depth_m": ParameterValue(
                    LaunchConfiguration("rgbd_maximum_depth_m"),
                    value_type=float,
                ),
                "vision.factor_mode": LaunchConfiguration("visual_factor_mode"),
            }],
            condition=IfCondition(start_fusion_stack),
            output="screen",
        ),
        Node(
            package="uf_reliability", executable="reliability_scheduler",
            parameters=[scheduler_config, {
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "active_modalities": ["lidar", "gnss", "imu", "optical_flow", "vision"],
                "required_modalities": ["imu"],
                # One valid propagation source is enough to keep publishing a
                # bounded estimator state. Optional factors are scheduled
                # independently and the safety state machine handles the
                # resulting DEGRADED/RISK condition.
                "minimum_usable_modalities": 1,
            }],
            condition=IfCondition(LaunchConfiguration("start_fusion_stack")),
            output="screen",
        ),
        Node(
            package="uf_backend_fusion", executable="online_backend_fusion",
            name="unified_backend_fusion",
            prefix=LaunchConfiguration("backend_process_prefix"),
            additional_env={
                "OMP_NUM_THREADS": LaunchConfiguration(
                    "backend_numeric_threads"
                ),
                "OPENBLAS_NUM_THREADS": LaunchConfiguration(
                    "backend_numeric_threads"
                ),
                "MKL_NUM_THREADS": LaunchConfiguration(
                    "backend_numeric_threads"
                ),
                "NUMEXPR_NUM_THREADS": LaunchConfiguration(
                    "backend_numeric_threads"
                ),
            },
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
                "barometer_topic": LaunchConfiguration("barometer_topic"),
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
                # Keep a finite, fresh estimator stream available when any
                # source survives. Capability loss is expressed by the safety
                # state and bounded covariance inflation instead of an outage.
                "require_capability_support": False,
                "inflate_covariance_from_estimator_support": True,
                "minimum_capability_support": 0.15,
                "maximum_propagation_age_s": 0.65,
                "maximum_position_variance_m2": 25.0,
                "maximum_orientation_variance_rad2": 1.0,
                "stop_on_excessive_covariance": False,
                "maximum_position_step_m": 1.0,
                "maximum_linear_speed_mps": 10.0,
                "maximum_orientation_step_rad": 0.5,
                "maximum_angular_speed_radps": 5.0,
            }],
            condition=IfCondition(PythonExpression([
                "'", start_fusion_stack, "' == 'true' and '",
                external_nav_enabled, "' == 'true'",
            ])),
            output="screen",
        ),
    ])
