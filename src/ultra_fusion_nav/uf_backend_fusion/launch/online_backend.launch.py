import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    backend_share = get_package_share_directory("uf_backend_fusion")
    reliability_share = get_package_share_directory("uf_reliability")
    relocalization_share = get_package_share_directory("uf_relocalization")
    backend_config = backend_share + "/config/online_backend.yaml"
    reliability_config = reliability_share + "/config/reliability.yaml"
    scheduler_config = reliability_share + "/config/scheduler_config.yaml"
    calibration_motion_config = (
        relocalization_share + "/config/lidar_calibration_motion.yaml"
    )
    relocalization_config = relocalization_share + "/config/relocalization.yaml"
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_vision = LaunchConfiguration("enable_vision")
    external_nav_output_topic = LaunchConfiguration("external_nav_output_topic")
    publish_mavros_frame_transforms = LaunchConfiguration(
        "publish_mavros_frame_transforms"
    )
    relocalization_prefix = None
    if os.environ.get("UF_RELOCALIZATION_GDB", "0") == "1":
        relocalization_prefix = (
            "gdb -q -batch -ex run -ex 'thread apply all bt full' --args"
        )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("enable_vision", default_value="false"),
        DeclareLaunchArgument("visual_factor_mode", default_value="disabled"),
        DeclareLaunchArgument("rgbd_minimum_depth_m", default_value="0.30"),
        DeclareLaunchArgument("rgbd_maximum_depth_m", default_value="6.0"),
        DeclareLaunchArgument("rgbd_depth_factor_enabled", default_value="false"),
        DeclareLaunchArgument(
            "range_facet_enabled",
            default_value="false",
            description=(
                "Fuse the MTF-01P ray/plane RangeFacet row with its flow packet"
            ),
        ),
        DeclareLaunchArgument(
            "rgbd_depth_healthy_lidar_stride",
            default_value="1",
            description=(
                "Keep every quality-gated RGB-D depth batch; values above one "
                "are retained only for controlled ablation"
            ),
        ),
        DeclareLaunchArgument("reliability_mode", default_value="dynamic"),
        DeclareLaunchArgument("fixed_lidar_weight", default_value="1.0"),
        DeclareLaunchArgument("fixed_gnss_weight", default_value="1.0"),
        DeclareLaunchArgument("fixed_imu_weight", default_value="1.0"),
        DeclareLaunchArgument("fixed_optical_flow_weight", default_value="1.0"),
        DeclareLaunchArgument("fixed_vision_weight", default_value="1.0"),
        DeclareLaunchArgument(
            "frontend_map_commit_delay_states",
            default_value="7",
            description=(
                "Fixed-lag states retained before an irreversible LiDAR map write"
            ),
        ),
        DeclareLaunchArgument(
            "external_nav_output_topic",
            default_value="/mavros/odometry/out",
            description=(
                "ExternalNav gate output; use a non-MAVROS topic for estimator-only "
                "validation"
            ),
        ),
        DeclareLaunchArgument(
            "publish_mavros_frame_transforms",
            default_value="true",
            description=(
                "Publish the FLU-to-FRD frame aliases required by the MAVROS "
                "ODOMETRY plugin"
            ),
        ),
        DeclareLaunchArgument(
            "frontend_state_seed_enabled",
            default_value="false",
            description="Publish integrity-checked backend state seeds to FAST-LIO",
        ),
        DeclareLaunchArgument(
            "frontend_scan_prediction_enabled",
            default_value="true",
            description="Serve backend-owned scan trajectories to the LiDAR front-end",
        ),
        DeclareLaunchArgument(
            "preserve_lio_anchor",
            default_value="false",
            description="Legacy weak LiDAR anchor; disabled for backend-owned trajectory mode",
        ),
        DeclareLaunchArgument(
            "relocalization_search_timeout_s",
            default_value="6.0",
            description="Maximum ROS-time search budget for one relocalization request",
        ),
        DeclareLaunchArgument(
            "performance_profiling_enabled",
            default_value="false",
            description="Record bounded per-cycle backend timing and resource evidence",
        ),
        DeclareLaunchArgument(
            "calibration_apply_locked_time_offset",
            default_value="false",
            description="Apply a locked LiDAR/IMU time offset independently",
        ),
        DeclareLaunchArgument(
            "calibration_apply_locked_rotation",
            default_value="false",
            description="Apply a locked online rotation instead of the measured extrinsic",
        ),
        DeclareLaunchArgument(
            "visual_time_calibration_apply_locked",
            default_value="false",
            description=(
                "Apply a locked camera/IMU time offset; keep shadow-only by default"
            ),
        ),
        DeclareLaunchArgument(
            "barometer_topic", default_value="/mavros/imu/static_pressure"
        ),
        DeclareLaunchArgument(
            "axis_information_handoff_enabled", default_value="false"
        ),
        DeclareLaunchArgument(
            "gnss_z_reanchor_enabled", default_value="false"
        ),
        DeclareLaunchArgument(
            "gnss_z_recovery_information_scale", default_value="0.50"
        ),
        DeclareLaunchArgument(
            "barometer_fallback_enabled", default_value="false"
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="camera_init_to_camera_init_ned",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "3.141592653589793", "--pitch", "0",
                "--yaw", "1.5707963267948966",
                "--frame-id", "camera_init",
                "--child-frame-id", "camera_init_ned",
            ],
            condition=IfCondition(publish_mavros_frame_transforms),
            output="screen",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="body_to_body_frd",
            arguments=[
                "--x", "0", "--y", "0", "--z", "0",
                "--roll", "3.141592653589793", "--pitch", "0", "--yaw", "0",
                "--frame-id", "body",
                "--child-frame-id", "body_frd",
            ],
            condition=IfCondition(publish_mavros_frame_transforms),
            output="screen",
        ),
        Node(
            package="uf_reliability",
            executable="reliability_monitor",
            name="reliability_monitor",
            parameters=[reliability_config, {
                "use_sim_time": use_sim_time,
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
            output="screen",
        ),
        Node(
            package="uf_reliability",
            executable="reliability_scheduler",
            name="reliability_scheduler",
            parameters=[
                scheduler_config,
                {
                    "active_modalities": ["lidar", "gnss", "imu", "optical_flow"],
                    "use_sim_time": use_sim_time,
                },
            ],
            condition=UnlessCondition(enable_vision),
            output="screen",
        ),
        Node(
            package="uf_reliability",
            executable="reliability_scheduler",
            name="reliability_scheduler",
            parameters=[
                scheduler_config,
                {
                    "active_modalities": [
                        "lidar", "gnss", "imu", "optical_flow", "vision"
                    ],
                    "use_sim_time": use_sim_time,
                },
            ],
            condition=IfCondition(enable_vision),
            output="screen",
        ),
        Node(
            package="uf_relocalization",
            executable="lidar_calibration_motion_node",
            name="lidar_calibration_motion_node",
            parameters=[calibration_motion_config, {"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="uf_backend_fusion",
            executable="online_backend_fusion",
            name="unified_backend_fusion",
            parameters=[
                backend_config,
                {
                    "use_sim_time": use_sim_time,
                    "visual_factor_mode": LaunchConfiguration(
                        "visual_factor_mode"
                    ),
                    "rgbd_depth_factor_enabled": ParameterValue(
                        LaunchConfiguration("rgbd_depth_factor_enabled"),
                        value_type=bool,
                    ),
                    "range_facet_enabled": ParameterValue(
                        LaunchConfiguration("range_facet_enabled"),
                        value_type=bool,
                    ),
                    "rgbd_depth_healthy_lidar_stride": ParameterValue(
                        LaunchConfiguration("rgbd_depth_healthy_lidar_stride"),
                        value_type=int,
                    ),
                    "reliability_mode": LaunchConfiguration("reliability_mode"),
                    "fixed_lidar_weight": ParameterValue(
                        LaunchConfiguration("fixed_lidar_weight"), value_type=float
                    ),
                    "fixed_gnss_weight": ParameterValue(
                        LaunchConfiguration("fixed_gnss_weight"), value_type=float
                    ),
                    "fixed_imu_weight": ParameterValue(
                        LaunchConfiguration("fixed_imu_weight"), value_type=float
                    ),
                    "fixed_optical_flow_weight": ParameterValue(
                        LaunchConfiguration("fixed_optical_flow_weight"),
                        value_type=float,
                    ),
                    "fixed_vision_weight": ParameterValue(
                        LaunchConfiguration("fixed_vision_weight"), value_type=float
                    ),
                    "frontend_map_commit_delay_states": ParameterValue(
                        LaunchConfiguration("frontend_map_commit_delay_states"),
                        value_type=int,
                    ),
                    "preserve_lio_anchor": ParameterValue(
                        LaunchConfiguration("preserve_lio_anchor"),
                        value_type=bool,
                    ),
                    "frontend_state_seed_enabled": ParameterValue(
                        LaunchConfiguration("frontend_state_seed_enabled"),
                        value_type=bool,
                    ),
                    "frontend_scan_prediction_enabled": ParameterValue(
                        LaunchConfiguration("frontend_scan_prediction_enabled"),
                        value_type=bool,
                    ),
                    "performance_profiling_enabled": ParameterValue(
                        LaunchConfiguration("performance_profiling_enabled"),
                        value_type=bool,
                    ),
                    "calibration_apply_locked_time_offset": ParameterValue(
                        LaunchConfiguration(
                            "calibration_apply_locked_time_offset"
                        ),
                        value_type=bool,
                    ),
                    "calibration_apply_locked_rotation": ParameterValue(
                        LaunchConfiguration("calibration_apply_locked_rotation"),
                        value_type=bool,
                    ),
                    "visual_time_calibration_apply_locked": ParameterValue(
                        LaunchConfiguration("visual_time_calibration_apply_locked"),
                        value_type=bool,
                    ),
                    "barometer_topic": LaunchConfiguration("barometer_topic"),
                    "axis_information_handoff_enabled": ParameterValue(
                        LaunchConfiguration("axis_information_handoff_enabled"),
                        value_type=bool,
                    ),
                    "gnss_z_reanchor_enabled": ParameterValue(
                        LaunchConfiguration("gnss_z_reanchor_enabled"),
                        value_type=bool,
                    ),
                    "gnss_z_recovery_information_scale": ParameterValue(
                        LaunchConfiguration(
                            "gnss_z_recovery_information_scale"
                        ),
                        value_type=float,
                    ),
                    "barometer_fallback_enabled": ParameterValue(
                        LaunchConfiguration("barometer_fallback_enabled"),
                        value_type=bool,
                    ),
                },
            ],
            output="screen",
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="external_nav_gate",
            name="unified_external_nav_gate",
            parameters=[{
                "use_sim_time": use_sim_time,
                "input_topic": "/fusion/unified/odom",
                "output_topic": external_nav_output_topic,
                # Match the native FAST-LIO factor contract. This gate validates
                # frames but does not perform a coordinate transformation.
                "expected_map_frame": "camera_init",
                "expected_body_frame": "body",
                # Bound the real backend's measured 0.53 s worst-case gap.
                # Covariance grows during propagation; longer loss still stops.
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
                    "RELOCALIZING"
                ],
                # Capability loss is reported to the safety state machine and
                # inflates covariance, but it is not an output kill switch.
                # Timestamp, covariance and physical jump guards remain hard.
                "require_capability_support": False,
                "inflate_covariance_from_estimator_support": True,
                "minimum_capability_support": 0.15,
                "maximum_propagation_age_s": 0.65,
                # Stop finite but divergent states before they reach EKF3.
                "maximum_position_variance_m2": 25.0,
                "maximum_orientation_variance_rad2": 1.0,
                # ArduPilot consumes ODOMETRY covariance as ExternalNav
                # measurement noise. Keep the stream continuous and let a
                # large finite covariance weaken EKF3 fusion while the safety
                # state machine holds. Non-finite state, stale data and jumps
                # remain hard gate failures.
                "stop_on_excessive_covariance": False,
                "maximum_position_step_m": 1.0,
                "maximum_linear_speed_mps": 10.0,
                "maximum_orientation_step_rad": 0.5,
                "maximum_angular_speed_radps": 5.0,
            }],
            output="screen",
        ),
        Node(
            package="uf_relocalization",
            executable="relocalization_node",
            name="relocalization_node",
            parameters=[
                relocalization_config,
                {
                    "use_sim_time": use_sim_time,
                    "search_timeout_s": ParameterValue(
                        LaunchConfiguration("relocalization_search_timeout_s"),
                        value_type=float,
                    ),
                },
            ],
            prefix=relocalization_prefix,
            output="screen",
        ),
    ])
