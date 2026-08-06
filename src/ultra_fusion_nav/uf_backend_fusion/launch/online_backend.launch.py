import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    external_nav_output_topic = LaunchConfiguration("external_nav_output_topic")
    relocalization_prefix = None
    if os.environ.get("UF_RELOCALIZATION_GDB", "0") == "1":
        relocalization_prefix = (
            "gdb -q -batch -ex run -ex 'thread apply all bt full' --args"
        )
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument(
            "external_nav_output_topic",
            default_value="/mavros/odometry/out",
            description=(
                "ExternalNav gate output; use a non-MAVROS topic for estimator-only "
                "validation"
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
        Node(
            package="uf_reliability",
            executable="reliability_monitor",
            name="reliability_monitor",
            parameters=[reliability_config, {"use_sim_time": use_sim_time}],
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
                # ReliabilityScheduler controls factor weights and covariance
                # inside the estimator. A valid fused state must not disappear
                # from the FCU link just because one capability is degraded.
                "require_capability_support": False,
                "maximum_propagation_age_s": 0.65,
                # Stop finite but divergent states before they reach EKF3.
                "maximum_position_variance_m2": 25.0,
                "maximum_orientation_variance_rad2": 1.0,
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
            parameters=[relocalization_config, {"use_sim_time": use_sim_time}],
            prefix=relocalization_prefix,
            output="screen",
        ),
    ])
