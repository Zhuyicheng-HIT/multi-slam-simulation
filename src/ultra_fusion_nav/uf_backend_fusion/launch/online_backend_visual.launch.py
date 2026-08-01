from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    backend_share = get_package_share_directory("uf_backend_fusion")
    reliability_share = get_package_share_directory("uf_reliability")
    backend_config = backend_share + "/config/online_backend.yaml"
    reliability_config = reliability_share + "/config/reliability.yaml"
    scheduler_config = (
        reliability_share + "/config/scheduler_visual_config.yaml")
    return LaunchDescription([
        DeclareLaunchArgument("preserve_lio_anchor", default_value="true"),
        DeclareLaunchArgument("input_trigger_mode", default_value="native_factor"),
        DeclareLaunchArgument("native_lidar_factor_enabled", default_value="true"),
        DeclareLaunchArgument("allow_lio_pose_fallback", default_value="false"),
        DeclareLaunchArgument("imu_factor_enabled", default_value="true"),
        Node(
            package="uf_reliability",
            executable="reliability_monitor",
            name="reliability_monitor",
            parameters=[
                reliability_config,
                {"vision.internal_score_enabled": False},
            ],
            output="screen",
        ),
        Node(
            package="uf_reliability",
            executable="reliability_scheduler",
            name="reliability_scheduler",
            parameters=[scheduler_config],
            output="screen",
        ),
        Node(
            package="uf_backend_fusion",
            executable="online_backend_fusion",
            name="unified_backend_fusion",
            parameters=[
                backend_config,
                {
                    "preserve_lio_anchor": ParameterValue(
                        LaunchConfiguration("preserve_lio_anchor"),
                        value_type=bool,
                    ),
                    "input_trigger_mode": LaunchConfiguration(
                        "input_trigger_mode"),
                    "native_lidar_factor_enabled": ParameterValue(
                        LaunchConfiguration("native_lidar_factor_enabled"),
                        value_type=bool,
                    ),
                    "allow_lio_pose_fallback": ParameterValue(
                        LaunchConfiguration("allow_lio_pose_fallback"),
                        value_type=bool,
                    ),
                    "imu_factor_enabled": ParameterValue(
                        LaunchConfiguration("imu_factor_enabled"),
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
                "input_topic": "/fusion/unified/odom",
                "output_topic": "/mavros/odometry/out",
                "expected_map_frame": "camera_init",
                "expected_body_frame": "body",
                "maximum_input_age_s": 0.25,
                "minimum_rate_hz": 4.0,
                "output_rate_hz": 20.0,
                "enabled": True,
                "require_scheduler_health": True,
                "scheduler_topic": "/reliability/scheduler_state",
                "scheduler_timeout_s": 0.5,
                "allowed_scheduler_states": [
                    "NORMAL", "RECOVERED", "DEGRADED", "RISK"
                ],
                "require_capability_support": False,
                "maximum_propagation_age_s": 0.35,
            }],
            output="screen",
        ),
    ])
