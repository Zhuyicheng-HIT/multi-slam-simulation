from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    backend_share = get_package_share_directory("uf_backend_fusion")
    reliability_share = get_package_share_directory("uf_reliability")
    backend_config = backend_share + "/config/online_backend.yaml"
    reliability_config = reliability_share + "/config/reliability.yaml"
    scheduler_config = reliability_share + "/config/scheduler_config.yaml"
    return LaunchDescription([
        Node(
            package="uf_reliability",
            executable="reliability_monitor",
            name="reliability_monitor",
            parameters=[reliability_config],
            output="screen",
        ),
        Node(
            package="uf_reliability",
            executable="reliability_scheduler",
            name="reliability_scheduler",
            parameters=[
                scheduler_config,
                {"active_modalities": ["lidar", "gnss", "imu", "optical_flow"]},
            ],
            output="screen",
        ),
        Node(
            package="uf_backend_fusion",
            executable="online_backend_fusion",
            name="unified_backend_fusion",
            parameters=[backend_config],
            output="screen",
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="external_nav_gate",
            name="unified_external_nav_gate",
            parameters=[{
                "input_topic": "/fusion/unified/odom",
                "output_topic": "/mavros/odometry/out",
                "expected_map_frame": "map",
                "expected_body_frame": "base_link",
                "maximum_input_age_s": 0.25,
                "minimum_rate_hz": 4.0,
                "enabled": True,
            }],
            output="screen",
        ),
    ])
