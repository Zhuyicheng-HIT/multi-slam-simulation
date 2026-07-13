from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = get_package_share_directory("uf_sensor_pipeline") + "/config/sim_sensor_config.yaml"
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    nodes = [
        Node(
            package="uf_sensor_pipeline",
            executable="pointcloud_body_filter",
            name="pointcloud_body_filter",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
        )
    ]
    for modality in ("lidar", "imu", "gnss", "optical_flow", "depth", "color"):
        nodes.append(
            Node(
                package="uf_sensor_pipeline",
                executable="fault_injector",
                name=f"fault_injector_{modality}",
                parameters=[config, {"use_sim_time": use_sim_time}],
                output="screen",
            )
        )
    nodes.append(
        Node(
            package="uf_sensor_pipeline",
            executable="sensor_contract_monitor",
            name="sensor_contract_monitor",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
        )
    )
    return LaunchDescription([
        DeclareLaunchArgument("config", default_value=default_config),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        *nodes,
    ])
