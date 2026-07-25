import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = get_package_share_directory("uf_sensor_pipeline") + "/config/sim_sensor_config.yaml"
    config = LaunchConfiguration("config")
    use_sim_time = LaunchConfiguration("use_sim_time")
    enable_fcu_observation_bridge = LaunchConfiguration("enable_fcu_observation_bridge")
    scheduled_fault_modality = os.environ.get("UF_FAULT_MODALITY", "").strip()
    scheduled_fault = {}
    if scheduled_fault_modality:
        scheduled_fault = {
            "fault_type": os.environ.get("UF_FAULT_TYPE", "none"),
            "fault_start_s": float(os.environ.get("UF_FAULT_START_S", "0.0")),
            "fault_duration_s": float(os.environ.get("UF_FAULT_DURATION_S", "0.0")),
            "magnitude": float(os.environ.get("UF_FAULT_MAGNITUDE", "0.0")),
            "secondary_magnitude": float(
                os.environ.get("UF_FAULT_SECONDARY_MAGNITUDE", "0.0")
            ),
        }
    nodes = [
        Node(
            package="uf_sensor_pipeline",
            executable="fcu_observation_bridge",
            name="fcu_observation_bridge",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
            condition=IfCondition(enable_fcu_observation_bridge),
        ),
        Node(
            package="uf_sensor_pipeline",
            executable="pointcloud_body_filter",
            name="pointcloud_body_filter",
            parameters=[config, {"use_sim_time": use_sim_time}],
            output="screen",
        )
    ]
    for modality in ("lidar", "imu", "gnss", "optical_flow", "depth", "color"):
        fault_parameters = scheduled_fault if modality == scheduled_fault_modality else {}
        nodes.append(
            Node(
                package="uf_sensor_pipeline",
                executable="fault_injector",
                name=f"fault_injector_{modality}",
                parameters=[config, fault_parameters, {"use_sim_time": use_sim_time}],
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
        DeclareLaunchArgument("enable_fcu_observation_bridge", default_value="false"),
        *nodes,
    ])
