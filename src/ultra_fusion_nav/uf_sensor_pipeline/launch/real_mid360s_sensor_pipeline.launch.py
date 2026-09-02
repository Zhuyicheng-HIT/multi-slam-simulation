"""Real MID360S sensor normalization with one canonical geometry contract.

The Livox driver and FAST-LIO keep ownership of immutable /livox/lidar and
/livox/imu. This launch creates the body-FLU/SI IMU copy and the independently
disableable, fail-open Livox body-filter copy. The generic simulation D435i TF
remains disabled because hardware body/camera closure is derived and validated
from the calibrated camera/LiDAR transform in the geometry contract.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    share = get_package_share_directory("uf_sensor_pipeline")
    pipeline_launch = os.path.join(share, "launch", "sensor_pipeline.launch.py")
    hardware_config = os.path.join(share, "config", "real_mid360_imu_units.yaml")

    return LaunchDescription([
        DeclareLaunchArgument("enable_vision", default_value="false"),
        DeclareLaunchArgument("enable_gnss", default_value="false"),
        DeclareLaunchArgument("enable_fault_injection", default_value="false"),
        DeclareLaunchArgument(
            "active_modalities",
            default_value="[imu]",
            description=(
                "Real relay modalities. LiDAR remains Livox CustomMsg and is not "
                "routed through the PointCloud2 relay manager."
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(pipeline_launch),
            launch_arguments={
                "config": hardware_config,
                "use_sim_time": "false",
                "enable_lidar": "true",
                "enable_gnss": LaunchConfiguration("enable_gnss"),
                "enable_vision": LaunchConfiguration("enable_vision"),
                "enable_fault_injection": LaunchConfiguration("enable_fault_injection"),
                "active_modalities": LaunchConfiguration("active_modalities"),
                "publish_d435i_mount_tf": "false",
            }.items(),
        ),
    ])
