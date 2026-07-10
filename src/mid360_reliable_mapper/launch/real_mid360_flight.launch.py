import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("mid360_reliable_mapper")
    fastlio_launch = os.path.join(share, "launch", "real_mid360_fastlio.launch.py")
    mount_tf_launch = os.path.join(share, "launch", "mid360_mount_tf.launch.py")
    mapper_params = os.path.join(share, "config", "reliable_mapping_params.yaml")

    start_livox = LaunchConfiguration("start_livox")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    start_mount_tf = LaunchConfiguration("start_mount_tf")
    start_reliable_mapper = LaunchConfiguration("start_reliable_mapper")
    start_grid_map = LaunchConfiguration("start_grid_map")
    mount_config = LaunchConfiguration("mount_config")

    lidar_to_imu_x = LaunchConfiguration("lidar_to_imu_x")
    lidar_to_imu_y = LaunchConfiguration("lidar_to_imu_y")
    lidar_to_imu_z = LaunchConfiguration("lidar_to_imu_z")
    lidar_to_imu_roll_deg = LaunchConfiguration("lidar_to_imu_roll_deg")
    lidar_to_imu_pitch_deg = LaunchConfiguration("lidar_to_imu_pitch_deg")
    lidar_to_imu_yaw_deg = LaunchConfiguration("lidar_to_imu_yaw_deg")
    time_offset_lidar_to_imu = LaunchConfiguration("time_offset_lidar_to_imu")
    time_sync_en = LaunchConfiguration("time_sync_en")
    extrinsic_est_en = LaunchConfiguration("extrinsic_est_en")

    return LaunchDescription([
        DeclareLaunchArgument("start_livox", default_value="true"),
        DeclareLaunchArgument("start_fast_lio", default_value="true"),
        DeclareLaunchArgument("start_mount_tf", default_value="true"),
        DeclareLaunchArgument("start_reliable_mapper", default_value="true"),
        DeclareLaunchArgument("start_grid_map", default_value="true"),
        DeclareLaunchArgument(
            "mount_config",
            default_value=os.path.join(share, "config", "mid360_mount_extrinsic.yaml"),
        ),
        DeclareLaunchArgument("lidar_to_imu_x", default_value="-0.011"),
        DeclareLaunchArgument("lidar_to_imu_y", default_value="-0.02329"),
        DeclareLaunchArgument("lidar_to_imu_z", default_value="0.04412"),
        DeclareLaunchArgument("lidar_to_imu_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("lidar_to_imu_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("lidar_to_imu_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("time_offset_lidar_to_imu", default_value="0.0"),
        DeclareLaunchArgument("time_sync_en", default_value="false"),
        DeclareLaunchArgument("extrinsic_est_en", default_value="false"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mount_tf_launch),
            launch_arguments={"mount_config": mount_config}.items(),
            condition=IfCondition(start_mount_tf),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fastlio_launch),
            launch_arguments={
                "start_livox": start_livox,
                "start_fast_lio": start_fast_lio,
                "rviz": "false",
                "lidar_to_imu_x": lidar_to_imu_x,
                "lidar_to_imu_y": lidar_to_imu_y,
                "lidar_to_imu_z": lidar_to_imu_z,
                "lidar_to_imu_roll_deg": lidar_to_imu_roll_deg,
                "lidar_to_imu_pitch_deg": lidar_to_imu_pitch_deg,
                "lidar_to_imu_yaw_deg": lidar_to_imu_yaw_deg,
                "time_offset_lidar_to_imu": time_offset_lidar_to_imu,
                "time_sync_en": time_sync_en,
                "extrinsic_est_en": extrinsic_est_en,
            }.items(),
        ),
        Node(
            package="mid360_reliable_mapper",
            executable="fastlio_cloud_mapper_node",
            name="fastlio_cloud_mapper",
            output="screen",
            parameters=[mapper_params],
            condition=IfCondition(start_reliable_mapper),
        ),
        Node(
            package="mid360_reliable_mapper",
            executable="pointcloud_occupancy_grid_node",
            name="fastlio_occupancy_grid",
            output="screen",
            parameters=[mapper_params],
            condition=IfCondition(start_grid_map),
        ),
    ])