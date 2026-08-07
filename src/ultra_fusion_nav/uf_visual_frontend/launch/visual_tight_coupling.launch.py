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
        DeclareLaunchArgument("enabled", default_value="false"),
        DeclareLaunchArgument("start_fusion_stack", default_value="false"),
        DeclareLaunchArgument("camera_time_offset_s", default_value="0.0"),
        Node(
            package="uf_visual_frontend", executable="rgbd_feature_frontend",
            parameters=[frontend_config], output="screen",
            condition=IfCondition(LaunchConfiguration("enabled")),
        ),
        Node(
            package="uf_reliability", executable="reliability_monitor",
            condition=IfCondition(LaunchConfiguration("start_fusion_stack")),
            output="screen",
        ),
        Node(
            package="uf_reliability", executable="reliability_scheduler",
            parameters=[{
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
                "visual_factor_mode": "paper_reprojection",
                "visual_time_offset_s": LaunchConfiguration("camera_time_offset_s"),
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
    ])
