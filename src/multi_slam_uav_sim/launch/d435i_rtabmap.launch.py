#!/usr/bin/env python3
import os
import shlex
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _text(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _parameter_arguments(parameters):
    arguments = []
    for name, value in parameters.items():
        arguments.extend((f"--{name}", _text(value)))
    return " ".join(shlex.quote(item) for item in arguments)


def _launch_setup(context):
    config_path = Path(
        LaunchConfiguration("config_file").perform(context)).expanduser()
    with config_path.open("r", encoding="utf-8") as handle:
        root = yaml.safe_load(handle) or {}
    config = root.get("d435i_rtabmap", {})
    topics = config.get("topics", {})
    launch = config.get("launch", {})
    odom_parameters = config.get("odometry_parameters", {})
    rtabmap_parameters = config.get("rtabmap_parameters", {})

    min_inliers = int(odom_parameters.get("Vis/MinInliers", 10))
    if min_inliers < 10:
        raise RuntimeError(
            "D435i baseline requires Vis/MinInliers >= 10; refusing to hide "
            "tracking failures with a lower threshold.")
    loop_threshold = float(rtabmap_parameters.get("Rtabmap/LoopThr", 0.11))
    if loop_threshold < 0.11:
        raise RuntimeError(
            "D435i baseline requires Rtabmap/LoopThr >= 0.11; refusing "
            "to accept weaker loop candidates.")
    if bool(launch.get("approx_sync", False)):
        raise RuntimeError("D435i baseline requires exact RGB/depth sync.")

    rtabmap_args = _parameter_arguments(rtabmap_parameters)
    if bool(launch.get("delete_db_on_start", True)):
        rtabmap_args = "--delete_db_on_start " + rtabmap_args

    upstream = Path(get_package_share_directory("rtabmap_launch")) / \
        "launch" / "rtabmap.launch.py"
    qos_value = int(os.environ.get(
        "D435I_RTAB_QOS", launch.get("qos", 2)))
    if qos_value not in (0, 1, 2):
        raise RuntimeError("D435I_RTAB_QOS must be 0, 1, or 2")

    database_override = LaunchConfiguration("database_path").perform(context)
    database_path = Path(database_override or launch.get(
        "database_path", "~/.ros/d435i_rtabmap_baseline.db")).expanduser()

    launch_arguments = {
        "rgb_topic": topics.get("rgb", "/front/d435i/color/image_raw"),
        "depth_topic": topics.get(
            "depth", "/front/d435i/aligned_depth_to_color/image_raw"),
        "camera_info_topic": topics.get(
            "camera_info", "/front/d435i/color/camera_info"),
        "odom_topic": topics.get("odom", "/rtabmap/odom"),
        "frame_id": launch.get("frame_id", "base_link"),
        "vo_frame_id": launch.get("vo_frame_id", "odom"),
        "use_sim_time": _text(launch.get("use_sim_time", True)),
        "approx_sync": "false",
        "qos": _text(qos_value),
        "visual_odometry": _text(launch.get("visual_odometry", True)),
        "rtabmap_viz": _text(launch.get("rtabmap_viz", False)),
        "rviz": _text(launch.get("rviz", False)),
        "database_path": str(database_path),
        "rtabmap_args": rtabmap_args.strip(),
        "odom_args": _parameter_arguments(odom_parameters),
        "output": "screen",
    }
    configured_values = (
        f"config={config_path} database={database_path} "
        f"Kp/DetectorStrategy={rtabmap_parameters.get('Kp/DetectorStrategy')} "
        f"Vis/FeatureType={rtabmap_parameters.get('Vis/FeatureType')} "
        f"Mem/UseOdomFeatures={rtabmap_parameters.get('Mem/UseOdomFeatures', 'upstream-default')} "
        f"Rtabmap/LoopThr={rtabmap_parameters.get('Rtabmap/LoopThr')} "
        f"Vis/MinInliers={odom_parameters.get('Vis/MinInliers')} "
        f"Odom/ResetCountdown={odom_parameters.get('Odom/ResetCountdown')}"
    )
    return [
        LogInfo(msg="D435i RTAB-Map configured values: " + configured_values),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(upstream)),
            launch_arguments=launch_arguments.items(),
        ),
    ]


def generate_launch_description():
    default_config = Path(get_package_share_directory(
        "multi_slam_uav_sim")) / "config" / "d435i_rtabmap_feature_aligned.yaml"
    return LaunchDescription([
        DeclareLaunchArgument(
            "config_file", default_value=str(default_config),
            description="Project-owned D435i RTAB-Map baseline YAML."),
        DeclareLaunchArgument(
            "database_path", default_value="",
            description=(
                "Optional per-run database path. Empty uses the selected "
                "profile YAML value.")),
        OpaqueFunction(function=_launch_setup),
    ])
