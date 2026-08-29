import json
import math
import os
import tempfile

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text):
    return str(text).strip().lower() in ("1", "true", "yes", "on")


def _float_arg(context, name):
    return float(context.perform_substitution(LaunchConfiguration(name)))


def _rpy_deg_to_matrix(roll_deg, pitch_deg, yaw_deg):
    roll = math.radians(roll_deg)
    pitch = math.radians(pitch_deg)
    yaw = math.radians(yaw_deg)
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr,
        sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr,
        -sp, cp * sr, cp * cr,
    ]


def _runtime_workspace():
    return os.environ.get(
        "MID360_WS",
        os.path.join(tempfile.gettempdir(), "mid360_reliable_mapper"),
    )


def _make_livox_node(context, *, package_share, start_livox):
    if not _as_bool(context.perform_substitution(start_livox)):
        return []
    template_config = os.path.join(package_share, "config", "MID360s_config.json")
    workspace_dir = _runtime_workspace()
    runtime_dir = os.path.join(workspace_dir, "runtime", "livox")
    os.makedirs(runtime_dir, exist_ok=True)
    runtime_config = os.path.join(runtime_dir, "MID360s_runtime.json")

    with open(template_config, "r", encoding="utf-8") as f:
        data = json.load(f)

    lidar_ip = context.perform_substitution(LaunchConfiguration("lidar_ip"))
    host_ip = context.perform_substitution(LaunchConfiguration("host_ip"))
    frame_id = context.perform_substitution(LaunchConfiguration("livox_frame_id"))
    data["Mid360s"]["host_net_info"][0]["host_ip"] = host_ip
    data["lidar_configs"][0]["ip"] = lidar_ip

    with open(runtime_config, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2)
        f.write("\n")

    print("MID360S Livox SDK2 runtime config")
    print(f"  config: {runtime_config}")
    print(f"  lidar_ip: {lidar_ip}")
    print(f"  host_ip: {host_ip}")
    print(f"  frame_id: {frame_id}")
    print("  imu_topic: /livox/imu (MID360S internal IMU, unchanged raw topic)")

    return [
        Node(
            package="livox_ros_driver2",
            executable="livox_ros_driver2_node",
            name="livox_lidar_publisher",
            output="screen",
            parameters=[{
                "xfer_format": 1,
                "multi_topic": 0,
                "data_src": 0,
                "publish_freq": 10.0,
                "output_data_type": 0,
                "frame_id": frame_id,
                "user_config_path": runtime_config,
                "cmdline_input_bd_code": "livox0000000001",
            }],
            condition=IfCondition(start_livox),
        )
    ]


def _make_fastlio_include(context, *, package_share, start_fast_lio, use_rviz):
    if not _as_bool(context.perform_substitution(start_fast_lio)):
        return []
    fast_lio_share = get_package_share_directory("fast_lio")
    fast_lio_launch = os.path.join(fast_lio_share, "launch", "mapping.launch.py")
    base_config = os.path.join(package_share, "config", "fast_lio_real_mid360.yaml")
    workspace_dir = _runtime_workspace()
    runtime_dir = os.path.join(workspace_dir, "runtime", "fastlio")
    data_dir = os.path.join(workspace_dir, "maps")
    os.makedirs(runtime_dir, exist_ok=True)
    os.makedirs(data_dir, exist_ok=True)
    runtime_config = os.path.join(runtime_dir, "fast_lio_real_mid360_runtime.yaml")

    with open(base_config, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    params = data["/**"]["ros__parameters"]
    common = params.setdefault("common", {})
    mapping = params.setdefault("mapping", {})
    params["map_file_path"] = os.path.join(data_dir, "real_mid360_fastlio_map.pcd")

    roll = _float_arg(context, "lidar_to_imu_roll_deg")
    pitch = _float_arg(context, "lidar_to_imu_pitch_deg")
    yaw = _float_arg(context, "lidar_to_imu_yaw_deg")

    common["time_offset_lidar_to_imu"] = _float_arg(context, "time_offset_lidar_to_imu")
    common["time_sync_en"] = _as_bool(context.perform_substitution(LaunchConfiguration("time_sync_en")))
    mapping["extrinsic_est_en"] = _as_bool(
        context.perform_substitution(LaunchConfiguration("extrinsic_est_en"))
    )
    mapping["extrinsic_T"] = [
        _float_arg(context, "lidar_to_imu_x"),
        _float_arg(context, "lidar_to_imu_y"),
        _float_arg(context, "lidar_to_imu_z"),
    ]
    mapping["extrinsic_R"] = _rpy_deg_to_matrix(roll, pitch, yaw)

    with open(runtime_config, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, sort_keys=False)

    print("MID360 FAST-LIO2 runtime config")
    print(f"  config: {runtime_config}")
    print(f"  map_file_path: {params['map_file_path']}")
    print(f"  lidar_to_imu_xyz_m: {mapping['extrinsic_T']}")
    print(f"  lidar_to_imu_rpy_deg: [{roll}, {pitch}, {yaw}]")
    print(f"  time_offset_lidar_to_imu_s: {common['time_offset_lidar_to_imu']}")
    print(f"  extrinsic_est_en: {mapping['extrinsic_est_en']}")

    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(fast_lio_launch),
            launch_arguments={
                "use_sim_time": "false",
                "config_path": runtime_dir,
                "config_file": os.path.basename(runtime_config),
                "rviz": use_rviz,
            }.items(),
            condition=IfCondition(start_fast_lio),
        )
    ]


def generate_launch_description():
    package_share = get_package_share_directory("mid360_reliable_mapper")

    start_livox = LaunchConfiguration("start_livox")
    start_fast_lio = LaunchConfiguration("start_fast_lio")
    use_rviz = LaunchConfiguration("rviz")

    return LaunchDescription([
        DeclareLaunchArgument("start_livox", default_value="true"),
        DeclareLaunchArgument("start_fast_lio", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("lidar_ip", default_value="192.168.1.123"),
        DeclareLaunchArgument("host_ip", default_value="192.168.1.50"),
        DeclareLaunchArgument("livox_frame_id", default_value="livox_frame"),
        DeclareLaunchArgument("lidar_to_imu_x", default_value="-0.011"),
        DeclareLaunchArgument("lidar_to_imu_y", default_value="-0.02329"),
        DeclareLaunchArgument("lidar_to_imu_z", default_value="0.04412"),
        DeclareLaunchArgument("lidar_to_imu_roll_deg", default_value="0.0"),
        DeclareLaunchArgument("lidar_to_imu_pitch_deg", default_value="0.0"),
        DeclareLaunchArgument("lidar_to_imu_yaw_deg", default_value="0.0"),
        DeclareLaunchArgument("time_offset_lidar_to_imu", default_value="0.0"),
        DeclareLaunchArgument("time_sync_en", default_value="false"),
        DeclareLaunchArgument("extrinsic_est_en", default_value="false"),
        OpaqueFunction(
            function=_make_livox_node,
            kwargs={
                "package_share": package_share,
                "start_livox": start_livox,
            },
        ),
        OpaqueFunction(
            function=_make_fastlio_include,
            kwargs={
                "package_share": package_share,
                "start_fast_lio": start_fast_lio,
                "use_rviz": use_rviz,
            },
        ),
    ])
