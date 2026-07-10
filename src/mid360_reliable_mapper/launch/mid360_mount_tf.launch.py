import math
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _rpy_to_quat(roll, pitch, yaw):
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)

    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return [qx, qy, qz, qw]


def _quat_conjugate(q):
    return [-q[0], -q[1], -q[2], q[3]]


def _quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v

    # q * v * q^-1, expanded to avoid extra dependencies.
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)

    rx = vx + w * tx + (y * tz - z * ty)
    ry = vy + w * ty + (z * tx - x * tz)
    rz = vz + w * tz + (x * ty - y * tx)
    return [rx, ry, rz]


def _invert_transform(t, q):
    q_inv = _quat_conjugate(q)
    t_inv = _quat_rotate(q_inv, [-t[0], -t[1], -t[2]])
    return t_inv, q_inv


def _get_nested(data, section, key, default):
    value = data.get(section, {})
    if isinstance(value, dict):
        return value.get(key, default)
    return default


def _make_static_tf(context):
    config_path = context.perform_substitution(LaunchConfiguration("mount_config"))
    if not os.path.isabs(config_path):
        share = get_package_share_directory("mid360_reliable_mapper")
        config_path = os.path.join(share, config_path)

    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    enabled = _as_bool(data.get("enabled", True))
    if not enabled:
        print(f"MID360 mount TF disabled by config: {config_path}")
        return []

    base_frame = str(data.get("base_frame", "base_link"))
    sensor_frame = str(data.get("sensor_frame", "body"))
    publish_inverse = _as_bool(data.get("publish_inverse_for_fastlio", True))

    t = [
        float(_get_nested(data, "translation_m", "x", 0.0)),
        float(_get_nested(data, "translation_m", "y", 0.0)),
        float(_get_nested(data, "translation_m", "z", 0.0)),
    ]
    roll_deg = float(_get_nested(data, "rotation_deg", "roll", 0.0))
    pitch_deg = float(_get_nested(data, "rotation_deg", "pitch", 0.0))
    yaw_deg = float(_get_nested(data, "rotation_deg", "yaw", 0.0))
    q = _rpy_to_quat(math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg))

    if publish_inverse:
        parent_frame = sensor_frame
        child_frame = base_frame
        t_pub, q_pub = _invert_transform(t, q)
        direction = f"{sensor_frame} -> {base_frame} (inverse of configured {base_frame} -> {sensor_frame})"
    else:
        parent_frame = base_frame
        child_frame = sensor_frame
        t_pub, q_pub = t, q
        direction = f"{base_frame} -> {sensor_frame}"

    print("MID360 module mount static TF:")
    print(f"  config: {config_path}")
    print(f"  configured base_to_sensor_xyz_m: {t}")
    print(f"  configured base_to_sensor_rpy_deg: [{roll_deg}, {pitch_deg}, {yaw_deg}]")
    print(f"  publishing: {direction}")
    print(f"  published_xyz_m: {t_pub}")
    print(f"  published_quat_xyzw: {q_pub}")

    return [
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="mid360_mount_static_tf",
            arguments=[
                "--x", f"{t_pub[0]:.9f}",
                "--y", f"{t_pub[1]:.9f}",
                "--z", f"{t_pub[2]:.9f}",
                "--qx", f"{q_pub[0]:.12f}",
                "--qy", f"{q_pub[1]:.12f}",
                "--qz", f"{q_pub[2]:.12f}",
                "--qw", f"{q_pub[3]:.12f}",
                "--frame-id", parent_frame,
                "--child-frame-id", child_frame,
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    share = get_package_share_directory("mid360_reliable_mapper")
    default_config = os.path.join(share, "config", "mid360_mount_extrinsic.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "mount_config",
            default_value=default_config,
            description="YAML file describing the MID360 module mounting transform relative to UAV base_link.",
        ),
        OpaqueFunction(function=_make_static_tf),
    ])