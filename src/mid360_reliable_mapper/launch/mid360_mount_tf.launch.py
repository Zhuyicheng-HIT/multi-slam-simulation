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


def _resolve_config_path(path, default_package):
    prefix = "package://"
    if str(path).startswith(prefix):
        package_and_path = str(path)[len(prefix):]
        package, separator, relative = package_and_path.partition("/")
        if not separator or not package or not relative:
            raise RuntimeError(f"invalid package URI: {path}")
        return os.path.join(get_package_share_directory(package), relative)
    if os.path.isabs(path):
        return path
    return os.path.join(get_package_share_directory(default_package), path)


def _make_static_tf(context):
    config_path = _resolve_config_path(
        context.perform_substitution(LaunchConfiguration("mount_config")),
        "mid360_reliable_mapper",
    )

    with open(config_path, "r", encoding="utf-8") as f:
        data = yaml.safe_load(f) or {}

    enabled = _as_bool(data.get("enabled", True))
    if not enabled:
        print(f"MID360 mount TF disabled by config: {config_path}")
        return []

    geometry_contract_file = data.get("geometry_contract_file")
    if geometry_contract_file:
        geometry_path = _resolve_config_path(geometry_contract_file, "uf_sensor_pipeline")
        with open(geometry_path, "r", encoding="utf-8") as f:
            geometry = yaml.safe_load(f) or {}
        frames = geometry.get("frames", {})
        body_lidar = geometry.get("transforms", {}).get("body_lidar", {})
        base_frame = str(frames.get("body", "base_link"))
        sensor_frame = str(frames.get("lidar", "livox_frame"))
        translation_status = str(body_lidar.get("translation_status", "unknown"))
        translation = body_lidar.get("translation_m")
        quaternion = body_lidar.get("quaternion_xyzw")
    else:
        geometry_path = None
        body_lidar = data
        base_frame = str(data.get("base_frame", "base_link"))
        sensor_frame = str(data.get("sensor_frame", "body"))
        translation_status = str(data.get("translation_status", "unknown"))
        translation = data.get("translation_m")
        quaternion = None

    if translation_status not in ("measured", "coordinate_definition"):
        raise RuntimeError(
            "MID360 mount TF requires measured or coordinate-defined translation"
        )
    publish_inverse = _as_bool(data.get("publish_inverse_for_fastlio", True))

    if isinstance(translation, list) and len(translation) == 3:
        t = [float(value) for value in translation]
    elif isinstance(translation, dict):
        t = [float(translation.get(axis, 0.0)) for axis in ("x", "y", "z")]
    else:
        raise RuntimeError("MID360 mount TF requires translation_m with three values")

    if isinstance(quaternion, list) and len(quaternion) == 4:
        q = [float(value) for value in quaternion]
        roll_deg = pitch_deg = yaw_deg = None
    else:
        roll_deg = float(_get_nested(body_lidar, "rotation_deg", "roll", 0.0))
        pitch_deg = float(_get_nested(body_lidar, "rotation_deg", "pitch", 0.0))
        yaw_deg = float(_get_nested(body_lidar, "rotation_deg", "yaw", 0.0))
        q = _rpy_to_quat(
            math.radians(roll_deg), math.radians(pitch_deg), math.radians(yaw_deg)
        )

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
    if geometry_path:
        print(f"  geometry_contract: {geometry_path}")
    print(f"  translation_status: {translation_status}")
    print(f"  configured base_to_sensor_xyz_m: {t}")
    print(f"  configured base_to_sensor_quat_xyzw: {q}")
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
