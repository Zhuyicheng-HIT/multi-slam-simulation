import copy
import math
import struct

import numpy as np
from sensor_msgs.msg import PointField


def standardize_imu_acceleration(msg, scale):
    """Convert linear acceleration to SI units, preserving unknown covariance."""
    output = copy.deepcopy(msg)
    scale = float(scale)
    if not math.isfinite(scale) or scale <= 0.0:
        raise ValueError("imu_acceleration_scale must be finite and positive")
    output.linear_acceleration.x *= scale
    output.linear_acceleration.y *= scale
    output.linear_acceleration.z *= scale
    covariance = list(output.linear_acceleration_covariance)
    if not covariance or covariance[0] != -1.0:
        scale_squared = scale * scale
        output.linear_acceleration_covariance = [value * scale_squared for value in covariance]
    return output


def _proper_rotation_matrix(values):
    rotation = tuple(float(value) for value in values)
    if len(rotation) != 9 or not all(math.isfinite(value) for value in rotation):
        raise ValueError("mid360_to_body_rotation must be a proper orthonormal rotation")
    rows = (rotation[0:3], rotation[3:6], rotation[6:9])
    tolerance = 1.0e-6
    for row_index, row in enumerate(rows):
        for other_index, other in enumerate(rows):
            dot = sum(row[column] * other[column] for column in range(3))
            expected = 1.0 if row_index == other_index else 0.0
            if abs(dot - expected) > tolerance:
                raise ValueError(
                    "mid360_to_body_rotation must be a proper orthonormal rotation"
                )
    determinant = (
        rotation[0] * (rotation[4] * rotation[8] - rotation[5] * rotation[7])
        - rotation[1] * (rotation[3] * rotation[8] - rotation[5] * rotation[6])
        + rotation[2] * (rotation[3] * rotation[7] - rotation[4] * rotation[6])
    )
    if abs(determinant - 1.0) > tolerance:
        raise ValueError("mid360_to_body_rotation must be a proper orthonormal rotation")
    return rotation


def _rotate_vector(vector, rotation):
    source = (float(vector.x), float(vector.y), float(vector.z))
    return tuple(
        sum(rotation[row * 3 + column] * source[column] for column in range(3))
        for row in range(3)
    )


def _rotate_covariance(covariance, rotation):
    values = tuple(float(value) for value in covariance)
    if len(values) != 9:
        raise ValueError("IMU covariance must contain 9 values")
    if values[0] == -1.0:
        return values
    if not all(math.isfinite(value) for value in values):
        raise ValueError("IMU covariance must contain finite values or the -1 sentinel")
    intermediate = [
        sum(rotation[row * 3 + inner] * values[inner * 3 + column]
            for inner in range(3))
        for row in range(3)
        for column in range(3)
    ]
    return [
        sum(intermediate[row * 3 + inner] * rotation[column * 3 + inner]
            for inner in range(3))
        for row in range(3)
        for column in range(3)
    ]


def standardize_imu_to_body(msg, scale, mid360_to_body_rotation, output_frame_id):
    """Convert MID360 IMU units and express vector/covariance data in body FLU.

    ``mid360_to_body_rotation`` is :math:`R_{body\leftarrow mid360}`.  It is
    deliberately independent from FAST-LIO's internal LiDAR-to-IMU extrinsic.
    """
    rotation = _proper_rotation_matrix(mid360_to_body_rotation)
    output = standardize_imu_acceleration(msg, scale)
    acceleration = _rotate_vector(output.linear_acceleration, rotation)
    angular_velocity = _rotate_vector(output.angular_velocity, rotation)
    output.linear_acceleration.x, output.linear_acceleration.y, output.linear_acceleration.z = (
        acceleration
    )
    output.angular_velocity.x, output.angular_velocity.y, output.angular_velocity.z = (
        angular_velocity
    )
    output.linear_acceleration_covariance = _rotate_covariance(
        output.linear_acceleration_covariance, rotation
    )
    output.angular_velocity_covariance = _rotate_covariance(
        output.angular_velocity_covariance, rotation
    )
    # Livox MID360 publishes acceleration and angular velocity but no
    # orientation estimate.  ROS Imu uses orientation_covariance[0] == -1 as
    # the explicit unavailable sentinel.
    output.orientation.x = 0.0
    output.orientation.y = 0.0
    output.orientation.z = 0.0
    output.orientation.w = 1.0
    output.orientation_covariance = [-1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    output.header.frame_id = str(output_frame_id)
    return output


_POINT_FIELD_FORMATS = {
    PointField.FLOAT32: "f",
    PointField.FLOAT64: "d",
}


def shift_stamp(stamp, offset_s):
    total_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    total_ns = max(0, total_ns + int(round(offset_s * 1.0e9)))
    stamp.sec, stamp.nanosec = divmod(total_ns, 1_000_000_000)


def ensure_monotonic_stamp(stamp, last_stamp_ns, repair=True):
    stamp_ns = int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    repaired = stamp_ns <= last_stamp_ns and stamp_ns != 0
    if repaired:
        if not repair:
            raise ValueError(
                f"non-monotonic timestamp: {stamp_ns} <= {last_stamp_ns}"
            )
        stamp_ns = last_stamp_ns + 1
        stamp.sec, stamp.nanosec = divmod(stamp_ns, 1_000_000_000)
    return max(last_stamp_ns, stamp_ns), repaired


def drop_pointcloud(msg, fraction, rng):
    output = copy.deepcopy(msg)
    count = min(int(msg.width) * int(msg.height), len(msg.data) // max(1, int(msg.point_step)))
    if count <= 0 or fraction <= 0.0:
        return output
    keep = rng.random(count) >= min(1.0, fraction)
    chunks = [
        msg.data[index * int(msg.point_step):(index + 1) * int(msg.point_step)]
        for index in range(count) if keep[index]
    ]
    output.height = 1
    output.width = len(chunks)
    output.row_step = int(output.point_step) * int(output.width)
    output.data = b"".join(chunks)
    output.is_dense = False
    return output


def add_moving_lidar_cluster(msg, point_count, elapsed_s, speed_mps=0.6):
    """Append a timestamp-driven moving cuboid while preserving point records."""
    output = copy.deepcopy(msg)
    requested = max(0, int(round(point_count)))
    point_step = int(msg.point_step)
    source_count = min(
        int(msg.width) * int(msg.height),
        len(msg.data) // max(1, point_step),
    )
    if requested == 0 or source_count == 0 or point_step <= 0:
        return output

    fields = {field.name: field for field in msg.fields}
    if not {"x", "y", "z"}.issubset(fields):
        raise ValueError("moving LiDAR cluster requires x/y/z fields")
    for name in ("x", "y", "z"):
        field = fields[name]
        if field.datatype not in _POINT_FIELD_FORMATS or int(field.count) != 1:
            raise ValueError(f"moving LiDAR cluster requires scalar floating-point {name}")

    prefix = ">" if msg.is_bigendian else "<"
    template = msg.data[:point_step]
    side = max(2, int(math.ceil(requested ** (1.0 / 3.0))))
    center_x = 5.0
    center_y = -2.0 + (float(speed_mps) * max(0.0, float(elapsed_s))) % 4.0
    center_z = -0.3
    injected = []
    for index in range(requested):
        ix = index % side
        iy = (index // side) % side
        iz = (index // (side * side)) % side
        x = center_x + 0.8 * (ix / (side - 1) - 0.5)
        y = center_y + 0.8 * (iy / (side - 1) - 0.5)
        z = center_z + 1.4 * (iz / (side - 1) - 0.5)
        record = bytearray(template)
        for name, value in (("x", x), ("y", y), ("z", z)):
            field = fields[name]
            struct.pack_into(
                prefix + _POINT_FIELD_FORMATS[field.datatype],
                record,
                int(field.offset),
                float(value),
            )
        injected.append(bytes(record))

    output.height = 1
    output.width = source_count + requested
    output.row_step = point_step * int(output.width)
    output.data = bytes(msg.data[:source_count * point_step]) + b"".join(injected)
    output.is_dense = False
    return output


def add_gnss_jump(msg, north_m, east_m=0.0):
    output = copy.deepcopy(msg)
    earth_radius_m = 6_378_137.0
    output.latitude += math.degrees(north_m / earth_radius_m)
    longitude_scale = max(1.0e-6, math.cos(math.radians(output.latitude)))
    output.longitude += math.degrees(east_m / (earth_radius_m * longitude_scale))
    return output


def add_depth_holes(msg, fraction, rng):
    output = copy.deepcopy(msg)
    if fraction <= 0.0 or not msg.data:
        return output
    if msg.encoding in ("16UC1", "mono16"):
        values = np.frombuffer(msg.data, dtype=np.uint16).copy()
    elif msg.encoding == "32FC1":
        values = np.frombuffer(msg.data, dtype=np.float32).copy()
    else:
        return output
    values[rng.random(values.size) < min(1.0, fraction)] = 0
    output.data = values.tobytes()
    return output


def flatten_image(msg, level=127):
    output = copy.deepcopy(msg)
    if msg.encoding in ("rgb8", "bgr8", "rgba8", "bgra8", "mono8"):
        output.data = bytes([max(0, min(255, int(level)))]) * len(msg.data)
    return output
