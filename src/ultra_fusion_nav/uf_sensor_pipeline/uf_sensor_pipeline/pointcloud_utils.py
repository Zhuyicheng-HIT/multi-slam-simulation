import copy
import math
import struct

import numpy as np

from sensor_msgs.msg import PointCloud2, PointField


_FORMATS = {
    PointField.INT8: "b",
    PointField.UINT8: "B",
    PointField.INT16: "h",
    PointField.UINT16: "H",
    PointField.INT32: "i",
    PointField.UINT32: "I",
    PointField.FLOAT32: "f",
    PointField.FLOAT64: "d",
}


def _read_scalar(data, base, field, bigendian):
    fmt = _FORMATS.get(field.datatype)
    if fmt is None:
        raise ValueError(f"Unsupported PointField datatype for {field.name}: {field.datatype}")
    prefix = ">" if bigendian else "<"
    return struct.unpack_from(prefix + fmt, data, base + int(field.offset))[0]


def _finite_tuple(values, length, name):
    values = tuple(float(value) for value in values)
    if len(values) != length or not all(math.isfinite(value) for value in values):
        raise ValueError(f"{name} must contain {length} finite values")
    return values


def filter_cloud(
    msg,
    bounds,
    min_range_m=0.1,
    max_range_m=100.0,
    lidar_to_body_rotation=None,
    lidar_to_body_translation=None,
):
    fields = {field.name: field for field in msg.fields}
    if not {"x", "y", "z"}.issubset(fields) or msg.point_step <= 0:
        raise ValueError("PointCloud2 must contain x/y/z fields and a positive point_step")
    count = min(int(msg.width) * int(msg.height), len(msg.data) // int(msg.point_step))
    kept = []
    removed_body = 0
    removed_range = 0
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    rotation = _finite_tuple(
        (1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0)
        if lidar_to_body_rotation is None else lidar_to_body_rotation,
        9,
        "lidar_to_body_rotation",
    )
    translation = _finite_tuple(
        (0.0, 0.0, 0.0)
        if lidar_to_body_translation is None else lidar_to_body_translation,
        3,
        "lidar_to_body_translation",
    )

    # MID360 publishes the common little-endian float32 XYZ layout.  Use a
    # structured NumPy view for this hot path; retain the scalar fallback for
    # other PointCloud2 layouts.
    xyz_fields = (fields["x"], fields["y"], fields["z"])
    if (
        not msg.is_bigendian
        and all(field.datatype == PointField.FLOAT32 and field.count == 1 for field in xyz_fields)
    ):
        dtype = np.dtype({
            "names": ["x", "y", "z"],
            "formats": ["<f4", "<f4", "<f4"],
            "offsets": [int(field.offset) for field in xyz_fields],
            "itemsize": int(msg.point_step),
        })
        points = np.frombuffer(msg.data, dtype=dtype, count=count)
        xyz = np.column_stack((points["x"], points["y"], points["z"]))
        distance = np.sqrt(np.sum(xyz * xyz, axis=1))
        finite = np.isfinite(distance)
        valid = finite & (distance >= min_range_m) & (distance <= max_range_m)
        body = (
            (rotation[0] * xyz[:, 0] + rotation[1] * xyz[:, 1] + rotation[2] * xyz[:, 2] + translation[0] >= min_x)
            & (rotation[0] * xyz[:, 0] + rotation[1] * xyz[:, 1] + rotation[2] * xyz[:, 2] + translation[0] <= max_x)
            & (rotation[3] * xyz[:, 0] + rotation[4] * xyz[:, 1] + rotation[5] * xyz[:, 2] + translation[1] >= min_y)
            & (rotation[3] * xyz[:, 0] + rotation[4] * xyz[:, 1] + rotation[5] * xyz[:, 2] + translation[1] <= max_y)
            & (rotation[6] * xyz[:, 0] + rotation[7] * xyz[:, 1] + rotation[8] * xyz[:, 2] + translation[2] >= min_z)
            & (rotation[6] * xyz[:, 0] + rotation[7] * xyz[:, 1] + rotation[8] * xyz[:, 2] + translation[2] <= max_z)
        )
        keep_indices = np.flatnonzero(valid & ~body)
        raw = np.frombuffer(msg.data, dtype=np.uint8, count=count * int(msg.point_step))
        kept = raw.reshape(count, int(msg.point_step))[keep_indices].tobytes()
        removed_body = int(np.count_nonzero(valid & body))
        removed_range = int(count - np.count_nonzero(valid))
    else:
        kept = []
        removed_body = 0
        removed_range = 0
        for index in range(count):
            base = index * int(msg.point_step)
            x = float(_read_scalar(msg.data, base, fields["x"], msg.is_bigendian))
            y = float(_read_scalar(msg.data, base, fields["y"], msg.is_bigendian))
            z = float(_read_scalar(msg.data, base, fields["z"], msg.is_bigendian))
            distance = math.sqrt(x * x + y * y + z * z)
            if not math.isfinite(distance) or distance < min_range_m or distance > max_range_m:
                removed_range += 1
                continue
            body_x = rotation[0] * x + rotation[1] * y + rotation[2] * z + translation[0]
            body_y = rotation[3] * x + rotation[4] * y + rotation[5] * z + translation[1]
            body_z = rotation[6] * x + rotation[7] * y + rotation[8] * z + translation[2]
            if min_x <= body_x <= max_x and min_y <= body_y <= max_y and min_z <= body_z <= max_z:
                removed_body += 1
                continue
            kept.append(msg.data[base:base + int(msg.point_step)])

    output = copy.deepcopy(msg)
    output.height = 1
    output.width = len(kept) // int(output.point_step) if isinstance(kept, bytes) else len(kept)
    output.row_step = int(output.point_step) * int(output.width)
    output.data = kept if isinstance(kept, bytes) else b"".join(kept)
    output.is_dense = False
    return output, removed_body, removed_range, count
