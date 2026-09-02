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

_NUMPY_DTYPES = {
    PointField.INT8: "i1",
    PointField.UINT8: "u1",
    PointField.INT16: "i2",
    PointField.UINT16: "u2",
    PointField.INT32: "i4",
    PointField.UINT32: "u4",
    PointField.FLOAT32: "f4",
    PointField.FLOAT64: "f8",
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
    width = max(0, int(msg.width))
    height = max(1, int(msg.height))
    point_step = int(msg.point_step)
    row_step = max(point_step * width, int(msg.row_step) or point_step * width)
    available_rows = min(height, len(msg.data) // row_step) if row_step else 0
    count = min(width * available_rows, len(msg.data) // point_step)
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
    # Interpret each field as a strided view over the original message.  This
    # preserves arbitrary point fields while avoiding one Python unpack/copy
    # per point, which was the dominant cost in the 10 Hz normalization path.
    if width and available_rows and row_step == width * point_step:
        byte_view = np.frombuffer(msg.data, dtype=np.uint8, count=count * point_step)
        records = byte_view.reshape(count, point_step)
    else:
        rows = [
            np.frombuffer(
                msg.data[row * row_step:row * row_step + width * point_step],
                dtype=np.uint8,
                count=width * point_step,
            ).reshape(width, point_step)
            for row in range(available_rows)
        ]
        records = np.concatenate(rows, axis=0) if rows else np.empty((0, point_step), dtype=np.uint8)
    endian = ">" if msg.is_bigendian else "<"

    def field_values(field):
        dtype = _NUMPY_DTYPES.get(field.datatype)
        if dtype is None or int(field.count) != 1:
            raise ValueError(f"Unsupported PointField datatype for {field.name}: {field.datatype}")
        np_dtype = np.dtype(endian + dtype)
        offset = int(field.offset)
        if offset < 0 or offset + np_dtype.itemsize > point_step:
            raise ValueError(f"PointField {field.name} exceeds point_step")
        # Explicit byte offset and strides are supported by NumPy 1.21.x and
        # avoid .view() on the non-contiguous [::point_step] field slices.
        values = np.ndarray(
            shape=(available_rows, width), dtype=np_dtype, buffer=msg.data,
            offset=offset, strides=(row_step, point_step),
        )
        return values.reshape(-1).astype(np.float64, copy=False)

    x = field_values(fields["x"])
    y = field_values(fields["y"])
    z = field_values(fields["z"])
    distance_sq = x * x + y * y + z * z
    range_keep = np.isfinite(distance_sq) & (distance_sq >= min_range_m ** 2) & (
        distance_sq <= max_range_m ** 2
    )
    removed_range = int(np.count_nonzero(~range_keep))

    body_x = rotation[0] * x + rotation[1] * y + rotation[2] * z + translation[0]
    body_y = rotation[3] * x + rotation[4] * y + rotation[5] * z + translation[1]
    body_z = rotation[6] * x + rotation[7] * y + rotation[8] * z + translation[2]
    body_keep = ~(
        (body_x >= min_x) & (body_x <= max_x) &
        (body_y >= min_y) & (body_y <= max_y) &
        (body_z >= min_z) & (body_z <= max_z)
    )
    body_removed_mask = range_keep & ~body_keep
    removed_body = int(np.count_nonzero(body_removed_mask))
    keep = range_keep & body_keep

    output = copy.deepcopy(msg)
    output.height = 1
    output.width = int(np.count_nonzero(keep))
    output.row_step = int(output.point_step) * int(output.width)
    output.data = records[keep].tobytes()
    output.is_dense = False
    return output, removed_body, removed_range, count
