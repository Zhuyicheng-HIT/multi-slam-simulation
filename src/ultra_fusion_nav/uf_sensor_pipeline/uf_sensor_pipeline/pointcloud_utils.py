import copy
import math
import struct

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


def filter_cloud(msg, bounds, min_range_m=0.1, max_range_m=100.0):
    fields = {field.name: field for field in msg.fields}
    if not {"x", "y", "z"}.issubset(fields) or msg.point_step <= 0:
        raise ValueError("PointCloud2 must contain x/y/z fields and a positive point_step")
    count = min(int(msg.width) * int(msg.height), len(msg.data) // int(msg.point_step))
    kept = []
    removed_body = 0
    removed_range = 0
    min_x, max_x, min_y, max_y, min_z, max_z = bounds
    for index in range(count):
        base = index * int(msg.point_step)
        x = float(_read_scalar(msg.data, base, fields["x"], msg.is_bigendian))
        y = float(_read_scalar(msg.data, base, fields["y"], msg.is_bigendian))
        z = float(_read_scalar(msg.data, base, fields["z"], msg.is_bigendian))
        distance = math.sqrt(x * x + y * y + z * z)
        if not math.isfinite(distance) or distance < min_range_m or distance > max_range_m:
            removed_range += 1
            continue
        if min_x <= x <= max_x and min_y <= y <= max_y and min_z <= z <= max_z:
            removed_body += 1
            continue
        kept.append(msg.data[base:base + int(msg.point_step)])

    output = copy.deepcopy(msg)
    output.height = 1
    output.width = len(kept)
    output.row_step = int(output.point_step) * int(output.width)
    output.data = b"".join(kept)
    output.is_dense = False
    return output, removed_body, removed_range, count
