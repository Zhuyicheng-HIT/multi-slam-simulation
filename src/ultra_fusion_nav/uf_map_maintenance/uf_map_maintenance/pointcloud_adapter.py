"""Exact PointCloud2-to-Livox field normalization for recorded MID360 scans."""

from dataclasses import dataclass

import numpy as np


class PointCloudContractError(ValueError):
    """Raised when the recorded PointCloud2 violates the MID360 contract."""


_DATATYPES = {
    2: "u1",  # sensor_msgs/PointField.UINT8
    7: "f4",  # sensor_msgs/PointField.FLOAT32
    8: "f8",  # sensor_msgs/PointField.FLOAT64
}

_EXPECTED = {
    "x": (0, 7),
    "y": (4, 7),
    "z": (8, 7),
    "intensity": (12, 7),
    "tag": (16, 2),
    "line": (17, 2),
    "timestamp": (18, 8),
}


@dataclass(frozen=True)
class NormalizedMid360Scan:
    points_xyz: np.ndarray
    reflectivity: np.ndarray
    offset_time: np.ndarray
    line: np.ndarray
    tag: np.ndarray
    source_points: int
    finite_points: int
    rejected_nonfinite_xyz: int
    zero_reflectivity_from_nonfinite: int


def _field_value(field, name):
    return field[name] if isinstance(field, dict) else getattr(field, name)


def _validated_dtype(fields, point_step, is_bigendian):
    by_name = {_field_value(field, "name"): field for field in fields}
    for name, (offset, datatype) in _EXPECTED.items():
        field = by_name.get(name)
        if field is None:
            raise PointCloudContractError(f"missing field: {name}")
        actual = (
            int(_field_value(field, "offset")),
            int(_field_value(field, "datatype")),
            int(_field_value(field, "count")),
        )
        if actual != (offset, datatype, 1):
            raise PointCloudContractError(
                f"invalid {name} field contract: {actual}"
            )
    if int(point_step) < 26:
        raise PointCloudContractError("point_step is smaller than MID360 layout")
    endian = ">" if is_bigendian else "<"
    return np.dtype(
        {
            "names": list(_EXPECTED),
            "formats": [
                endian + _DATATYPES[_EXPECTED[name][1]] for name in _EXPECTED
            ],
            "offsets": [_EXPECTED[name][0] for name in _EXPECTED],
            "itemsize": int(point_step),
        }
    )


def decode_mid360_pointcloud2(
    *, data, width, height, point_step, row_step, fields, is_bigendian,
    header_stamp_ns
):
    """Decode the team's exact 26-byte MID360 PointCloud2 layout.

    Absolute float64 point timestamps are converted to Livox uint32 offsets
    relative to the unchanged ROS header timestamp. Invalid offsets are
    rejected rather than clamped or synthesized.
    """
    width = int(width)
    height = int(height)
    point_step = int(point_step)
    row_step = int(row_step)
    header_stamp_ns = int(header_stamp_ns)
    if width < 0 or height < 0 or point_step <= 0:
        raise PointCloudContractError("invalid PointCloud2 dimensions")
    if row_step < width * point_step:
        raise PointCloudContractError("row_step is smaller than row payload")
    required_bytes = 0 if height == 0 else (height - 1) * row_step + width * point_step
    payload = memoryview(data)
    if len(payload) < required_bytes:
        raise PointCloudContractError("PointCloud2 payload is truncated")

    dtype = _validated_dtype(fields, point_step, bool(is_bigendian))
    rows = [
        np.frombuffer(payload, dtype=dtype, count=width, offset=row * row_step)
        for row in range(height)
    ]
    packed = np.concatenate(rows) if rows else np.empty(0, dtype=dtype)
    source_points = int(packed.size)

    xyz = np.column_stack((packed["x"], packed["y"], packed["z"]))
    finite_xyz = np.all(np.isfinite(xyz), axis=1)
    rejected_nonfinite_xyz = int(source_points - np.count_nonzero(finite_xyz))
    xyz = np.asarray(xyz[finite_xyz], dtype=np.float64)
    intensity = np.asarray(packed["intensity"][finite_xyz], dtype=np.float64)
    timestamps = np.asarray(packed["timestamp"][finite_xyz], dtype=np.float64)
    if not np.all(np.isfinite(timestamps)):
        raise PointCloudContractError("timestamp contains non-finite values")

    # The source stores Unix nanoseconds in float64. At the recorded epoch its
    # ULP is 256 ns, so subtract in the same float64 precision domain before
    # rounding. Subtracting the exact integer header after separately rounding
    # the point produces artificial offsets in [-127, 127] ns on frame starts.
    offsets_float = np.rint(timestamps - float(header_stamp_ns))
    int64 = np.iinfo(np.int64)
    if np.any(offsets_float < int64.min) or np.any(offsets_float > int64.max):
        raise PointCloudContractError("timestamp difference is outside int64 range")
    offsets = offsets_float.astype(np.int64)
    uint32 = np.iinfo(np.uint32)
    if np.any(offsets < 0) or np.any(offsets > uint32.max):
        minimum = int(offsets.min()) if offsets.size else 0
        maximum = int(offsets.max()) if offsets.size else 0
        raise PointCloudContractError(
            f"offset_time outside uint32 range: min={minimum} max={maximum}"
        )

    nonfinite_intensity = ~np.isfinite(intensity)
    reflectivity = np.rint(np.where(nonfinite_intensity, 0.0, intensity))
    reflectivity = np.clip(reflectivity, 0.0, 255.0).astype(np.uint8)
    return NormalizedMid360Scan(
        points_xyz=xyz,
        reflectivity=reflectivity,
        offset_time=offsets.astype(np.uint32),
        line=np.asarray(packed["line"][finite_xyz], dtype=np.uint8),
        tag=np.asarray(packed["tag"][finite_xyz], dtype=np.uint8),
        source_points=source_points,
        finite_points=int(xyz.shape[0]),
        rejected_nonfinite_xyz=rejected_nonfinite_xyz,
        zero_reflectivity_from_nonfinite=int(np.count_nonzero(nonfinite_intensity)),
    )
