import struct
from types import SimpleNamespace

import numpy as np
import pytest

from uf_map_maintenance.pointcloud_adapter import (
    PointCloudContractError,
    decode_mid360_pointcloud2,
)
from uf_map_maintenance.normalize import prepare_output_path


FIELD_LAYOUT = [
    SimpleNamespace(name="x", offset=0, datatype=7, count=1),
    SimpleNamespace(name="y", offset=4, datatype=7, count=1),
    SimpleNamespace(name="z", offset=8, datatype=7, count=1),
    SimpleNamespace(name="intensity", offset=12, datatype=7, count=1),
    SimpleNamespace(name="tag", offset=16, datatype=2, count=1),
    SimpleNamespace(name="line", offset=17, datatype=2, count=1),
    SimpleNamespace(name="timestamp", offset=18, datatype=8, count=1),
]


def pack_point(values, big_endian=False):
    prefix = ">" if big_endian else "<"
    return struct.pack(prefix + "ffffBBd", *values)


def decode(data, *, stamp_ns=1_000_000_000, width=1, height=1,
           row_step=None, big_endian=False):
    return decode_mid360_pointcloud2(
        data=data,
        width=width,
        height=height,
        point_step=26,
        row_step=row_step or width * 26,
        fields=FIELD_LAYOUT,
        is_bigendian=big_endian,
        header_stamp_ns=stamp_ns,
    )


def test_exact_mid360_layout_maps_intensity_and_preserves_identity_fields():
    stamp = 1_000_000_000
    data = pack_point((1.0, -2.0, 3.5, 17.6, 5, 3, float(stamp + 42)))
    scan = decode(data, stamp_ns=stamp)

    np.testing.assert_allclose(scan.points_xyz, [[1.0, -2.0, 3.5]])
    np.testing.assert_array_equal(scan.reflectivity, [18])
    np.testing.assert_array_equal(scan.offset_time, [42])
    np.testing.assert_array_equal(scan.line, [3])
    np.testing.assert_array_equal(scan.tag, [5])
    assert scan.source_points == 1
    assert scan.finite_points == 1


def test_reflectivity_is_rounded_clamped_and_nonfinite_becomes_zero():
    stamp = 2_000_000_000
    rows = [
        (0.0, 0.0, 0.0, -4.0, 1, 0, float(stamp)),
        (1.0, 0.0, 0.0, 300.0, 2, 1, float(stamp + 1)),
        (2.0, 0.0, 0.0, float("nan"), 3, 2, float(stamp + 2)),
    ]
    scan = decode(b"".join(pack_point(row) for row in rows), stamp_ns=stamp, width=3)
    np.testing.assert_array_equal(scan.reflectivity, [0, 255, 0])
    assert scan.zero_reflectivity_from_nonfinite == 1
    assert len(scan.points_xyz) == 3  # finite zero returns remain transport-preserved


def test_nonfinite_xyz_is_removed_without_reordering_remaining_points():
    stamp = 3_000_000_000
    rows = [
        (1.0, 0.0, 0.0, 1.0, 4, 0, float(stamp + 1)),
        (float("nan"), 0.0, 0.0, 2.0, 5, 1, float(stamp + 2)),
        (3.0, 0.0, 0.0, 3.0, 6, 2, float(stamp + 3)),
    ]
    scan = decode(b"".join(pack_point(row) for row in rows), stamp_ns=stamp, width=3)
    np.testing.assert_allclose(scan.points_xyz[:, 0], [1.0, 3.0])
    np.testing.assert_array_equal(scan.tag, [4, 6])
    np.testing.assert_array_equal(scan.offset_time, [1, 3])
    assert scan.rejected_nonfinite_xyz == 1


def test_big_endian_and_organized_row_padding_are_supported():
    stamp = 4_000_000_000
    row0 = pack_point((1, 2, 3, 4, 5, 1, float(stamp)), True) + b"pad!"
    row1 = pack_point((6, 7, 8, 9, 10, 2, float(stamp + 256)), True) + b"pad!"
    scan = decode(
        row0 + row1,
        stamp_ns=stamp,
        width=1,
        height=2,
        row_step=30,
        big_endian=True,
    )
    np.testing.assert_allclose(scan.points_xyz, [[1, 2, 3], [6, 7, 8]])
    np.testing.assert_array_equal(scan.offset_time, [0, 256])
    np.testing.assert_array_equal(scan.line, [1, 2])


def test_absolute_float64_timestamp_uses_same_precision_domain_as_header():
    stamp = 1_787_396_484_768_052_729
    encoded_header = float(stamp)
    assert int(round(encoded_header)) - stamp == 7
    scan = decode(
        pack_point((1, 2, 3, 4, 5, 1, encoded_header)),
        stamp_ns=stamp,
    )
    np.testing.assert_array_equal(scan.offset_time, [0])


@pytest.mark.parametrize(
    "timestamp",
    [999_999_744.0, 1_000_000_000.0 + 2**32],
)
def test_negative_or_uint32_overflow_offset_is_rejected(timestamp):
    data = pack_point((1, 2, 3, 4, 5, 1, timestamp))
    with pytest.raises(PointCloudContractError, match="offset_time"):
        decode(data, stamp_ns=1_000_000_000)


def test_missing_or_mistyped_field_is_rejected():
    fields = list(FIELD_LAYOUT)
    fields[-1] = SimpleNamespace(name="timestamp", offset=18, datatype=7, count=1)
    with pytest.raises(PointCloudContractError, match="timestamp"):
        decode_mid360_pointcloud2(
            data=b"\0" * 26,
            width=1,
            height=1,
            point_step=26,
            row_step=26,
            fields=fields,
            is_bigendian=False,
            header_stamp_ns=0,
        )


def test_truncated_payload_is_rejected():
    with pytest.raises(PointCloudContractError, match="payload"):
        decode(b"\0" * 25)


def test_rosbag_writer_output_path_is_not_precreated(tmp_path):
    output = tmp_path / "derived" / "normalized_bag"
    temporary = prepare_output_path(output)
    assert output.parent.is_dir()
    assert temporary == output.with_name(output.name + ".incomplete")
    assert not temporary.exists()
