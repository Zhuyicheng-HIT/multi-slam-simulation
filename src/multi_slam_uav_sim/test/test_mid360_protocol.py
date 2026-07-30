import math

from multi_slam_uav_sim.mid360_protocol import (
    MID360_DEFAULT_TAG,
    MID360_LINE_COUNT,
    line_for_output_index,
    relative_time_seconds,
)


def test_mid360_protocol_defaults_match_sdk2():
    assert MID360_LINE_COUNT == 4
    assert MID360_DEFAULT_TAG == 0
    assert [line_for_output_index(i) for i in range(8)] == [0, 1, 2, 3, 0, 1, 2, 3]


def test_relative_point_time_is_monotonic_across_complete_scan():
    offsets = [relative_time_seconds(i, 5, 0.1) for i in range(5)]
    assert offsets == sorted(offsets)
    assert math.isclose(offsets[0], 0.0)
    assert math.isclose(offsets[-1], 0.1)


def test_relative_point_time_clamps_source_index():
    assert relative_time_seconds(-3, 5, 0.1) == 0.0
    assert math.isclose(relative_time_seconds(99, 5, 0.1), 0.1)
