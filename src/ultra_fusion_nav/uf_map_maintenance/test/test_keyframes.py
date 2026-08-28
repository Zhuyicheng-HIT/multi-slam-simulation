import numpy as np

from uf_map_maintenance.keyframes import select_keyframe_indices


def record(stamp_ns, x=0.0, yaw_rad=0.0):
    return {
        "stamp_ns": stamp_ns,
        "translation": np.array([x, 0.0, 0.0]),
        "quaternion": np.array([0.0, 0.0, np.sin(yaw_rad / 2), np.cos(yaw_rad / 2)]),
    }


def test_keyframes_require_translation_or_rotation_and_time_spacing():
    records = [
        record(0, 0.0, 0.0),
        record(1_000_000_000, 0.2, 0.05),
        record(2_000_000_000, 1.1, 0.05),
        record(2_500_000_000, 1.1, 0.40),  # motion qualifies, time spacing does not
        record(4_000_000_000, 1.1, 0.40),
    ]
    assert select_keyframe_indices(
        records,
        minimum_translation_m=1.0,
        minimum_rotation_rad=0.25,
        minimum_time_spacing_s=1.0,
    ) == [0, 2, 4]


def test_keyframe_selection_is_deterministic_and_keeps_final_revisit():
    records = [record(index * 1_000_000_000, float(index)) for index in range(4)]
    first = select_keyframe_indices(records, 0.5, 0.25, 0.5)
    second = select_keyframe_indices(records, 0.5, 0.25, 0.5)
    assert first == second == [0, 1, 2, 3]
