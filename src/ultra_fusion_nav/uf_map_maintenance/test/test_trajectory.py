import csv

import numpy as np
import pytest

from uf_map_maintenance.association import PoseSample
from uf_map_maintenance.trajectory import (
    PoseTrajectory,
    TrajectoryContractError,
    deskew_lidar_points_to_map,
    load_pose_trajectory,
    write_pose_trajectory,
)


def sample(stamp, translation=(0.0, 0.0, 0.0), quaternion=(0.0, 0.0, 0.0, 1.0), epoch=1):
    return PoseSample(stamp, epoch, translation, quaternion)


def test_exact_pose_and_linear_translation_interpolation():
    trajectory = PoseTrajectory([
        sample(100, (0, 0, 0)),
        sample(200, (2, 4, 6)),
    ], max_bracket_span_ns=100)
    exact_t, exact_q = trajectory.interpolate(100, epoch=1)
    middle_t, middle_q = trajectory.interpolate(150, epoch=1)
    np.testing.assert_allclose(exact_t, [0, 0, 0])
    np.testing.assert_allclose(exact_q, [0, 0, 0, 1])
    np.testing.assert_allclose(middle_t, [1, 2, 3])
    np.testing.assert_allclose(middle_q, [0, 0, 0, 1])


def test_shortest_path_slerp_and_quaternion_sign_equivalence():
    trajectory = PoseTrajectory([
        sample(0, quaternion=(0, 0, 0, 1)),
        sample(100, quaternion=(0.0, 0.0, 1.0, 0.0)),
    ], 100)
    _, quaternion = trajectory.interpolate(50, epoch=1)
    np.testing.assert_allclose(np.abs(quaternion[2:]), [np.sqrt(0.5), np.sqrt(0.5)], atol=1e-8)

    sign_trajectory = PoseTrajectory([
        sample(0, quaternion=(0, 0, 0, 1)),
        sample(100, quaternion=(0, 0, 0, -1)),
    ], 100)
    _, sign_quaternion = sign_trajectory.interpolate(50, epoch=1)
    np.testing.assert_allclose(sign_quaternion, [0, 0, 0, 1], atol=1e-8)


@pytest.mark.parametrize(
    "stamp, reason",
    [(-1, "missing_left_state"), (201, "missing_right_state")],
)
def test_unbracketed_pose_is_rejected(stamp, reason):
    trajectory = PoseTrajectory([sample(0), sample(200)], 200)
    with pytest.raises(TrajectoryContractError, match=reason):
        trajectory.interpolate(stamp, epoch=1)


def test_excessive_bracket_and_epoch_crossing_are_rejected():
    with pytest.raises(TrajectoryContractError, match="bracket_span"):
        PoseTrajectory([sample(0), sample(201)], 200).interpolate(100, epoch=1)
    trajectory = PoseTrajectory([sample(0, epoch=1), sample(100, epoch=2)], 100)
    with pytest.raises(TrajectoryContractError, match="epoch_mismatch"):
        trajectory.interpolate(50, epoch=1)


def test_timestamp_regression_and_nonfinite_pose_are_rejected():
    with pytest.raises(TrajectoryContractError, match="timestamp_regression"):
        PoseTrajectory([sample(100), sample(99)], 100)
    with pytest.raises(TrajectoryContractError, match="nonfinite"):
        PoseTrajectory([sample(0), sample(1, translation=(float("nan"), 0, 0))], 1)


def test_per_point_deskew_uses_each_timestamp_and_extrinsic():
    trajectory = PoseTrajectory([
        sample(0, (0, 0, 0)),
        sample(100, (1, 0, 0)),
    ], 100)
    points = np.array([[1.0, 0, 0, 10], [1.0, 0, 0, 20]])
    output = deskew_lidar_points_to_map(
        points,
        np.array([0, 100], dtype=np.int64),
        trajectory,
        epoch=1,
        pose_child_from_scan_translation=np.array([0.5, 0, 0]),
        pose_child_from_scan_quaternion=np.array([0, 0, 0, 1]),
    )
    np.testing.assert_allclose(output[:, :3], [[1.5, 0, 0], [2.5, 0, 0]])
    np.testing.assert_allclose(output[:, 3], [10, 20])


def test_trajectory_csv_round_trip_has_no_scan_id_dependency(tmp_path):
    path = tmp_path / "trajectory.csv"
    samples = [sample(10, (1, 2, 3)), sample(20, (4, 5, 6))]
    write_pose_trajectory(path, samples)
    with path.open(newline="") as stream:
        assert "scan_id" not in next(csv.reader(stream))
    loaded = load_pose_trajectory(path)
    assert loaded == samples
