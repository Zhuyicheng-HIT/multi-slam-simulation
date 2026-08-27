import math

from uf_map_maintenance.association import PoseSample, associate_scan_to_pose


def pose(stamp, epoch=1, x=0.0):
    return PoseSample(stamp, epoch, (x, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))


def test_exact_and_nearest_source_timestamp_association():
    samples = [pose(1_000, x=1.0), pose(2_000, x=2.0)]
    assert associate_scan_to_pose(1_000, 1, samples, 100).pose.translation[0] == 1.0
    result = associate_scan_to_pose(1_960, 1, samples, 100)
    assert result.accepted and result.delta_ns == 40


def test_stale_epoch_nonfinite_and_timestamp_regression_reject():
    samples = [pose(1_000), pose(2_000, epoch=2)]
    assert associate_scan_to_pose(1_500, 1, samples, 100).reason == "pose_tolerance"
    assert associate_scan_to_pose(2_000, 1, samples, 100).reason == "epoch_mismatch"
    bad = PoseSample(1_000, 1, (math.nan, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0))
    assert associate_scan_to_pose(1_000, 1, [bad], 100).reason == "pose_nonfinite"
    assert associate_scan_to_pose(900, 1, samples, 100, previous_scan_stamp_ns=1_000).reason == "scan_timestamp_regression"
