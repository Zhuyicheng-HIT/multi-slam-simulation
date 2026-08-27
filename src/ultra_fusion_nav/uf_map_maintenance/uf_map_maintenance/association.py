"""Timestamp-only deterministic scan/pose association."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class PoseSample:
    stamp_ns: int
    epoch: int
    translation: tuple
    quaternion_xyzw: tuple

    def finite(self):
        values = self.translation + self.quaternion_xyzw
        return all(math.isfinite(value) for value in values) and sum(
            value * value for value in self.quaternion_xyzw
        ) > 1e-12


@dataclass(frozen=True)
class AssociationResult:
    accepted: bool
    reason: str
    pose: PoseSample = None
    delta_ns: int = None


def associate_scan_to_pose(
    scan_stamp_ns,
    epoch,
    poses,
    tolerance_ns=50_000_000,
    previous_scan_stamp_ns=None,
):
    if scan_stamp_ns < 0 or tolerance_ns < 0:
        return AssociationResult(False, "timestamp_invalid")
    if previous_scan_stamp_ns is not None and scan_stamp_ns <= previous_scan_stamp_ns:
        return AssociationResult(False, "scan_timestamp_regression")
    if any(poses[index].stamp_ns >= poses[index + 1].stamp_ns for index in range(len(poses) - 1)):
        return AssociationResult(False, "pose_timestamp_regression")
    if any(not item.finite() for item in poses):
        return AssociationResult(False, "pose_nonfinite")
    if not poses:
        return AssociationResult(False, "pose_missing")
    nearest_any = min(poses, key=lambda item: (abs(item.stamp_ns - scan_stamp_ns), item.stamp_ns))
    if nearest_any.epoch != epoch:
        return AssociationResult(False, "epoch_mismatch", nearest_any, abs(nearest_any.stamp_ns - scan_stamp_ns))
    same_epoch = [item for item in poses if item.epoch == epoch]
    if not same_epoch:
        return AssociationResult(False, "epoch_mismatch")
    nearest = min(same_epoch, key=lambda item: (abs(item.stamp_ns - scan_stamp_ns), item.stamp_ns))
    delta = abs(nearest.stamp_ns - scan_stamp_ns)
    if delta > tolerance_ns:
        return AssociationResult(False, "pose_tolerance", nearest, delta)
    return AssociationResult(True, "accepted", nearest, delta)
