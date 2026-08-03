"""Backend-owned LiDAR scan prediction and deskew interpolation.

The prediction integrates the IMU interval exactly once.  The returned
preintegration object is cached by the online backend and later reused by the
same scan's IMU factor, while the LiDAR front-end only consumes the begin/end
poses required by Ultra-Fusion's intra-scan interpolation model.
"""

from dataclasses import dataclass, replace
import math
from typing import Callable, Sequence

import numpy as np

from .imu_preintegration import (
    ImuSample,
    ManifoldPreintegratedImu,
    preintegrate_manifold,
)
from .manifold_window import propagate_state


@dataclass(frozen=True)
class ScanPrediction:
    valid: bool
    reason: str
    sequence: int
    scan_begin_s: float
    scan_end_s: float
    start_state: np.ndarray
    begin_state: np.ndarray
    end_state: np.ndarray
    measurement: ManifoldPreintegratedImu | None
    quality: float


def scan_request_ready(last_consumed_sequence: int, requested_sequence: int) -> bool:
    """Require frame N-1 to reach a terminal outcome before predicting N.

    Native factors and scan requests use different ROS topics, so DDS delivery
    order cannot distinguish a committed factor from an explicitly rejected one.
    """
    return int(requested_sequence) <= int(last_consumed_sequence) + 1


def scan_request_stale(last_consumed_sequence: int, requested_sequence: int) -> bool:
    """Return true when a retried request already reached a terminal outcome."""
    return int(requested_sequence) <= int(last_consumed_sequence)


def _invalid_prediction(
    reason: str,
    sequence: int,
    scan_begin_s: float,
    scan_end_s: float,
    previous_state,
) -> ScanPrediction:
    state = np.asarray(previous_state, dtype=float).copy()
    return ScanPrediction(
        False,
        reason,
        int(sequence),
        float(scan_begin_s),
        float(scan_end_s),
        state,
        state.copy(),
        state.copy(),
        None,
        0.0,
    )


def _inflate_measurement_for_gap(measurement, nominal_gap_s):
    ratio = float(measurement.max_gap_s) / float(nominal_gap_s)
    if ratio <= 1.0:
        return measurement, 1.0
    inflation = min(25.0, ratio * ratio)
    covariance = np.asarray(measurement.covariance, dtype=float) * inflation
    return (
        replace(
            measurement,
            covariance=tuple(float(value) for value in covariance),
        ),
        max(0.1, 1.0 / ratio),
    )


def build_scan_prediction(
    sequence: int,
    previous_stamp_s: float,
    scan_begin_s: float,
    scan_end_s: float,
    previous_state,
    imu_samples: Sequence[ImuSample],
    *,
    maximum_begin_gap_s: float = 0.02,
    nominal_imu_gap_s: float = 0.10,
    maximum_imu_gap_s: float = 0.30,
    preintegrator: Callable[..., ManifoldPreintegratedImu] = preintegrate_manifold,
) -> ScanPrediction:
    """Predict one LiDAR scan from the last committed backend state.

    The full previous-state-to-scan-end preintegration is returned for the IMU
    factor. If a LiDAR packet was dropped, a partial propagation computes the
    later scan-begin pose for deskew only; it is never added as another factor.
    ``maximum_begin_gap_s`` is the tolerated backwards overlap with the last
    committed state. Positive gaps are valid when the IMU covers them.
    """
    values = np.asarray(
        [
            previous_stamp_s,
            scan_begin_s,
            scan_end_s,
            maximum_begin_gap_s,
            nominal_imu_gap_s,
            maximum_imu_gap_s,
        ],
        dtype=float,
    )
    state = np.asarray(previous_state, dtype=float)
    if state.shape != (15,) or np.any(~np.isfinite(state)):
        return _invalid_prediction(
            "invalid_previous_state", sequence, scan_begin_s, scan_end_s,
            np.zeros(15, dtype=float),
        )
    if np.any(~np.isfinite(values)):
        return _invalid_prediction(
            "nonfinite_timestamp", sequence, scan_begin_s, scan_end_s, state,
        )
    if previous_stamp_s <= 0.0 or scan_begin_s <= 0.0 or scan_end_s <= scan_begin_s:
        return _invalid_prediction(
            "invalid_scan_interval", sequence, scan_begin_s, scan_end_s, state,
        )
    if maximum_begin_gap_s < 0.0:
        return _invalid_prediction(
            "invalid_begin_gap_limit", sequence, scan_begin_s, scan_end_s, state,
        )
    if nominal_imu_gap_s <= 0.0 or maximum_imu_gap_s < nominal_imu_gap_s:
        return _invalid_prediction(
            "invalid_imu_gap_limits", sequence, scan_begin_s, scan_end_s, state,
        )
    begin_gap = float(scan_begin_s) - float(previous_stamp_s)
    if begin_gap < -maximum_begin_gap_s:
        return _invalid_prediction(
            "scan_begin_precedes_last_committed_state",
            sequence,
            scan_begin_s,
            scan_end_s,
            state,
        )

    measurement = preintegrator(
        imu_samples,
        float(previous_stamp_s),
        float(scan_end_s),
        accel_bias=state[9:12],
        gyro_bias=state[12:15],
        max_gap_s=float(maximum_imu_gap_s),
    )
    if not measurement.valid:
        return _invalid_prediction(
            f"imu_{measurement.reason}",
            sequence,
            scan_begin_s,
            scan_end_s,
            state,
        )
    measurement, quality = _inflate_measurement_for_gap(
        measurement, nominal_imu_gap_s
    )
    end_state = propagate_state(state, measurement)
    if np.any(~np.isfinite(end_state)):
        return _invalid_prediction(
            "nonfinite_predicted_state",
            sequence,
            scan_begin_s,
            scan_end_s,
            state,
        )
    begin_state = state.copy()
    if begin_gap > maximum_begin_gap_s:
        begin_measurement = preintegrator(
            imu_samples,
            float(previous_stamp_s),
            float(scan_begin_s),
            accel_bias=state[9:12],
            gyro_bias=state[12:15],
            max_gap_s=float(maximum_imu_gap_s),
        )
        if not begin_measurement.valid:
            return _invalid_prediction(
                f"scan_begin_imu_{begin_measurement.reason}",
                sequence,
                scan_begin_s,
                scan_end_s,
                state,
            )
        begin_state = propagate_state(state, begin_measurement)
        if np.any(~np.isfinite(begin_state)):
            return _invalid_prediction(
                "nonfinite_scan_begin_state",
                sequence,
                scan_begin_s,
                scan_end_s,
                state,
            )
    return ScanPrediction(
        True,
        "ok" if quality >= 1.0 else "imu_gap_degraded",
        int(sequence),
        float(scan_begin_s),
        float(scan_end_s),
        state.copy(),
        np.asarray(begin_state, dtype=float).copy(),
        np.asarray(end_state, dtype=float).copy(),
        measurement,
        quality,
    )


def prediction_reusable(
    prediction: ScanPrediction,
    *,
    sequence: int,
    previous_stamp_s: float,
    scan_end_s: float,
    current_previous_state,
    timestamp_tolerance_s: float = 1.0e-6,
    state_tolerance: float = 1.0e-8,
) -> tuple[bool, str]:
    """Check that a cached IMU delta still starts at the committed state."""
    if not prediction.valid or prediction.measurement is None:
        return False, "prediction_invalid"
    if int(prediction.sequence) != int(sequence):
        return False, "sequence_mismatch"
    if (
        abs(float(prediction.scan_end_s) - float(scan_end_s))
        > timestamp_tolerance_s
        or abs(float(prediction.measurement.dt_s) - (
            float(scan_end_s) - float(previous_stamp_s)
        )) > timestamp_tolerance_s
    ):
        return False, "timestamp_mismatch"
    state = np.asarray(current_previous_state, dtype=float)
    if state.shape != (15,) or np.any(~np.isfinite(state)):
        return False, "current_state_invalid"
    if not np.allclose(
        prediction.start_state,
        state,
        rtol=0.0,
        atol=float(state_tolerance),
    ):
        return False, "start_state_changed"
    return True, "ok"


def consume_cached_prediction(
    predictions: dict[int, ScanPrediction],
    *,
    sequence: int,
    previous_stamp_s: float,
    scan_end_s: float,
    current_previous_state,
    timestamp_tolerance_s: float = 1.0e-6,
    state_tolerance: float = 1.0e-8,
) -> tuple[ScanPrediction | None, str]:
    """Pop and validate the only IMU prediction allowed for one scan.

    Popping before validation is deliberate: a stale or inconsistent prediction
    must never be retried against a later backend state.
    """
    prediction = predictions.pop(int(sequence), None)
    if prediction is None:
        return None, "cache_miss"
    reusable, reason = prediction_reusable(
        prediction,
        sequence=sequence,
        previous_stamp_s=previous_stamp_s,
        scan_end_s=scan_end_s,
        current_previous_state=current_previous_state,
        timestamp_tolerance_s=timestamp_tolerance_s,
        state_tolerance=state_tolerance,
    )
    if not reusable:
        return None, reason
    return prediction, "ok"


def _normalize_quaternion_xyzw(quaternion) -> np.ndarray:
    value = np.asarray(quaternion, dtype=float)
    if value.shape != (4,) or np.any(~np.isfinite(value)):
        raise ValueError("quaternion must contain four finite values")
    norm = float(np.linalg.norm(value))
    if norm <= 1.0e-12:
        raise ValueError("quaternion norm is zero")
    return value / norm


def slerp_quaternion_xyzw(begin, end, alpha: float) -> np.ndarray:
    """Shortest-arc quaternion interpolation with stable small-angle handling."""
    alpha = float(alpha)
    if not math.isfinite(alpha) or alpha < 0.0 or alpha > 1.0:
        raise ValueError("alpha must be finite and within [0, 1]")
    first = _normalize_quaternion_xyzw(begin)
    second = _normalize_quaternion_xyzw(end)
    dot = float(np.dot(first, second))
    if dot < 0.0:
        second = -second
        dot = -dot
    dot = float(np.clip(dot, -1.0, 1.0))
    if dot > 0.9995:
        return _normalize_quaternion_xyzw(first + alpha * (second - first))
    angle = math.acos(dot)
    sin_angle = math.sin(angle)
    return (
        math.sin((1.0 - alpha) * angle) / sin_angle * first
        + math.sin(alpha * angle) / sin_angle * second
    )


def interpolate_scan_pose(
    begin_position,
    begin_orientation_xyzw,
    end_position,
    end_orientation_xyzw,
    alpha: float,
) -> tuple[np.ndarray, np.ndarray]:
    """Apply Ultra-Fusion Eq. (3) to one normalized point time."""
    alpha = float(alpha)
    if not math.isfinite(alpha) or alpha < 0.0 or alpha > 1.0:
        raise ValueError("alpha must be finite and within [0, 1]")
    begin_position = np.asarray(begin_position, dtype=float)
    end_position = np.asarray(end_position, dtype=float)
    if (
        begin_position.shape != (3,)
        or end_position.shape != (3,)
        or np.any(~np.isfinite(begin_position))
        or np.any(~np.isfinite(end_position))
    ):
        raise ValueError("positions must contain three finite values")
    position = (1.0 - alpha) * begin_position + alpha * end_position
    orientation = slerp_quaternion_xyzw(
        begin_orientation_xyzw, end_orientation_xyzw, alpha
    )
    return position, orientation
