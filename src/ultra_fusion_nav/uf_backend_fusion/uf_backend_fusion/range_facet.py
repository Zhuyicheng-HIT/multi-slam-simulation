"""Geometry-only RangeFacet measurement evaluation.

The evaluator is intentionally independent from the sliding-window backend.
It provides the gates and uncertainty bookkeeping needed before a range
observation is admitted as a factor.
"""

from dataclasses import dataclass
import math
from typing import Optional

import numpy as np


def _skew(v: np.ndarray) -> np.ndarray:
    x, y, z = np.asarray(v, dtype=float)
    return np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])


def _as_vector(value, size: int) -> np.ndarray:
    array = np.asarray(value, dtype=float).reshape(-1)
    if array.size != size or not np.all(np.isfinite(array)):
        raise ValueError("expected finite vector")
    return array


def _as_rotation(value) -> np.ndarray:
    rotation = np.asarray(value, dtype=float).reshape(3, 3)
    if not np.all(np.isfinite(rotation)):
        raise ValueError("expected finite rotation")
    return rotation


@dataclass
class RangeFacetObservation:
    stamp_s: float
    measured_range_m: float
    ray_direction_sensor: np.ndarray
    sensor_translation_body: np.ndarray
    sensor_rotation_body: np.ndarray
    plane_normal_world: np.ndarray
    plane_offset: float
    support_points_world: np.ndarray
    plane_rmse_m: float
    facet_stamp_s: float
    dynamic: bool = False
    plane_covariance: Optional[np.ndarray] = None


@dataclass
class RangeFacetResult:
    accepted: bool
    reason: str
    predicted_range_m: float = float("nan")
    residual_m: float = float("nan")
    pose_jacobian: Optional[np.ndarray] = None
    plane_jacobian: Optional[np.ndarray] = None
    measurement_variance_m2: float = float("nan")
    mahalanobis_sq: float = float("nan")
    robust_weight: float = 0.0
    support_count: int = 0
    intersection_world: Optional[np.ndarray] = None


def _reject(reason: str, support_count: int = 0) -> RangeFacetResult:
    return RangeFacetResult(
        accepted=False,
        reason=reason,
        support_count=support_count,
    )


def _huber_weight(whitened_abs: float, delta: float) -> float:
    if not math.isfinite(whitened_abs):
        return 0.0
    if whitened_abs <= delta:
        return 1.0
    return delta / max(whitened_abs, 1e-12)


def range_facet_prediction_jacobian(
    observation: RangeFacetObservation,
    body_position_world,
    body_rotation_world,
):
    """Return the predicted ray distance and right-local pose Jacobian.

    This deliberately contains no admission gates.  The backend calls it
    after :func:`evaluate_range_facet` has accepted a packet, so LM trial
    states can be evaluated without changing the packet's admission status.
    The pose columns are [world translation, right-local rotation].
    """
    body_position = _as_vector(body_position_world, 3)
    body_rotation = _as_rotation(body_rotation_world)
    ray_sensor = _as_vector(observation.ray_direction_sensor, 3)
    translation_body = _as_vector(observation.sensor_translation_body, 3)
    sensor_rotation = _as_rotation(observation.sensor_rotation_body)
    normal = _as_vector(observation.plane_normal_world, 3)
    ray_norm = float(np.linalg.norm(ray_sensor))
    normal_norm = float(np.linalg.norm(normal))
    if ray_norm <= 1.0e-12 or normal_norm <= 1.0e-12:
        raise ValueError("range facet direction or normal is degenerate")
    ray_sensor = ray_sensor / ray_norm
    normal = normal / normal_norm
    offset = float(observation.plane_offset) / normal_norm
    sensor_position = body_position + body_rotation @ translation_body
    sensor_axis = sensor_rotation @ ray_sensor
    ray_world = body_rotation @ sensor_axis
    denominator = float(normal @ ray_world)
    if abs(denominator) <= 1.0e-12:
        raise ValueError("range facet ray is parallel to plane")
    numerator = float(normal @ sensor_position + offset)
    predicted = -numerator / denominator
    intersection = sensor_position + predicted * ray_world
    position_jacobian = -normal / denominator
    rotation_jacobian = (
        (normal @ body_rotation @ _skew(translation_body)) / denominator
        - numerator
        * (normal @ body_rotation @ _skew(sensor_axis))
        / (denominator * denominator)
    )
    return float(predicted), np.concatenate(
        (position_jacobian, rotation_jacobian)
    ), intersection


def evaluate_range_facet(
    observation: RangeFacetObservation,
    body_position_world,
    body_rotation_world,
    *,
    range_sigma_m: float = 0.04,
    min_range_m: float = 0.02,
    max_range_m: float = 12.0,
    minimum_support_points: int = 3,
    maximum_plane_rmse_m: float = 0.05,
    denominator_epsilon: float = 0.05,
    facet_margin_m: float = 0.20,
    timestamp_tolerance_s: float = 0.05,
    state_covariance: Optional[np.ndarray] = None,
    mahalanobis_gate: float = 9.0,
    huber_delta: float = 2.5,
) -> RangeFacetResult:
    """Evaluate one sensor-ray/plane-facet range observation.

    Pose perturbations are right-local [translation, rotation] perturbations.
    The returned Jacobian is for predicted_range - measured_range.
    """
    support = np.asarray(observation.support_points_world, dtype=float)
    if support.ndim != 2 or support.shape[1] != 3:
        return _reject("invalid_support_points")
    support_count = int(support.shape[0])
    if support_count < minimum_support_points:
        return _reject("insufficient_support", support_count)
    if not np.all(np.isfinite(support)):
        return _reject("invalid_support_points", support_count)
    if observation.dynamic:
        return _reject("dynamic_facet", support_count)
    if not math.isfinite(observation.plane_rmse_m):
        return _reject("invalid_plane_rmse", support_count)
    if observation.plane_rmse_m > maximum_plane_rmse_m:
        return _reject("plane_rmse", support_count)
    if (
        not math.isfinite(observation.stamp_s)
        or not math.isfinite(observation.facet_stamp_s)
        or abs(observation.stamp_s - observation.facet_stamp_s)
        > timestamp_tolerance_s
    ):
        return _reject("facet_timestamp", support_count)
    measured = float(observation.measured_range_m)
    if not math.isfinite(measured) or not (min_range_m <= measured <= max_range_m):
        return _reject("range_limits", support_count)
    if not math.isfinite(range_sigma_m) or range_sigma_m <= 0.0:
        return _reject("invalid_range_sigma", support_count)

    try:
        body_position = _as_vector(body_position_world, 3)
        body_rotation = _as_rotation(body_rotation_world)
        ray_sensor = _as_vector(observation.ray_direction_sensor, 3)
        translation_body = _as_vector(observation.sensor_translation_body, 3)
        sensor_rotation = _as_rotation(observation.sensor_rotation_body)
        normal = _as_vector(observation.plane_normal_world, 3)
    except (ValueError, TypeError):
        return _reject("invalid_geometry", support_count)
    ray_norm = float(np.linalg.norm(ray_sensor))
    normal_norm = float(np.linalg.norm(normal))
    if ray_norm <= 1e-12 or normal_norm <= 1e-12:
        return _reject("invalid_geometry", support_count)
    ray_sensor /= ray_norm
    normal /= normal_norm
    offset = float(observation.plane_offset) / normal_norm

    sensor_position = body_position + body_rotation @ translation_body
    sensor_axis = sensor_rotation @ ray_sensor
    ray_world = body_rotation @ sensor_axis
    denominator = float(normal @ ray_world)
    if not math.isfinite(denominator) or abs(denominator) <= denominator_epsilon:
        return _reject("parallel_facet", support_count)
    numerator = float(normal @ sensor_position + offset)
    predicted = -numerator / denominator
    if not math.isfinite(predicted) or predicted <= 0.0:
        return _reject("nonpositive_intersection", support_count)
    intersection = sensor_position + predicted * ray_world

    centroid = np.mean(support, axis=0)
    support_radius = float(np.max(np.linalg.norm(support - centroid, axis=1)))
    if float(np.linalg.norm(intersection - centroid)) > support_radius + facet_margin_m:
        return _reject("outside_facet", support_count)

    # Right-local pose Jacobian. Translation is expressed in world coordinates.
    position_jacobian = -normal / denominator
    rotation_jacobian = (
        (normal @ body_rotation @ _skew(translation_body)) / denominator
        - numerator
        * (normal @ body_rotation @ _skew(sensor_axis))
        / (denominator * denominator)
    )
    pose_jacobian = np.concatenate((position_jacobian, rotation_jacobian))

    # Plane parameter Jacobian for [normal(3), offset].
    plane_jacobian = np.concatenate(
        (
            -sensor_position / denominator
            + numerator * ray_world / (denominator * denominator),
            np.array([-1.0 / denominator]),
        )
    )
    variance = range_sigma_m * range_sigma_m
    if observation.plane_covariance is not None:
        plane_covariance = np.asarray(observation.plane_covariance, dtype=float)
        if plane_covariance.shape != (4, 4) or not np.all(
            np.isfinite(plane_covariance)
        ):
            return _reject("invalid_plane_covariance", support_count)
        variance += float(plane_jacobian @ plane_covariance @ plane_jacobian)
    if state_covariance is not None:
        pose_covariance = np.asarray(state_covariance, dtype=float)
        if pose_covariance.shape != (6, 6) or not np.all(
            np.isfinite(pose_covariance)
        ):
            return _reject("invalid_state_covariance", support_count)
        variance += float(pose_jacobian @ pose_covariance @ pose_jacobian)
    if not math.isfinite(variance) or variance <= 0.0:
        return _reject("invalid_variance", support_count)

    residual = predicted - measured
    mahalanobis_sq = residual * residual / variance
    if not math.isfinite(mahalanobis_sq):
        return _reject("invalid_mahalanobis", support_count)
    if mahalanobis_sq > mahalanobis_gate:
        return RangeFacetResult(
            accepted=False,
            reason="mahalanobis",
            predicted_range_m=predicted,
            residual_m=residual,
            pose_jacobian=pose_jacobian,
            plane_jacobian=plane_jacobian,
            measurement_variance_m2=variance,
            mahalanobis_sq=mahalanobis_sq,
            robust_weight=0.0,
            support_count=support_count,
            intersection_world=intersection,
        )
    return RangeFacetResult(
        accepted=True,
        reason="accepted",
        predicted_range_m=predicted,
        residual_m=residual,
        pose_jacobian=pose_jacobian,
        plane_jacobian=plane_jacobian,
        measurement_variance_m2=variance,
        mahalanobis_sq=mahalanobis_sq,
        robust_weight=_huber_weight(math.sqrt(mahalanobis_sq), huber_delta),
        support_count=support_count,
        intersection_world=intersection,
    )
