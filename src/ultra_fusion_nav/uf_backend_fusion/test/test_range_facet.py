import math

import numpy as np

from uf_backend_fusion.range_facet import (
    RangeFacetObservation,
    evaluate_range_facet,
)


def _so3_exp(rotation_vector):
    vector = np.asarray(rotation_vector, dtype=float)
    angle = float(np.linalg.norm(vector))
    if angle < 1e-12:
        return np.eye(3)
    axis = vector / angle
    skew = np.array(
        [
            [0.0, -axis[2], axis[1]],
            [axis[2], 0.0, -axis[0]],
            [-axis[1], axis[0], 0.0],
        ]
    )
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (
        skew @ skew
    )


def _observation(
    *,
    measured=2.0,
    normal=(0.0, 0.0, 1.0),
    offset=-2.0,
    ray=(0.0, 0.0, 1.0),
    stamp=10.0,
    facet_stamp=10.0,
    rmse=0.01,
    dynamic=False,
):
    return RangeFacetObservation(
        stamp_s=stamp,
        measured_range_m=measured,
        ray_direction_sensor=np.asarray(ray, dtype=float),
        sensor_translation_body=np.zeros(3),
        sensor_rotation_body=np.eye(3),
        plane_normal_world=np.asarray(normal, dtype=float),
        plane_offset=offset,
        support_points_world=np.array(
            [[-1.0, -1.0, 2.0], [1.0, -1.0, 2.0], [0.0, 1.0, 2.0]]
        ),
        plane_rmse_m=rmse,
        facet_stamp_s=facet_stamp,
        dynamic=dynamic,
    )


def test_horizontal_facet_is_accepted_and_has_expected_z_jacobian():
    result = evaluate_range_facet(_observation(), np.zeros(3), np.eye(3))
    assert result.accepted
    assert math.isclose(result.predicted_range_m, 2.0, abs_tol=1e-9)
    assert math.isclose(result.residual_m, 0.0, abs_tol=1e-9)
    assert math.isclose(result.pose_jacobian[2], -1.0, abs_tol=1e-9)
    assert np.all(np.isfinite(result.pose_jacobian))


def test_nonhorizontal_facet_uses_ray_plane_intersection_not_body_z():
    observation = _observation(
        measured=1.594896,
        normal=(1.0, 0.0, 1.0),
        offset=-2.0,
        ray=(0.3, 0.0, 0.954),
    )
    result = evaluate_range_facet(observation, np.zeros(3), np.eye(3))
    assert result.accepted
    assert not math.isclose(result.predicted_range_m, 2.0, abs_tol=1e-3)
    assert abs(result.pose_jacobian[0]) > 0.1


def test_covariance_is_propagated_and_huber_weight_is_bounded():
    observation = _observation(measured=1.95)
    observation.plane_covariance = np.eye(4) * 1e-4
    result = evaluate_range_facet(
        observation,
        np.zeros(3),
        np.eye(3),
        state_covariance=np.eye(6) * 1e-4,
        mahalanobis_gate=100.0,
    )
    assert result.accepted
    assert result.measurement_variance_m2 > 0.04 * 0.04
    assert 0.0 < result.robust_weight <= 1.0


def test_pose_jacobian_matches_right_local_finite_difference():
    observation = _observation(measured=1.9)
    observation.sensor_translation_body = np.array([0.2, 0.0, 0.1])
    position = np.zeros(3)
    rotation = np.eye(3)
    nominal = evaluate_range_facet(observation, position, rotation)
    assert nominal.accepted
    numeric = np.zeros(6)
    epsilon = 1e-7
    for axis in range(6):
        perturbed_position = position.copy()
        perturbed_rotation = rotation.copy()
        if axis < 3:
            perturbed_position[axis] += epsilon
        else:
            delta = np.zeros(3)
            delta[axis - 3] = epsilon
            perturbed_rotation = rotation @ _so3_exp(delta)
        perturbed = evaluate_range_facet(
            observation,
            perturbed_position,
            perturbed_rotation,
        )
        assert perturbed.accepted
        numeric[axis] = (
            perturbed.predicted_range_m - nominal.predicted_range_m
        ) / epsilon
    assert np.allclose(nominal.pose_jacobian, numeric, atol=1e-6)


def test_geometry_and_temporal_gates_reject_bad_observations():
    cases = [
        (_observation(ray=(1.0, 0.0, 0.0)), "parallel_facet"),
        (_observation(rmse=0.2), "plane_rmse"),
        (_observation(stamp=10.2), "facet_timestamp"),
        (_observation(dynamic=True), "dynamic_facet"),
        (_observation(measured=20.0), "range_limits"),
    ]
    for observation, reason in cases:
        result = evaluate_range_facet(observation, np.zeros(3), np.eye(3))
        assert not result.accepted
        assert result.reason == reason


def test_mahalanobis_gate_rejects_large_residual():
    result = evaluate_range_facet(
        _observation(measured=1.0),
        np.zeros(3),
        np.eye(3),
        mahalanobis_gate=1.0,
    )
    assert not result.accepted
    assert result.reason == "mahalanobis"
