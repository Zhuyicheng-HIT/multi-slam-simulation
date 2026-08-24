"""Deterministic Native-LiDAR directional reliability evaluation.

This is an offline frozen-normal benchmark.  Scenario truth is used only to
score the detector; it is never passed to reliability or factor shaping.
"""

from dataclasses import dataclass
import math
import resource
import time

import numpy as np

from .native_lidar import (
    NativeLidarPoseNormal,
    lidar_directional_reliability,
)
from .online_backend import axis_information_handoff, subspace_information_handoff


@dataclass(frozen=True)
class DirectionalScenario:
    name: str
    information: np.ndarray | None
    expected_weak_basis: np.ndarray | None
    health_reason: str = "healthy"


def _orthonormal_basis(weak):
    weak = np.asarray(weak, dtype=float)
    weak /= np.linalg.norm(weak)
    seed = np.array([0.0, 0.0, 1.0])
    if abs(float(seed @ weak)) > 0.85:
        seed = np.array([0.0, 1.0, 0.0])
    second = seed - weak * float(seed @ weak)
    second /= np.linalg.norm(second)
    third = np.cross(weak, second)
    return np.column_stack((weak, second, third))


def _information(weak, eigenvalues=(20.0, 180.0, 200.0)):
    basis = _orthonormal_basis(weak)
    return basis @ np.diag(np.asarray(eigenvalues, dtype=float)) @ basis.T


def scenario_matrix():
    xy45 = np.array([1.0, 1.0, 0.0]) / math.sqrt(2.0)
    asymmetric = np.array([0.8, -0.4, 0.4472135955])
    asymmetric /= np.linalg.norm(asymmetric)
    return (
        DirectionalScenario("normal_3d_room", np.diag([180.0, 200.0, 160.0]), None),
        DirectionalScenario("x_axis_corridor", np.diag([20.0, 200.0, 180.0]), np.eye(3)[:, :1]),
        DirectionalScenario("y_axis_corridor", np.diag([200.0, 20.0, 180.0]), np.eye(3)[:, 1:2]),
        DirectionalScenario("corridor_45deg", _information(xy45), xy45[:, None]),
        DirectionalScenario("floor_dominant", np.diag([12.0, 10.0, 200.0]), np.eye(3)[:, :2]),
        DirectionalScenario("z_weak", np.diag([200.0, 180.0, 20.0]), np.eye(3)[:, 2:3]),
        DirectionalScenario("single_wall", np.diag([200.0, 10.0, 12.0]), np.eye(3)[:, 1:]),
        DirectionalScenario(
            "partial_sector_dropout",
            _information([0.9063078, 0.4226183, 0.0], (35.0, 170.0, 200.0)),
            np.asarray([0.9063078, 0.4226183, 0.0])[:, None],
        ),
        DirectionalScenario(
            "asymmetric_occlusion",
            _information(asymmetric, (30.0, 155.0, 200.0)),
            asymmetric[:, None],
        ),
        DirectionalScenario("complete_dropout", None, None, "dropout"),
        DirectionalScenario("stale_lidar", None, None, "stale"),
        DirectionalScenario("timestamp_invalid", None, None, "timestamp_regression_or_future"),
        DirectionalScenario("nonfinite_or_corrupt", None, None, "nonfinite_or_contract_failure"),
    )


def _factor(information, stamp_ns=1_000_000_000):
    pose = np.zeros(6, dtype=float)
    hessian = np.zeros((6, 6), dtype=float)
    hessian[:3, :3] = np.asarray(information, dtype=float)
    hessian[3:, 3:] = np.diag([220.0, 210.0, 190.0])
    return NativeLidarPoseNormal(
        stamp_ns=stamp_ns,
        stamp_s=stamp_ns * 1.0e-9,
        scan_sequence=1,
        reset_counter=0,
        matched_points=200,
        candidate_points=240,
        linearization_pose=pose,
        pose_hessian=hessian,
        pose_gradient=np.zeros(6),
        residual_squared=2.0,
        measurement_variance=1.0,
        source="native_point_to_plane",
        map_frame="camera_init",
        state_frame="body",
        sensor_frame="livox_frame",
        correspondences_valid=True,
        pose_hessian_right=hessian,
        pose_gradient_right=np.zeros(6),
    )


def _subspace_angle_deg(detected, expected_basis):
    detected = np.asarray(detected, dtype=float)
    expected = np.asarray(expected_basis, dtype=float)
    projection = expected @ np.linalg.pinv(expected)
    cosine = np.linalg.norm(projection @ detected) / max(np.linalg.norm(detected), 1.0e-12)
    return math.degrees(math.acos(float(np.clip(cosine, -1.0, 1.0))))


def _solve(information, measurement, alternative_information, alternative_measurement):
    total = information + alternative_information
    return np.linalg.solve(
        total,
        information @ measurement + alternative_information @ alternative_measurement,
    )


def evaluate_scenario(scenario, seed=0):
    if scenario.information is None:
        return {
            "name": scenario.name,
            "source_health": 0.0,
            "admission": "HARD_REJECT",
            "reason": scenario.health_reason,
            "prediction_gate_rejection": 0,
            "factor_admitted": 0,
        }
    started = time.perf_counter_ns()
    factor = _factor(scenario.information)
    evidence = lidar_directional_reliability(factor)
    information = np.asarray(evidence.conditional_information).reshape(3, 3)
    eigenvalues, eigenvectors = np.linalg.eigh(information)
    maximum = max(float(eigenvalues[-1]), 1.0e-12)
    support = np.clip(eigenvalues / maximum, 0.0, 1.0)
    rng = np.random.default_rng(seed + 1701)
    lidar_measurement = rng.normal(0.0, 0.004, 3)
    if scenario.expected_weak_basis is not None:
        weak_bias = np.sum(np.asarray(scenario.expected_weak_basis), axis=1)
        weak_bias /= max(np.linalg.norm(weak_bias), 1.0e-12)
        lidar_measurement += weak_bias * (0.30 + rng.normal(0.0, 0.01))
    alternative_information = np.diag([5.0, 5.0, 5.0])
    alternative_measurement = rng.normal(0.0, 0.08, 3)

    scalar_scale = (
        1.0 if float(np.min(support)) >= 0.30
        else max(1.0e-4, float(np.min(support)))
    )
    scalar_information = information * scalar_scale
    axis_scale, _ = axis_information_handoff(
        np.diag(information),
        np.clip(np.diag(information) / maximum, 0.0, 1.0),
        np.diag(alternative_information),
        np.zeros(3, dtype=bool),
        enter_support=0.30,
        exit_support=0.35,
        minimum_lidar_information_scale=1.0e-4,
    )
    axis_transform = np.diag(np.sqrt(axis_scale))
    axis_information = axis_transform @ information @ axis_transform
    subspace_transform, subspace_scale, directions, _, _ = subspace_information_handoff(
        information,
        alternative_information,
        np.zeros(3, dtype=bool),
        enter_support=0.30,
        exit_support=0.35,
        minimum_lidar_information_scale=1.0e-4,
    )
    subspace_information = subspace_transform @ information @ subspace_transform

    estimates = {
        "scalar": _solve(
            scalar_information, lidar_measurement,
            alternative_information, alternative_measurement,
        ),
        "xyz": _solve(
            axis_information, lidar_measurement,
            alternative_information, alternative_measurement,
        ),
        "subspace": _solve(
            subspace_information, lidar_measurement,
            alternative_information, alternative_measurement,
        ),
    }
    expected = scenario.expected_weak_basis
    if expected is None:
        weak_error = {name: 0.0 for name in estimates}
        strong_error = {name: float(np.linalg.norm(value)) for name, value in estimates.items()}
        angle = None
    else:
        projection = expected @ np.linalg.pinv(expected)
        weak_error = {
            name: float(np.linalg.norm(projection @ value))
            for name, value in estimates.items()
        }
        strong_error = {
            name: float(np.linalg.norm((np.eye(3) - projection) @ value))
            for name, value in estimates.items()
        }
        angle = _subspace_angle_deg(evidence.weakest_direction, expected)
    elapsed_ms = (time.perf_counter_ns() - started) * 1.0e-6
    return {
        "name": scenario.name,
        "source_health": evidence.health,
        "admission": "ADMIT_DIRECTIONAL",
        "reason": "healthy_directional_geometry",
        "factor_admitted": 1,
        "prediction_gate_rejection": 0,
        "matched_points": factor.matched_points,
        "residual_rmse": math.sqrt(factor.residual_squared / factor.matched_points),
        "conditional_eigenvalues": [float(v) for v in eigenvalues],
        "weakest_direction": [float(v) for v in evidence.weakest_direction],
        "weak_direction_angle_deg": angle,
        "reliability_xyz": [float(v) for v in evidence.reliability_xyz],
        "reliability_eigenspace": [float(v) for v in evidence.reliability_eigenspace],
        "axis_information_scale": [float(v) for v in axis_scale],
        "subspace_information_scale": [float(v) for v in subspace_scale],
        "subspace_directions": [[float(v) for v in row] for row in directions],
        "weak_projected_error": weak_error,
        "strong_projected_error": strong_error,
        "error_xyz": {name: [float(v) for v in value] for name, value in estimates.items()},
        "latency_ms": elapsed_ms,
    }


def run_matrix(repeats=5):
    runs = []
    for scenario in scenario_matrix():
        for seed in range(int(repeats)):
            item = evaluate_scenario(scenario, seed)
            item["seed"] = seed
            runs.append(item)
    latency = [item["latency_ms"] for item in runs if "latency_ms" in item]
    rss_mib = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss / 1024.0
    return {
        "contract": {
            "truth_used_online": False,
            "body_envelope_xyz_m": [[-0.45, 0.45], [-0.45, 0.45], [-0.35, 0.15]],
            "axis_handoff_default_enabled": False,
            "subspace_handoff_default_enabled": False,
            "alternative_information_requires_fresh_admitted_factor": True,
        },
        "repeats": int(repeats),
        "runs": runs,
        "runtime": {
            "latency_p50_ms": float(np.percentile(latency, 50)),
            "latency_p95_ms": float(np.percentile(latency, 95)),
            "latency_p99_ms": float(np.percentile(latency, 99)),
            "maximum_rss_mib": float(rss_mib),
        },
    }
