from pathlib import Path
from types import SimpleNamespace

import numpy as np

from uf_sensor_pipeline.fault_profiles import (
    load_fault_profile,
    profile_backend_overrides,
)
from uf_sensor_pipeline.robustness_fault_injector import (
    RobustnessFaultInjector,
    quaternion_multiply_xyzw,
    quaternion_rotation_xyzw,
)


PROFILE_PATH = (
    Path(__file__).resolve().parents[1] / "config" / "robustness_v3_profiles.yaml"
)


def test_every_profile_loads_and_has_unique_supported_faults():
    import yaml

    rows = yaml.safe_load(PROFILE_PATH.read_text(encoding="utf-8"))["profiles"]
    for name in rows:
        profile = load_fault_profile(str(PROFILE_PATH), name)
        assert profile.name == name
        assert len({id(spec) for spec in profile.faults}) == len(profile.faults)


def test_double_fault_profile_expands_both_modalities():
    profile = load_fault_profile(str(PROFILE_PATH), "dual_visual_gnss_medium")
    assert {spec.modality for spec in profile.faults} == {"vision", "gnss"}
    assert len(profile.faults) == 3


def test_calibration_overrides_are_documented_backend_parameters():
    profile = load_fault_profile(str(PROFILE_PATH), "d435_extrinsic_rot_medium")
    overrides = profile_backend_overrides(profile)
    rotation = np.asarray(overrides["visual_rotation_body_camera"]).reshape(3, 3)
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-5)
    assert np.isclose(np.linalg.det(rotation), 1.0, atol=1.0e-5)


def test_quaternion_extrinsic_perturbation_is_normalized():
    half = np.deg2rad(3.0) * 0.5
    result = quaternion_multiply_xyzw(
        [0.0, 0.0, np.sin(half), np.cos(half)],
        [0.0, 0.0, 0.0, 1.0],
    )
    assert np.isclose(np.linalg.norm(result), 1.0)
    assert np.isclose(result[2], np.sin(half))


def test_quaternion_rotation_is_proper_and_rotates_x_to_y():
    half = np.deg2rad(90.0) * 0.5
    rotation = quaternion_rotation_xyzw(
        [0.0, 0.0, np.sin(half), np.cos(half)]
    )
    assert np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-12)
    assert np.allclose(rotation @ [1.0, 0.0, 0.0], [0.0, 1.0, 0.0])


def test_correspondence_row_selection_preserves_row_order():
    values = list(range(18))
    selected = RobustnessFaultInjector._select_rows(
        values, 3, np.asarray([1, 4])
    )
    assert selected == [3, 4, 5, 12, 13, 14]


def test_native_relinearization_updates_pose_normal_from_raw_rows():
    points = np.asarray([
        [1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0], [1.0, 1.0, 0.0],
    ])
    normals = np.asarray([
        [1.0, 0.0, 0.0], [0.0, 1.0, 0.0],
        [0.0, 0.0, 1.0], [1.0, 0.0, 0.0],
    ])
    message = SimpleNamespace(
        matched_points=4,
        lidar_points_xyz=points.reshape(-1).tolist(),
        plane_normals_xyz=normals.reshape(-1).tolist(),
        plane_points_xyz=(points - 0.1 * normals).reshape(-1).tolist(),
        lidar_to_body_quaternion=[0.0, 0.0, 0.0, 1.0],
        lidar_to_body_translation=[0.0, 0.0, 0.0],
        linearization_quaternion=[0.0, 0.0, 0.0, 1.0],
        linearization_position=[0.0, 0.0, 0.0],
        residuals=[9.0] * 4,
        state_hessian=np.zeros((12, 12)).reshape(-1).tolist(),
        state_gradient=np.zeros(12).tolist(),
    )
    output = RobustnessFaultInjector._relinearize_native_message(message)
    hessian = np.asarray(output.state_hessian).reshape(12, 12)
    assert np.allclose(output.residuals, [0.1] * 4)
    assert np.linalg.matrix_rank(hessian[:6, :6]) >= 3
    assert np.all(np.isfinite(hessian))
