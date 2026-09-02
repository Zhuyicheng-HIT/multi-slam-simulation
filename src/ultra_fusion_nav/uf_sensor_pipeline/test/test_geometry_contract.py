import math
import json
from pathlib import Path

import numpy as np
import pytest
import yaml

from uf_sensor_pipeline.geometry_contract import (
    GeometryContractError,
    body_filter_parameters,
    closure_report,
    imu_parameters,
    load_geometry_contract,
    parse_geometry_contract,
)


def test_geometry_contract_cli_reports_coordinate_defined_closed_chain(capsys):
    from uf_sensor_pipeline.geometry_contract_check import main

    assert main([str(CONFIG), "--json"]) == 0
    report = json.loads(capsys.readouterr().out)
    assert report["closure"]["status"] == "DERIVED"
    assert report["closure"]["missing"] == []
    assert report["body_lidar_translation_available"] is True
    assert report["body_lidar_translation_measured"] is False
    assert report["body_lidar_translation_status"] == "coordinate_definition"
    assert report["hardware_tf_publishable"] is True
    assert main([str(CONFIG), "--require-complete"]) == 0


CONFIG = Path(__file__).resolve().parents[1] / "config" / "real_mid360s_d435i_geometry.yaml"


def test_real_contract_uses_exact_positive_fifteen_degree_body_from_lidar_rotation():
    contract = load_geometry_contract(CONFIG)
    angle = math.radians(15.0)
    expected = np.asarray([
        [math.cos(angle), 0.0, math.sin(angle)],
        [0.0, 1.0, 0.0],
        [-math.sin(angle), 0.0, math.cos(angle)],
    ])

    assert contract.frames.body == "base_link"
    assert contract.frames.lidar == "livox_frame"
    assert contract.body_lidar.rotation_status == "nominal"
    assert np.allclose(contract.body_lidar.rotation, expected, atol=1.0e-12)
    assert contract.body_lidar.status == "coordinate_defined"
    assert contract.body_lidar.translation_status == "coordinate_definition"
    assert np.array_equal(contract.body_lidar.translation, np.zeros(3))
    assert contract.fast_lio_internal_extrinsic_owned_externally is True


def test_static_level_gravity_rotates_from_sensor_g_to_body_si():
    contract = load_geometry_contract(CONFIG)
    angle = math.radians(15.0)
    raw_specific_force_g = np.asarray([-math.sin(angle), 0.0, math.cos(angle)])

    body_specific_force = (
        contract.body_lidar.rotation @ raw_specific_force_g
        * contract.imu_acceleration_scale
    )

    assert np.allclose(body_specific_force, [0.0, 0.0, 9.80665], atol=1.0e-9)


def test_camera_lidar_calibration_is_proper_and_inverse_round_trip_is_exact():
    contract = load_geometry_contract(CONFIG)
    transform = contract.camera_lidar

    assert transform.status == "calibrated"
    assert np.allclose(
        transform.translation,
        [0.063705566722, -0.170242628857, -0.008687984507],
        atol=1.0e-12,
    )
    assert np.allclose(transform.rotation.T @ transform.rotation, np.eye(3), atol=1.0e-12)
    assert np.isclose(np.linalg.det(transform.rotation), 1.0, atol=1.0e-12)

    point_lidar = np.asarray([1.2, -0.4, 2.5])
    point_camera = transform.rotation @ point_lidar + transform.translation
    point_lidar_round_trip = transform.rotation.T @ (
        point_camera - transform.translation
    )
    assert np.allclose(point_lidar_round_trip, point_lidar, atol=1.0e-12)


def test_camera_lidar_calibration_provenance_and_reported_accuracy_are_preserved():
    data = yaml.safe_load(CONFIG.read_text(encoding="utf-8"))

    assert data["provenance"]["calibration_document"] == (
        "MID360S_D435i_外参标定结果.docx"
    )
    assert data["provenance"]["calibration_document_sha256"] == (
        "5e6e1401f26eb369b8d7c3132b3a5c6c777f44d1f44d6bce93b938b1dd28160c"
    )
    calibration = data["transforms"]["camera_lidar"]
    assert calibration["calibration_rmse_m"] == 0.0303
    assert calibration["calibration_max_residual_m"] == 0.0593


def test_camera_mount_is_derived_from_calibration_and_closes_body_lidar_chain():
    contract = load_geometry_contract(CONFIG)

    report = closure_report(contract)
    body_parameters = body_filter_parameters(contract)

    assert contract.body_camera is not None
    assert contract.body_camera.status == "derived"
    assert report.status == "DERIVED"
    assert report.missing == ()
    assert report.translation_residual_m < 1.0e-12
    assert report.rotation_residual_rad < 1.0e-12
    assert np.allclose(
        contract.body_camera.rotation @ contract.camera_lidar.rotation,
        contract.body_lidar.rotation,
        atol=1.0e-12,
    )
    assert np.allclose(
        contract.body_camera.rotation @ contract.camera_lidar.translation
        + contract.body_camera.translation,
        contract.body_lidar.translation,
        atol=1.0e-12,
    )
    assert body_parameters["lidar_to_body_translation"] == [0.0, 0.0, 0.0]
    assert body_parameters["filter_enabled"] is True
    assert body_parameters["geometry_complete"] is True
    assert body_parameters["primitive_count"] == 1
    assert body_parameters["input_message_type"] == "livox_custom"


def test_provisional_envelope_has_exact_approved_body_flu_bounds():
    contract = load_geometry_contract(CONFIG)
    primitives = contract.body_envelope["primitives"]

    assert contract.body_envelope["status"] == "provisional_conservative_envelope"
    assert contract.body_envelope["fail_open"] is True
    assert len(primitives) == 1
    box = primitives[0]
    assert box["type"] == "box"
    assert box["center_body_m"] == [0.0, 0.0, -0.12]
    assert box["dimensions_m"] == [0.56, 0.56, 0.36]
    assert box["padding_m"] == 0.0


def test_contract_generates_one_production_imu_topic_and_no_lidar_imu_alias():
    parameters = imu_parameters(load_geometry_contract(CONFIG))

    assert parameters["imu_input_topic"] == "/livox/imu"
    assert parameters["imu_output_topic"] == "/sensors/imu"
    assert parameters["imu_output_frame_id"] == "base_link"
    assert parameters["imu_acceleration_scale"] == 9.80665
    assert "/livox/lidar_imu" not in parameters.values()


def test_malformed_rotation_is_rejected_before_any_consumer_can_start():
    data = {
        "schema_version": 1,
        "frames": {
            "body": "base_link",
            "lidar": "livox_frame",
            "camera_optical": "d435i_color_optical_frame",
        },
        "topics": {
            "lidar_raw": "/livox/lidar",
            "lidar_filtered": "/sensors/lidar/livox_body_filtered",
            "imu_raw": "/livox/imu",
            "imu_body": "/sensors/imu",
        },
        "imu": {"acceleration_scale": 9.80665},
        "transforms": {
            "body_lidar": {
                "rotation_status": "nominal",
                "rotation_matrix": [2.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
                "translation_status": "unmeasured",
                "translation_m": None,
            },
            "camera_lidar": {
                "status": "calibrated",
                "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
                "translation_m": [0.0, 0.0, 0.0],
            },
        },
        "fast_lio_internal_lidar_imu": {"owned_externally": True},
        "body_envelope": {"enabled": False, "mode": "composite", "primitives": []},
    }

    with pytest.raises(GeometryContractError, match="proper rotation"):
        parse_geometry_contract(data)
