"""Validated single source of truth for real MID360S/D435i geometry."""

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping, Optional, Sequence, Tuple

import numpy as np
import yaml


class GeometryContractError(ValueError):
    """Raised when hardware geometry is malformed or ambiguous."""


@dataclass(frozen=True)
class Frames:
    body: str
    lidar: str
    camera_optical: str


@dataclass(frozen=True)
class Transform:
    rotation: np.ndarray
    translation: Optional[np.ndarray]
    status: str
    rotation_status: str
    translation_status: str


@dataclass(frozen=True)
class GeometryContract:
    schema_version: int
    frames: Frames
    topics: Mapping[str, str]
    imu_acceleration_scale: float
    body_lidar: Transform
    camera_lidar: Transform
    body_camera: Optional[Transform]
    fast_lio_internal_extrinsic_owned_externally: bool
    body_envelope: Mapping[str, Any]


@dataclass(frozen=True)
class ClosureReport:
    status: str
    missing: Tuple[str, ...]
    translation_residual_m: Optional[float] = None
    rotation_residual_rad: Optional[float] = None


def _finite_vector(values: Sequence[Any], size: int, name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.shape != (size,) or not np.all(np.isfinite(array)):
        raise GeometryContractError(f"{name} must contain {size} finite values")
    return array


def _proper_rotation(values: Sequence[Any], name: str) -> np.ndarray:
    array = np.asarray(values, dtype=float)
    if array.size != 9 or not np.all(np.isfinite(array)):
        raise GeometryContractError(f"{name} must be a proper rotation")
    rotation = array.reshape(3, 3)
    if not np.allclose(rotation.T @ rotation, np.eye(3), atol=1.0e-9):
        raise GeometryContractError(f"{name} must be a proper rotation")
    if not math.isclose(float(np.linalg.det(rotation)), 1.0, abs_tol=1.0e-9):
        raise GeometryContractError(f"{name} must be a proper rotation")
    return rotation


def _quaternion_rotation_xyzw(values: Sequence[Any], name: str) -> np.ndarray:
    quaternion = _finite_vector(values, 4, name)
    norm = float(np.linalg.norm(quaternion))
    if norm <= 1.0e-12:
        raise GeometryContractError(f"{name} must be a non-zero quaternion")
    x, y, z, w = quaternion / norm
    return _proper_rotation([
        1.0 - 2.0 * (y * y + z * z),
        2.0 * (x * y - z * w),
        2.0 * (x * z + y * w),
        2.0 * (x * y + z * w),
        1.0 - 2.0 * (x * x + z * z),
        2.0 * (y * z - x * w),
        2.0 * (x * z - y * w),
        2.0 * (y * z + x * w),
        1.0 - 2.0 * (x * x + y * y),
    ], name)


def _optional_translation(value: Any, name: str) -> Optional[np.ndarray]:
    if value is None:
        return None
    return _finite_vector(value, 3, name)


def _required_mapping(data: Mapping[str, Any], name: str) -> Mapping[str, Any]:
    value = data.get(name)
    if not isinstance(value, Mapping):
        raise GeometryContractError(f"{name} must be a mapping")
    return value


def _derive_body_camera(
    body_lidar: Transform,
    camera_lidar: Transform,
) -> Optional[Transform]:
    """Derive T_body_camera so T_body_camera * T_camera_lidar = T_body_lidar."""
    if body_lidar.translation is None or camera_lidar.translation is None:
        return None
    rotation = body_lidar.rotation @ camera_lidar.rotation.T
    translation = body_lidar.translation - rotation @ camera_lidar.translation
    return Transform(
        rotation=rotation,
        translation=translation,
        status="derived",
        rotation_status="derived_from_body_lidar_and_camera_lidar",
        translation_status="derived_from_body_lidar_and_camera_lidar",
    )


def parse_geometry_contract(data: Mapping[str, Any]) -> GeometryContract:
    if not isinstance(data, Mapping):
        raise GeometryContractError("geometry contract must be a mapping")
    schema_version = int(data.get("schema_version", 0))
    if schema_version != 1:
        raise GeometryContractError("schema_version must be 1")

    frames_data = _required_mapping(data, "frames")
    frames = Frames(
        body=str(frames_data.get("body", "")),
        lidar=str(frames_data.get("lidar", "")),
        camera_optical=str(frames_data.get("camera_optical", "")),
    )
    if not all((frames.body, frames.lidar, frames.camera_optical)):
        raise GeometryContractError("body, lidar, and camera optical frames are required")

    topics_data = _required_mapping(data, "topics")
    topics = {str(key): str(value) for key, value in topics_data.items()}
    for required in ("lidar_raw", "lidar_filtered", "imu_raw", "imu_body"):
        if not topics.get(required):
            raise GeometryContractError(f"topics.{required} is required")

    imu_data = _required_mapping(data, "imu")
    acceleration_scale = float(imu_data.get("acceleration_scale", math.nan))
    if not math.isfinite(acceleration_scale) or acceleration_scale <= 0.0:
        raise GeometryContractError("imu.acceleration_scale must be finite and positive")

    transforms = _required_mapping(data, "transforms")
    body_lidar_data = _required_mapping(transforms, "body_lidar")
    body_lidar = Transform(
        rotation=_proper_rotation(
            body_lidar_data.get("rotation_matrix", []),
            "transforms.body_lidar.rotation_matrix",
        ),
        translation=_optional_translation(
            body_lidar_data.get("translation_m"),
            "transforms.body_lidar.translation_m",
        ),
        status=str(body_lidar_data.get("status", "incomplete")),
        rotation_status=str(body_lidar_data.get("rotation_status", "unknown")),
        translation_status=str(body_lidar_data.get("translation_status", "unknown")),
    )

    camera_lidar_data = _required_mapping(transforms, "camera_lidar")
    camera_translation = _optional_translation(
        camera_lidar_data.get("translation_m"),
        "transforms.camera_lidar.translation_m",
    )
    if camera_translation is None:
        raise GeometryContractError("transforms.camera_lidar.translation_m is required")
    camera_lidar = Transform(
        rotation=_quaternion_rotation_xyzw(
            camera_lidar_data.get("quaternion_xyzw", []),
            "transforms.camera_lidar.quaternion_xyzw",
        ),
        translation=camera_translation,
        status=str(camera_lidar_data.get("status", "unknown")),
        rotation_status=str(camera_lidar_data.get("rotation_status", "calibrated")),
        translation_status=str(camera_lidar_data.get("translation_status", "calibrated")),
    )

    body_camera = None
    body_camera_data = transforms.get("body_camera")
    if body_camera_data is None:
        body_camera = _derive_body_camera(body_lidar, camera_lidar)
    elif isinstance(body_camera_data, Mapping) and body_camera_data.get("derivation") == (
        "T_body_lidar * inverse(T_camera_lidar)"
    ):
        body_camera = _derive_body_camera(body_lidar, camera_lidar)
        if body_camera is None:
            raise GeometryContractError(
                "derived transforms.body_camera requires complete T_body_lidar"
            )
    else:
        if not isinstance(body_camera_data, Mapping):
            raise GeometryContractError("transforms.body_camera must be a mapping or null")
        body_camera_translation = _optional_translation(
            body_camera_data.get("translation_m"),
            "transforms.body_camera.translation_m",
        )
        if body_camera_translation is None:
            raise GeometryContractError("transforms.body_camera.translation_m is required")
        body_camera = Transform(
            rotation=_proper_rotation(
                body_camera_data.get("rotation_matrix", []),
                "transforms.body_camera.rotation_matrix",
            ),
            translation=body_camera_translation,
            status=str(body_camera_data.get("status", "measured")),
            rotation_status=str(body_camera_data.get("rotation_status", "measured")),
            translation_status=str(
                body_camera_data.get("translation_status", "measured")
            ),
        )

    fast_lio = _required_mapping(data, "fast_lio_internal_lidar_imu")
    body_envelope = _required_mapping(data, "body_envelope")
    mode = str(body_envelope.get("mode", ""))
    if mode not in ("legacy_aabb", "composite"):
        raise GeometryContractError("body_envelope.mode must be legacy_aabb or composite")
    primitives = body_envelope.get("primitives", [])
    if not isinstance(primitives, list):
        raise GeometryContractError("body_envelope.primitives must be a list")

    return GeometryContract(
        schema_version=schema_version,
        frames=frames,
        topics=topics,
        imu_acceleration_scale=acceleration_scale,
        body_lidar=body_lidar,
        camera_lidar=camera_lidar,
        body_camera=body_camera,
        fast_lio_internal_extrinsic_owned_externally=bool(
            fast_lio.get("owned_externally", False)
        ),
        body_envelope=dict(body_envelope),
    )


def resolve_geometry_contract_path(path: Any) -> Path:
    text = str(path)
    prefix = "package://"
    if not text.startswith(prefix):
        return Path(text)
    package_and_path = text[len(prefix):]
    package, separator, relative = package_and_path.partition("/")
    if not separator or not package or not relative:
        raise GeometryContractError("invalid package:// geometry contract URI")
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory(package)) / relative


def load_geometry_contract(path: Any) -> GeometryContract:
    with resolve_geometry_contract_path(path).open("r", encoding="utf-8") as stream:
        return parse_geometry_contract(yaml.safe_load(stream))


def imu_parameters(contract: GeometryContract) -> Mapping[str, Any]:
    return {
        "imu_input_topic": contract.topics["imu_raw"],
        "imu_output_topic": contract.topics["imu_body"],
        "imu_acceleration_scale": contract.imu_acceleration_scale,
        "mid360_to_body_rotation": contract.body_lidar.rotation.reshape(-1).tolist(),
        "imu_output_frame_id": contract.frames.body,
    }


def body_filter_parameters(contract: GeometryContract) -> Mapping[str, Any]:
    primitives = list(contract.body_envelope.get("primitives", []))
    geometry_complete = contract.body_lidar.translation is not None and bool(primitives)
    requested_enabled = bool(contract.body_envelope.get("enabled", False))
    parameters = {
        "input_topic": contract.topics["lidar_raw"],
        "output_topic": contract.topics["lidar_filtered"],
        "input_message_type": "livox_custom",
        "geometry_mode": str(contract.body_envelope["mode"]),
        "geometry_complete": geometry_complete,
        "filter_enabled": requested_enabled and geometry_complete,
        "lidar_to_body_rotation": contract.body_lidar.rotation.reshape(-1).tolist(),
        "primitive_count": len(primitives),
    }
    if contract.body_lidar.translation is not None:
        parameters["lidar_to_body_translation"] = contract.body_lidar.translation.tolist()
    return parameters


def closure_report(contract: GeometryContract) -> ClosureReport:
    missing = []
    if contract.body_camera is None:
        missing.append("T_body_camera")
    if contract.body_lidar.translation is None:
        missing.append("t_body_lidar")
    if missing:
        return ClosureReport(status="INCOMPLETE", missing=tuple(missing))

    expected_rotation = contract.body_camera.rotation @ contract.camera_lidar.rotation
    expected_translation = (
        contract.body_camera.rotation @ contract.camera_lidar.translation
        + contract.body_camera.translation
    )
    rotation_delta = expected_rotation.T @ contract.body_lidar.rotation
    cosine = max(-1.0, min(1.0, (float(np.trace(rotation_delta)) - 1.0) * 0.5))
    return ClosureReport(
        status="DERIVED" if contract.body_camera.status == "derived" else "MEASURED",
        missing=(),
        translation_residual_m=float(
            np.linalg.norm(expected_translation - contract.body_lidar.translation)
        ),
        rotation_residual_rad=math.acos(cosine),
    )
