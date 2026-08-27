"""Immutable session-manifest validation for offline map maintenance."""

import argparse
import hashlib
import json
import math
from pathlib import Path


class ArchiveValidationError(ValueError):
    """Raised when an archive violates its declared data contract."""


def sha256_file(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _finite_vector(value, size):
    if not isinstance(value, list) or len(value) != size:
        return False
    return all(
        isinstance(item, (int, float)) and math.isfinite(item)
        for item in value
    )


def validate_manifest(manifest, archive_root, raise_on_error=False):
    """Return stable reason codes for all detected archive-contract violations."""
    root = Path(archive_root).resolve()
    errors = []
    if manifest.get("schema_version") != 1:
        errors.append("schema_version")
    if not manifest.get("session_id"):
        errors.append("session_id")
    if manifest.get("status") != "complete":
        errors.append("archive_incomplete")

    topics = manifest.get("topics", [])
    for topic in ("/livox/lidar", "/livox/imu", "/Odometry"):
        if topic not in topics:
            errors.append("required_topic:" + topic)

    frames = manifest.get("frames", {})
    for name in ("scan_frame", "pose_parent_frame", "pose_child_frame"):
        if not isinstance(frames.get(name), str) or not frames.get(name):
            errors.append("frame_missing:" + name)

    calibration = manifest.get("calibration", {})
    if not _finite_vector(calibration.get("translation"), 3):
        errors.append("calibration_nonfinite")
    quaternion = calibration.get("quaternion_xyzw")
    if not _finite_vector(quaternion, 4) or sum(item * item for item in quaternion or []) < 1e-12:
        errors.append("calibration_quaternion")
    if calibration.get("direction") != "pose_child_from_scan":
        errors.append("calibration_direction")
    if not calibration.get("source"):
        errors.append("calibration_source")

    artifacts = manifest.get("artifacts", [])
    if not isinstance(artifacts, list) or not artifacts:
        errors.append("artifacts_missing")
    for artifact in artifacts if isinstance(artifacts, list) else []:
        relative = Path(str(artifact.get("path", "")))
        if not str(relative) or relative.is_absolute() or ".." in relative.parts:
            errors.append("artifact_path_invalid:" + str(relative))
            continue
        path = root / relative
        try:
            path.resolve().relative_to(root)
        except ValueError:
            errors.append("artifact_path_escape:" + str(relative))
            continue
        if not path.is_file():
            errors.append("artifact_missing:" + str(relative))
            continue
        if artifact.get("bytes") != path.stat().st_size:
            errors.append("artifact_size_mismatch:" + str(relative))
        if artifact.get("sha256") != sha256_file(path):
            errors.append("artifact_hash_mismatch:" + str(relative))

    if raise_on_error and errors:
        raise ArchiveValidationError(",".join(errors))
    return errors


def write_manifest_atomic(manifest, path):
    path = Path(path)
    temporary = path.with_suffix(path.suffix + ".tmp")
    temporary.write_text(
        json.dumps(manifest, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    temporary.replace(path)


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate an immutable map archive")
    parser.add_argument("archive", type=Path)
    arguments = parser.parse_args(argv)
    manifest_path = arguments.archive / "manifest.json"
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    errors = validate_manifest(manifest, arguments.archive)
    print(json.dumps({"valid": not errors, "errors": errors}, sort_keys=True))
    return 0 if not errors else 2
