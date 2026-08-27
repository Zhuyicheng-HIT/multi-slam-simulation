import hashlib
import json

import pytest

from uf_map_maintenance.manifest import ArchiveValidationError, validate_manifest


def valid_manifest(tmp_path):
    raw = tmp_path / "raw" / "bag.db3"
    raw.parent.mkdir()
    raw.write_bytes(b"immutable raw scan and pose archive")
    return {
        "schema_version": 1,
        "session_id": "tiny",
        "status": "complete",
        "frames": {
            "scan_frame": "livox_frame",
            "pose_parent_frame": "camera_init",
            "pose_child_frame": "body",
        },
        "topics": ["/livox/lidar", "/livox/imu", "/Odometry"],
        "calibration": {
            "translation": [0.0, 0.0, 0.0],
            "quaternion_xyzw": [0.0, 0.0, 0.0, 1.0],
            "direction": "pose_child_from_scan",
            "source": "test",
        },
        "artifacts": [{
            "path": "raw/bag.db3",
            "sha256": hashlib.sha256(raw.read_bytes()).hexdigest(),
            "bytes": raw.stat().st_size,
        }],
    }


def test_complete_manifest_and_hash_are_valid(tmp_path):
    manifest = valid_manifest(tmp_path)
    assert validate_manifest(manifest, tmp_path) == []


def test_missing_required_raw_topic_is_rejected(tmp_path):
    manifest = valid_manifest(tmp_path)
    manifest["topics"].remove("/livox/lidar")
    with pytest.raises(ArchiveValidationError, match="required_topic"):
        validate_manifest(manifest, tmp_path, raise_on_error=True)


def test_nonfinite_calibration_and_hash_mutation_are_rejected(tmp_path):
    manifest = valid_manifest(tmp_path)
    manifest["calibration"]["translation"][0] = float("nan")
    (tmp_path / "raw" / "bag.db3").write_bytes(b"changed")
    errors = validate_manifest(manifest, tmp_path)
    assert "calibration_nonfinite" in errors
    assert "artifact_hash_mismatch:raw/bag.db3" in errors


def test_manifest_json_round_trip_has_no_absolute_paths(tmp_path):
    manifest = valid_manifest(tmp_path)
    encoded = json.dumps(manifest, allow_nan=False, sort_keys=True)
    assert str(tmp_path) not in encoded
