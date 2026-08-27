"""Stream a raw MID360/pose session directly to rosbag2 on disk."""

import argparse
from datetime import datetime, timezone
import json
from pathlib import Path
import signal
import subprocess

import yaml

from .manifest import sha256_file, write_manifest_atomic


DEFAULT_TOPICS = [
    "/livox/lidar",
    "/livox/imu",
    "/Odometry",
    "/fusion/unified/odom",
    "/fusion/epoch",
    "/tf",
    "/tf_static",
]


def bag_record_command(raw_directory, topics, storage_id="sqlite3"):
    return [
        "ros2", "bag", "record", "--storage", storage_id,
        "--output", str(raw_directory), *topics,
    ]


def _artifacts(root):
    return [
        {
            "path": str(path.relative_to(root)),
            "sha256": sha256_file(path),
            "bytes": path.stat().st_size,
        }
        for path in sorted((root / "raw").rglob("*"))
        if path.is_file()
    ]


def main(argv=None):
    parser = argparse.ArgumentParser(description="Record immutable raw MID360 and pose history")
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--session-id", required=True)
    parser.add_argument("--config", required=True, type=Path)
    parser.add_argument("--dry-run", action="store_true")
    arguments = parser.parse_args(argv)
    if arguments.output.exists() and any(arguments.output.iterdir()):
        raise SystemExit("archive output must be new or empty")
    arguments.output.mkdir(parents=True, exist_ok=True)
    configuration = yaml.safe_load(arguments.config.read_text(encoding="utf-8"))
    values = configuration["archive"]
    frames = configuration["frames"]
    calibration = configuration["calibration"]
    topics = values.get("raw_topics", DEFAULT_TOPICS)
    storage_id = values.get("storage_id", "sqlite3")
    manifest = {
        "schema_version": 1,
        "session_id": arguments.session_id,
        "status": "planned" if arguments.dry_run else "recording",
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "frames": frames,
        "topics": topics,
        "calibration": calibration,
        "artifacts": [],
        "materialization": {
            "scan_cache": "scans/<scan_id>.npz",
            "original_pose_revision": "poses/original.csv",
            "corrected_pose_revision": "poses/<revision>.csv",
            "raw_bag_is_authoritative": True,
        },
    }
    write_manifest_atomic(manifest, arguments.output / "manifest.json")
    command = bag_record_command(arguments.output / "raw", topics, storage_id)
    if arguments.dry_run:
        print(json.dumps({"command": command, "manifest": manifest}, sort_keys=True))
        return 0

    process = subprocess.Popen(command)
    try:
        return_code = process.wait()
    except KeyboardInterrupt:
        process.send_signal(signal.SIGINT)
        return_code = process.wait()
    if return_code != 0:
        manifest["status"] = "failed"
        manifest["recorder_exit_code"] = return_code
        write_manifest_atomic(manifest, arguments.output / "manifest.json")
        return return_code
    manifest["status"] = "complete"
    manifest["artifacts"] = _artifacts(arguments.output)
    write_manifest_atomic(manifest, arguments.output / "manifest.json")
    return 0
