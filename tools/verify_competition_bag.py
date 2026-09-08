#!/usr/bin/env python3
"""Verify rosbag metadata against one competition-mode evidence contract."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
from typing import Any

import yaml


COMMON_REQUIRED = {
    "/livox/lidar",
    "/livox/imu",
    "/sensors/optical_flow/rad",
    "/fusion/unified/odom",
    "/competition/event",
}
MODE_REQUIRED = {
    "metric1": COMMON_REQUIRED | {"/fast_lio/native_lidar_factor"},
    "metric2-full": COMMON_REQUIRED
    | {
        "/sensors/gnss/fix",
        "/sensors/rgbd/color",
        "/sensors/rgbd/depth",
        "/dynamic_observer/dynamic_candidates",
        "/dynamic_observer/statistics",
    },
    "metric2-no-camera": COMMON_REQUIRED
    | {
        "/sensors/gnss/fix",
        "/dynamic_observer/dynamic_candidates",
        "/dynamic_observer/statistics",
    },
    "metric3": COMMON_REQUIRED
    | {
        "/sensors/gnss/fix",
        "/sensors/rgbd/color",
        "/sensors/rgbd/depth",
        "/relocalization/result",
        "/fusion/unified/epoch",
        "/safety/active_relocalization_status",
    },
    "metric3-four-source": COMMON_REQUIRED
    | {
        "/sensors/gnss/fix",
        "/relocalization/result",
        "/fusion/unified/epoch",
        "/safety/active_relocalization_status",
    },
    "metric3-five-source": COMMON_REQUIRED
    | {
        "/sensors/gnss/fix",
        "/sensors/rgbd/color",
        "/sensors/rgbd/depth",
        "/relocalization/result",
        "/fusion/unified/epoch",
        "/safety/active_relocalization_status",
    },
}
MODE_FORBIDDEN = {
    "metric1": {"/sensors/gnss/fix", "/sensors/rgbd/color", "/sensors/rgbd/depth"},
    "metric2-full": set(),
    "metric2-no-camera": {"/sensors/rgbd/color", "/sensors/rgbd/depth"},
    "metric3": set(),
    "metric3-four-source": {"/sensors/rgbd/color", "/sensors/rgbd/depth"},
    "metric3-five-source": set(),
}


def topic_counts(metadata: dict[str, Any]) -> dict[str, int]:
    info = metadata.get("rosbag2_bagfile_information", metadata)
    result: dict[str, int] = {}
    for item in info.get("topics_with_message_count", []):
        topic = item.get("topic_metadata", {}).get("name", "")
        if topic:
            result[topic] = int(item.get("message_count", 0))
    return result


def build_report(mode: str, metadata: dict[str, Any]) -> dict[str, Any]:
    if mode not in MODE_REQUIRED:
        raise ValueError(f"unknown mode: {mode}")
    counts = topic_counts(metadata)
    missing = sorted(topic for topic in MODE_REQUIRED[mode] if counts.get(topic, 0) <= 0)
    unexpected = sorted(topic for topic in MODE_FORBIDDEN[mode] if counts.get(topic, 0) > 0)
    return {
        "schema": "competition_bag_validation_v1",
        "mode": mode,
        "valid": not missing and not unexpected,
        "missing_or_empty_required_topics": missing,
        "unexpected_active_topics": unexpected,
        "topic_message_counts": dict(sorted(counts.items())),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("mode", choices=sorted(MODE_REQUIRED))
    parser.add_argument("metadata", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    metadata = yaml.safe_load(args.metadata.read_text(encoding="utf-8"))
    report = build_report(args.mode, metadata)
    rendered = json.dumps(report, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.write_text(rendered, encoding="utf-8")
    print(rendered, end="")
    return 0 if report["valid"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
