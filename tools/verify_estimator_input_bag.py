#!/usr/bin/env python3
"""Reject estimator replay bags that omit a required sensor contract."""

import argparse
import json
from pathlib import Path

import yaml


CORE_TOPICS = (
    "/clock",
    "/fast_lio/native_lidar_factor",
    "/sensors/imu",
    "/sensors/gnss/fix",
    "/sensors/gnss/raw",
    "/sensors/optical_flow/rad",
    "/reliability/scheduler_state",
)
FRONTEND_SCAN_REQUEST_TOPIC = "/fast_lio/frontend_scan_request"
VISUAL_TOPICS = (
    "/vision/feature_tracks",
    "/reliability/vision_score",
)
VISUAL_FACTOR_SCORE_TOPIC = "/reliability/vision_factor_score"
RGBD_GEOMETRY_TOPIC = "/vision/rgbd_geometry_tracks"
RGBD_DIRECT_TOPIC = "/vision/rgbd_direct_tracks"


def topic_counts(metadata):
    information = metadata.get("rosbag2_bagfile_information", {})
    rows = information.get("topics_with_message_count", ())
    return {
        str(row.get("topic_metadata", {}).get("name", "")): int(
            row.get("message_count", 0)
        )
        for row in rows
    }


def build_report(
    metadata,
    require_visual=False,
    require_visual_factor_score=False,
    require_rgbd_geometry=False,
    require_rgbd_direct=False,
    require_frontend_scan_request=False,
):
    counts = topic_counts(metadata)
    required = list(CORE_TOPICS)
    if require_visual:
        required.extend(VISUAL_TOPICS)
    if require_visual_factor_score:
        required.append(VISUAL_FACTOR_SCORE_TOPIC)
    if require_rgbd_geometry:
        required.append(RGBD_GEOMETRY_TOPIC)
    if require_rgbd_direct:
        required.append(RGBD_DIRECT_TOPIC)
    if require_frontend_scan_request:
        required.append(FRONTEND_SCAN_REQUEST_TOPIC)
    missing = [topic for topic in required if counts.get(topic, 0) <= 0]
    return {
        "valid": not missing,
        "required_topics": required,
        "missing_or_empty_topics": missing,
        "message_counts": counts,
        "native_factor_count": counts.get(
            "/fast_lio/native_lidar_factor", 0
        ),
        "frontend_scan_request_count": counts.get(
            FRONTEND_SCAN_REQUEST_TOPIC, 0
        ),
        "reference_unified_odom_count": counts.get(
            "/fusion/unified/odom", 0
        ),
        "visual_factor_score_count": counts.get(
            VISUAL_FACTOR_SCORE_TOPIC, 0
        ),
        "rgbd_geometry_count": counts.get(RGBD_GEOMETRY_TOPIC, 0),
        "rgbd_direct_count": counts.get(RGBD_DIRECT_TOPIC, 0),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--bag", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--require-visual", action="store_true")
    parser.add_argument(
        "--require-frontend-scan-request",
        action="store_true",
        help=(
            "Require the optional frontend prediction handshake. Native-factor "
            "replays do not need this topic unless handshake behavior itself "
            "is under test."
        ),
    )
    parser.add_argument(
        "--require-visual-factor-score",
        action="store_true",
        help=(
            "Require the candidate-level visual factor score used by the "
            "paper_reprojection backend. This is intentionally separate from "
            "the sensor-level /reliability/vision_score contract."
        ),
    )
    parser.add_argument(
        "--require-rgbd-geometry",
        action="store_true",
        help=(
            "Require timestamped RGB-D depth geometry used by the sparse "
            "metric depth factor."
        ),
    )
    parser.add_argument(
        "--require-rgbd-direct",
        action="store_true",
        help=(
            "Require timestamped RGB-D depth and photometric tracks used by "
            "the direct visual factor."
        ),
    )
    args = parser.parse_args()

    bag = Path(args.bag)
    metadata_path = bag / "metadata.yaml"
    if not metadata_path.is_file():
        raise SystemExit(f"rosbag metadata is missing: {metadata_path}")
    with metadata_path.open("r", encoding="utf-8") as stream:
        metadata = yaml.safe_load(stream) or {}
    report = build_report(
        metadata,
        require_visual=args.require_visual,
        require_visual_factor_score=args.require_visual_factor_score,
        require_rgbd_geometry=args.require_rgbd_geometry,
        require_rgbd_direct=args.require_rgbd_direct,
        require_frontend_scan_request=args.require_frontend_scan_request,
    )
    report["bag"] = str(bag)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(report, indent=2, sort_keys=True))
    return 0 if report["valid"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
