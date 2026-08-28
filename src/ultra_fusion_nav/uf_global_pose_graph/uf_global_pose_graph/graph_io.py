"""Strict file contracts between the offline loop frontend and pose graph."""

import csv
import json
from pathlib import Path

import numpy as np

from .graph import GraphNode, LoopCandidate
from .se3 import pose_matrix


def load_keyframes(path):
    nodes = []
    with Path(path).open(newline="", encoding="utf-8") as stream:
        for row in csv.DictReader(stream):
            nodes.append(GraphNode(
                keyframe_id=int(row["keyframe_id"]),
                scan_id=int(row["scan_id"]),
                stamp_ns=int(row["stamp_ns"]),
                original_pose=pose_matrix(
                    [float(row[name]) for name in ("tx", "ty", "tz")],
                    [float(row[name]) for name in ("qx", "qy", "qz", "qw")],
                ),
            ))
    if len(nodes) < 2:
        raise ValueError("at least two keyframes are required")
    identifiers = [node.keyframe_id for node in nodes]
    stamps = [node.stamp_ns for node in nodes]
    if len(set(identifiers)) != len(identifiers):
        raise ValueError("duplicate keyframe id")
    if any(right <= left for left, right in zip(stamps[:-1], stamps[1:])):
        raise ValueError("keyframe timestamps must be strictly increasing")
    return nodes


def _transform(values):
    matrix = np.asarray(values, dtype=np.float64)
    if matrix.size != 16 or not np.all(np.isfinite(matrix)):
        raise ValueError("loop measurement must be a finite 4x4 transform")
    matrix = matrix.reshape(4, 4)
    if not np.allclose(matrix[3], [0.0, 0.0, 0.0, 1.0], atol=1.0e-8):
        raise ValueError("loop measurement must be a finite 4x4 transform")
    return matrix


def load_loop_candidates(path, nodes):
    payload = json.loads(Path(path).read_text(encoding="utf-8"))
    if payload.get("schema_version") != 2:
        raise ValueError("unsupported loop frontend schema")
    valid_ids = {node.keyframe_id for node in nodes}
    output = []
    for row in payload.get("verified_edges", []):
        measurement = _transform(row.get("candidate_from_query", []))
        candidate_id = int(row["candidate_keyframe"])
        query_id = int(row["query_keyframe"])
        if candidate_id not in valid_ids or query_id not in valid_ids:
            raise ValueError("loop edge references unknown keyframe")
        metrics = np.asarray([
            float(row["descriptor_distance"]),
            float(row["overlap_ratio"]),
            float(row["reciprocal_ratio"]),
            float(row["inlier_rmse_m"]),
            float(row["condition_number"]),
        ])
        if not np.all(np.isfinite(metrics)):
            raise ValueError("loop edge contains nonfinite metrics")
        output.append(LoopCandidate(
            candidate_id=candidate_id,
            query_id=query_id,
            measurement=measurement,
            descriptor_distance=float(row["descriptor_distance"]),
            correspondence_points=int(row["correspondence_points"]),
            overlap_ratio=float(row["overlap_ratio"]),
            reciprocal_ratio=float(row["reciprocal_ratio"]),
            inlier_rmse_m=float(row["inlier_rmse_m"]),
            effective_rank=int(row["effective_rank"]),
            condition_number=float(row["condition_number"]),
        ))
    if not output:
        raise ValueError("loop frontend produced no verified edges")
    return output
