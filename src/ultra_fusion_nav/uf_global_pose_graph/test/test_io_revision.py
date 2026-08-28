import csv
import hashlib
import json

import numpy as np
import pytest

from uf_global_pose_graph.graph_io import load_keyframes, load_loop_candidates
from uf_global_pose_graph.revision import create_pose_revision
from uf_global_pose_graph.se3 import compose, pose_matrix, se3_exp


POSE_HEADER = [
    "scan_id",
    "stamp_ns",
    "epoch",
    "tx",
    "ty",
    "tz",
    "qx",
    "qy",
    "qz",
    "qw"]
TRAJECTORY_HEADER = [
    "stamp_ns",
    "epoch",
    "tx",
    "ty",
    "tz",
    "qx",
    "qy",
    "qz",
    "qw"]


def _write_csv(path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def _hash(path):
    return hashlib.sha256(path.read_bytes()).hexdigest()


def test_frontend_schema_loads_relative_loop_measurement(tmp_path):
    keyframes = tmp_path / "keyframes.csv"
    _write_csv(
        keyframes,
        ["keyframe_id", *POSE_HEADER, "descriptor_pcd", "map_pcd"],
        [
            [0, 0, 100, 0, 0, 0, 0, 0, 0, 0, 1, "a.pcd", "am.pcd"],
            [1, 1, 200, 0, 1, 0, 0, 0, 0, 0, 1, "b.pcd", "bm.pcd"],
        ],
    )
    measurement = pose_matrix([0.9, 0.1, 0.0], [0.0, 0.0, 0.0, 1.0])
    frontend = tmp_path / "loops.json"
    frontend.write_text(json.dumps({
        "schema_version": 2,
        "verified_edges": [{
            "candidate_keyframe": 0,
            "query_keyframe": 1,
            "candidate_from_query": measurement.reshape(-1).tolist(),
            "descriptor_distance": 0.02,
            "correspondence_points": 800,
            "overlap_ratio": 0.8,
            "reciprocal_ratio": 0.6,
            "inlier_rmse_m": 0.05,
            "effective_rank": 6,
            "condition_number": 30.0,
        }],
    }), encoding="utf-8")

    nodes = load_keyframes(keyframes)
    loops = load_loop_candidates(frontend, nodes)

    assert [node.keyframe_id for node in nodes] == [0, 1]
    assert len(loops) == 1
    np.testing.assert_allclose(loops[0].measurement, measurement)


def test_frontend_schema_rejects_nonfinite_transform(tmp_path):
    frontend = tmp_path / "loops.json"
    frontend.write_text(
        '{"schema_version":2,"verified_edges":[{"candidate_keyframe":0,'
        '"query_keyframe":1,"candidate_from_query":[NaN]}]}',
        encoding="utf-8",
    )

    with pytest.raises(ValueError, match="finite 4x4"):
        load_loop_candidates(frontend, [])


def test_corrected_revision_is_separate_traceable_and_interpolated(tmp_path):
    poses = tmp_path / "original.csv"
    trajectory = tmp_path / "trajectory_original.csv"
    identity = [0, 0, 0, 1]
    _write_csv(poses, POSE_HEADER, [
        [0, 0, 0, 0, 0, 0, *identity],
        [1, 5, 0, 5, 0, 0, *identity],
        [2, 10, 0, 10, 0, 0, *identity],
    ])
    _write_csv(trajectory, TRAJECTORY_HEADER, [
        [0, 0, 0, 0, 0, *identity],
        [5, 0, 5, 0, 0, *identity],
        [10, 0, 10, 0, 0, *identity],
    ])
    nodes = load_keyframes_from_arrays()
    corrected = {
        0: nodes[0].original_pose,
        1: compose(se3_exp([1.0, 0, 0, 0, 0, 0]), nodes[1].original_pose),
    }
    before = (_hash(poses), _hash(trajectory))

    result = create_pose_revision(
        tmp_path / "revision",
        "loop-0001",
        poses,
        trajectory,
        nodes,
        corrected,
        {"loop_frontend_sha256": "abc"},
    )

    assert before == (_hash(poses), _hash(trajectory))
    with result["poses"].open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert float(rows[1]["tx"]) == pytest.approx(5.5)
    assert float(rows[2]["tx"]) == pytest.approx(11.0)
    manifest = json.loads(result["manifest"].read_text(encoding="utf-8"))
    assert manifest["revision_id"] == "loop-0001"
    assert manifest["inputs"]["original_pose_sha256"] == before[0]
    assert manifest["outputs"]["poses_sha256"] == _hash(result["poses"])
    with pytest.raises(FileExistsError):
        create_pose_revision(
            tmp_path / "revision", "loop-0001", poses, trajectory,
            nodes, corrected, {},
        )


def load_keyframes_from_arrays():
    from uf_global_pose_graph.graph import GraphNode

    return [
        GraphNode(0, 0, 0, pose_matrix([0, 0, 0], [0, 0, 0, 1])),
        GraphNode(1, 2, 10, pose_matrix([10, 0, 0], [0, 0, 0, 1])),
    ]
