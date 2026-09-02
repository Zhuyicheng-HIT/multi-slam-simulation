import csv
import json

from uf_global_pose_graph.pipeline import PipelineConfig, run_global_pose_graph


def _write_csv(path, header, rows):
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(header)
        writer.writerows(rows)


def test_pipeline_writes_audited_graph_and_versioned_pose_revision(tmp_path):
    keyframes = tmp_path / "keyframes.csv"
    identity = [0, 0, 0, 1]
    keyframe_rows = []
    for index, x in enumerate([0.0, 1.0, 2.0, 3.1]):
        keyframe_rows.append([
            index, index, index * 10, 0, x, 0, 0, *identity,
            f"d{index}.pcd", f"m{index}.pcd",
        ])
    _write_csv(
        keyframes,
        ["keyframe_id", "scan_id", "stamp_ns", "epoch", "tx", "ty", "tz",
         "qx", "qy", "qz", "qw", "descriptor_pcd", "map_pcd"],
        keyframe_rows,
    )
    loop_frontend = tmp_path / "loops.json"
    measurement = [1, 0, 0, 3, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1]
    loop_frontend.write_text(json.dumps({
        "schema_version": 2,
        "verified_edges": [{
            "candidate_keyframe": 0, "query_keyframe": 3,
            "candidate_from_query": measurement,
            "descriptor_distance": 0.01, "correspondence_points": 1000,
            "overlap_ratio": 0.9, "reciprocal_ratio": 0.8,
            "inlier_rmse_m": 0.04, "effective_rank": 6,
            "condition_number": 20.0,
        }],
    }), encoding="utf-8")
    session = tmp_path / "session"
    (session / "poses").mkdir(parents=True)
    pose_rows = [[i, i * 10, 0, x, 0, 0, *identity]
                 for i, x in enumerate([0, 1, 2, 3.1])]
    _write_csv(
        session / "poses" / "original.csv",
        ["scan_id", "stamp_ns", "epoch", "tx", "ty", "tz", "qx", "qy", "qz", "qw"],
        pose_rows,
    )
    _write_csv(
        session / "poses" / "trajectory_original.csv",
        ["stamp_ns", "epoch", "tx", "ty", "tz", "qx", "qy", "qz", "qw"],
        [row[1:] for row in pose_rows],
    )

    result = run_global_pose_graph(
        keyframes, loop_frontend, session, tmp_path / "output",
        "loop-0001", PipelineConfig(),
    )

    metrics = json.loads(result["metrics"].read_text(encoding="utf-8"))
    audit = json.loads(result["audit"].read_text(encoding="utf-8"))
    assert metrics["graph_nodes"] == 4
    assert metrics["sequential_edges"] == 3
    assert metrics["selected_loop_edges"] == 1
    assert metrics["physical_loop_episodes"] == 1
    assert metrics["effective_loop_evidence"] == 1.0
    assert metrics["loop_translation_rmse_after_m"] < metrics["loop_translation_rmse_before_m"]
    assert audit["correlation_clusters"] == 1
    assert result["poses"].name == "poses_loop-0001.csv"
