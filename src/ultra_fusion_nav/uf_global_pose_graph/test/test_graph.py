import numpy as np
import pytest

from uf_global_pose_graph.graph import (
    CorrelationConfig,
    EpisodeConfig,
    GraphNode,
    LoopCandidate,
    audit_correlated_loops,
    audit_loop_episodes,
    build_sequential_edges,
)
from uf_global_pose_graph.se3 import pose_matrix, se3_exp


def _node(keyframe_id, x):
    return GraphNode(
        keyframe_id=keyframe_id,
        scan_id=keyframe_id * 10,
        stamp_ns=keyframe_id * 1_000_000_000,
        original_pose=pose_matrix([x, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
    )


def _candidate(candidate_id, query_id, delta, rmse, reciprocal=0.6):
    return LoopCandidate(
        candidate_id=candidate_id,
        query_id=query_id,
        measurement=se3_exp(np.asarray(delta, dtype=float)),
        descriptor_distance=0.02,
        correspondence_points=1000,
        overlap_ratio=0.8,
        reciprocal_ratio=reciprocal,
        inlier_rmse_m=rmse,
        effective_rank=6,
        condition_number=20.0,
    )


def test_sequential_edges_preserve_original_relative_motion():
    nodes = [_node(0, 0.0), _node(1, 1.0), _node(2, 2.5)]

    edges = build_sequential_edges(nodes)

    assert [(edge.source_id, edge.target_id)
            for edge in edges] == [(0, 1), (1, 2)]
    np.testing.assert_allclose(edges[0].measurement[:3, 3], [1.0, 0.0, 0.0])
    np.testing.assert_allclose(edges[1].measurement[:3, 3], [1.5, 0.0, 0.0])


def test_correlated_loop_cluster_contributes_only_best_representative():
    candidates = [
        _candidate(0, 10, [0.10, 0, 0, 0, 0, 0], 0.08),
        _candidate(1, 11, [0.11, 0, 0, 0, 0, 0.01], 0.05, reciprocal=0.7),
        _candidate(0, 10, [0.12, 0, 0, 0, 0, 0], 0.12),
        _candidate(0, 15, [0.10, 0, 0, 0, 0, 0], 0.04),
    ]
    config = CorrelationConfig(
        endpoint_index_radius=1,
        translation_similarity_m=0.20,
        rotation_similarity_rad=0.10,
    )

    selected, audit = audit_correlated_loops(candidates, config)

    assert [(edge.candidate_id, edge.query_id)
            for edge in selected] == [(1, 11), (0, 15)]
    assert len(audit) == 4
    assert sum(item["decision"] == "selected" for item in audit) == 2
    assert sum(item["decision"] ==
               "correlated_redundant" for item in audit) == 2
    first_cluster = [item for item in audit if item["cluster_id"] == 0]
    assert {item["cluster_size"] for item in first_cluster} == {3}


def test_geometrically_distinct_loop_is_not_collapsed_by_endpoint_proximity():
    candidates = [
        _candidate(0, 10, [0.0, 0, 0, 0, 0, 0], 0.05),
        _candidate(1, 11, [0.8, 0, 0, 0, 0, 0], 0.04),
    ]

    selected, _ = audit_correlated_loops(candidates, CorrelationConfig())

    assert len(selected) == 2


def test_same_revisit_episode_is_information_normalized_even_when_measurements_differ():
    candidates = [
        _candidate(0, 10, [0.0, 0, 0, 0, 0, 0], 0.05),
        _candidate(1, 11, [0.8, 0, 0, 0, 0, 0], 0.04),
        _candidate(8, 20, [0.1, 0, 0, 0, 0, 0], 0.04),
    ]

    assignments, audit = audit_loop_episodes(
        candidates, EpisodeConfig(
            query_index_radius=2, candidate_index_radius=2)
    )

    assert assignments[0]["episode_id"] == assignments[1]["episode_id"]
    assert assignments[0]["correlation_scale"] == pytest.approx(
        1.0 / np.sqrt(2.0))
    assert assignments[2]["correlation_scale"] == 1.0
    assert audit["effective_independent_evidence"] == pytest.approx(2.0)
