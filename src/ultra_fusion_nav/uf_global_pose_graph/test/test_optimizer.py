import numpy as np

from uf_global_pose_graph.graph import GraphEdge, GraphNode, build_sequential_edges
from uf_global_pose_graph.optimizer import OptimizerConfig, edge_error, optimize_pose_graph
from uf_global_pose_graph.se3 import pose_matrix, se3_exp


def _node(keyframe_id, translation):
    return GraphNode(
        keyframe_id=keyframe_id,
        scan_id=keyframe_id,
        stamp_ns=keyframe_id * 1_000_000_000,
        original_pose=pose_matrix(translation, [0.0, 0.0, 0.0, 1.0]),
    )


def _loop(source, target, translation, audit_id=0):
    return GraphEdge(
        source_id=source,
        target_id=target,
        measurement=se3_exp([*translation, 0.0, 0.0, 0.0]),
        kind="loop",
        translation_sigma_m=0.04,
        rotation_sigma_rad=0.05,
        audit_id=audit_id,
    )


def test_fixed_first_node_and_valid_loop_reduce_closure_residual():
    nodes = [
        _node(0, [0.0, 0.0, 0.0]),
        _node(1, [1.0, 0.0, 0.0]),
        _node(2, [1.0, 1.0, 0.0]),
        _node(3, [0.2, 1.0, 0.0]),
        _node(4, [0.2, 0.15, 0.0]),
    ]
    sequential = build_sequential_edges(nodes, 0.08, 0.05)
    loop = _loop(0, 4, [0.0, 0.0, 0.0])
    original = {node.keyframe_id: node.original_pose for node in nodes}
    before = np.linalg.norm(edge_error(loop, original)[:3])

    result = optimize_pose_graph(
        nodes, sequential, [loop],
        OptimizerConfig(robust_phi=100.0, minimum_loop_weight=0.02),
    )

    after = np.linalg.norm(edge_error(loop, result.corrected_poses)[:3])
    assert result.converged
    np.testing.assert_allclose(
        result.corrected_poses[0],
        nodes[0].original_pose,
        atol=1.0e-12)
    assert before > 0.20
    assert after < 0.05
    assert result.metrics["effective_rank"] == 24


def test_single_adversarial_loop_is_quarantined_without_distorting_chain():
    nodes = [_node(index, [float(index), 0.0, 0.0]) for index in range(4)]
    sequential = build_sequential_edges(nodes, 0.03, 0.03)
    good = _loop(0, 3, [3.0, 0.0, 0.0], audit_id=10)
    bad = _loop(0, 2, [30.0, 0.0, 0.0], audit_id=11)

    result = optimize_pose_graph(
        nodes, sequential, [good, bad],
        OptimizerConfig(robust_phi=4.0, minimum_loop_weight=0.10),
    )

    assert result.converged
    assert result.loop_status[10]["decision"] == "active"
    assert result.loop_status[11]["decision"] == "quarantined_low_robust_weight"
    for node in nodes:
        np.testing.assert_allclose(
            result.corrected_poses[node.keyframe_id], node.original_pose, atol=1.0e-5
        )
