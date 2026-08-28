import numpy as np
import pytest

from uf_global_pose_graph.metrics import compare_point_clouds, graph_residual_metrics
from uf_global_pose_graph.graph import GraphEdge
from uf_global_pose_graph.se3 import pose_matrix, se3_exp


def _edge(source, target, translation, kind):
    return GraphEdge(
        source, target, se3_exp([*translation, 0, 0, 0]), kind,
        0.1, 0.1,
    )


def test_graph_metrics_report_loop_and_sequential_residuals():
    before = {
        0: pose_matrix([0, 0, 0], [0, 0, 0, 1]),
        1: pose_matrix([1.1, 0, 0], [0, 0, 0, 1]),
    }
    after = {
        0: before[0],
        1: pose_matrix([1.0, 0, 0], [0, 0, 0, 1]),
    }
    sequential = [_edge(0, 1, [1.1, 0, 0], "sequential")]
    loops = [_edge(0, 1, [1.0, 0, 0], "loop")]

    metrics = graph_residual_metrics(before, after, sequential, loops)

    assert metrics["loop_translation_rmse_before_m"] == pytest.approx(0.1)
    assert metrics["loop_translation_rmse_after_m"] < 1.0e-10
    assert metrics["sequential_translation_rmse_before_m"] < 1.0e-10
    assert metrics["sequential_translation_rmse_after_m"] == pytest.approx(0.1)


def test_point_cloud_comparison_is_symmetric_and_detects_thickness_change():
    x, y = np.meshgrid(np.linspace(-1, 1, 20), np.linspace(-1, 1, 20))
    clean = np.column_stack((x.ravel(), y.ravel(), np.zeros(x.size)))
    ghost = np.vstack((clean, clean + np.array([0.0, 0.0, 0.08])))

    metrics = compare_point_clouds(ghost, clean, voxel_size_m=0.10)

    assert metrics["symmetric_nn_p95_m"] > 0.05
    assert metrics["surface_thickness_p95_before_m"] > metrics["surface_thickness_p95_after_m"]
    assert 0.0 < metrics["voxel_jaccard"] <= 1.0
