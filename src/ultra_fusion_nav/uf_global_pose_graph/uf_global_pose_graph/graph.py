"""Pose-graph records and deterministic correlated-loop admission."""

from dataclasses import dataclass

import numpy as np

from .se3 import compose, inverse, se3_log


@dataclass(frozen=True)
class GraphNode:
    keyframe_id: int
    scan_id: int
    stamp_ns: int
    original_pose: np.ndarray


@dataclass(frozen=True)
class GraphEdge:
    source_id: int
    target_id: int
    measurement: np.ndarray
    kind: str
    translation_sigma_m: float
    rotation_sigma_rad: float
    audit_id: int = -1
    correlation_scale: float = 1.0

    def __post_init__(self):
        if self.correlation_scale <= 0.0 or self.correlation_scale > 1.0:
            raise ValueError("correlation_scale must be in (0, 1]")


@dataclass(frozen=True)
class LoopCandidate:
    candidate_id: int
    query_id: int
    measurement: np.ndarray
    descriptor_distance: float
    correspondence_points: int
    overlap_ratio: float
    reciprocal_ratio: float
    inlier_rmse_m: float
    effective_rank: int
    condition_number: float


@dataclass(frozen=True)
class CorrelationConfig:
    endpoint_index_radius: int = 2
    translation_similarity_m: float = 0.25
    rotation_similarity_rad: float = np.deg2rad(10.0)

    def __post_init__(self):
        if self.endpoint_index_radius < 0:
            raise ValueError("endpoint_index_radius must be nonnegative")
        if self.translation_similarity_m <= 0.0 or self.rotation_similarity_rad <= 0.0:
            raise ValueError("loop similarity thresholds must be positive")


@dataclass(frozen=True)
class EpisodeConfig:
    query_index_radius: int = 2
    candidate_index_radius: int = 2

    def __post_init__(self):
        if self.query_index_radius < 0 or self.candidate_index_radius < 0:
            raise ValueError("episode index radii must be nonnegative")


def build_sequential_edges(
    nodes, translation_sigma_m=0.03, rotation_sigma_rad=np.deg2rad(1.0)
):
    ordered = list(nodes)
    if len({node.keyframe_id for node in ordered}) != len(ordered):
        raise ValueError("duplicate keyframe id")
    edges = []
    for left, right in zip(ordered[:-1], ordered[1:]):
        edges.append(GraphEdge(
            source_id=left.keyframe_id,
            target_id=right.keyframe_id,
            measurement=compose(
                inverse(
                    left.original_pose),
                right.original_pose),
            kind="sequential",
            translation_sigma_m=float(translation_sigma_m),
            rotation_sigma_rad=float(rotation_sigma_rad),
        ))
    return edges


def _quality_key(candidate):
    return (
        -int(candidate.effective_rank),
        float(candidate.inlier_rmse_m),
        -float(candidate.reciprocal_ratio),
        -float(candidate.overlap_ratio),
        -int(candidate.correspondence_points),
        float(candidate.descriptor_distance),
        float(candidate.condition_number),
        int(candidate.query_id),
        int(candidate.candidate_id),
    )


def _correlated(left, right, config):
    endpoints_close = (
        abs(int(left.candidate_id) - int(right.candidate_id)
            ) <= config.endpoint_index_radius
        and abs(int(left.query_id) - int(right.query_id)) <= config.endpoint_index_radius
    )
    if not endpoints_close:
        return False
    delta = se3_log(compose(inverse(left.measurement), right.measurement))
    return (
        np.linalg.norm(delta[:3]) <= config.translation_similarity_m
        and np.linalg.norm(delta[3:]) <= config.rotation_similarity_rad
    )


def audit_correlated_loops(candidates, config=None):
    """Select one edge per correlated physical-loop cluster and retain full audit."""
    config = config or CorrelationConfig()
    values = list(candidates)
    parent = list(range(len(values)))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left, right):
        left_root = find(left)
        right_root = find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    for left in range(len(values)):
        for right in range(left + 1, len(values)):
            if _correlated(values[left], values[right], config):
                union(left, right)

    groups = {}
    for index in range(len(values)):
        groups.setdefault(find(index), []).append(index)

    selected = []
    audit = []
    for cluster_id, root in enumerate(
            sorted(groups, key=lambda item: min(groups[item]))):
        indices = groups[root]
        best_index = min(
            indices,
            key=lambda index: _quality_key(
                values[index]))
        selected.append(values[best_index])
        for index in indices:
            candidate = values[index]
            audit.append({
                "audit_id": index,
                "candidate_id": int(candidate.candidate_id),
                "query_id": int(candidate.query_id),
                "cluster_id": cluster_id,
                "cluster_size": len(indices),
                "representative_audit_id": best_index,
                "decision": "selected" if index == best_index else "correlated_redundant",
            })
    return selected, sorted(audit, key=lambda item: item["audit_id"])


def audit_loop_episodes(candidates, config=None):
    """Normalize information from temporally adjacent edges in one revisit episode."""
    config = config or EpisodeConfig()
    values = list(candidates)
    parent = list(range(len(values)))

    def find(index):
        while parent[index] != index:
            parent[index] = parent[parent[index]]
            index = parent[index]
        return index

    def union(left, right):
        left_root = find(left)
        right_root = find(right)
        if left_root != right_root:
            parent[right_root] = left_root

    for left in range(len(values)):
        for right in range(left + 1, len(values)):
            if (
                abs(values[left].query_id -
                    values[right].query_id) <= config.query_index_radius
                and abs(values[left].candidate_id - values[right].candidate_id) <=
                config.candidate_index_radius
            ):
                union(left, right)
    groups = {}
    for index in range(len(values)):
        groups.setdefault(find(index), []).append(index)
    assignments = [{} for _ in values]
    episode_sizes = []
    for episode_id, root in enumerate(
            sorted(groups, key=lambda item: min(groups[item]))):
        indices = groups[root]
        episode_sizes.append(len(indices))
        scale = 1.0 / np.sqrt(len(indices))
        for index in indices:
            assignments[index] = {
                "episode_id": episode_id,
                "episode_size": len(indices),
                "correlation_scale": float(scale),
            }
    return assignments, {
        "episode_count": len(groups),
        "episode_sizes": episode_sizes,
        "effective_independent_evidence": float(sum(
            item["correlation_scale"] ** 2 for item in assignments
        )),
    }
