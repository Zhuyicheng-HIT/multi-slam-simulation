"""File-driven offline global pose graph pipeline."""

from dataclasses import asdict, dataclass, field
import hashlib
import json
from pathlib import Path

import numpy as np

from .graph import (
    CorrelationConfig,
    EpisodeConfig,
    GraphEdge,
    audit_correlated_loops,
    audit_loop_episodes,
    build_sequential_edges,
)
from .graph_io import load_keyframes, load_loop_candidates
from .metrics import graph_residual_metrics
from .optimizer import OptimizerConfig, optimize_pose_graph
from .revision import create_pose_revision


@dataclass(frozen=True)
class PipelineConfig:
    sequential_translation_sigma_m: float = 0.03
    sequential_rotation_sigma_rad: float = np.deg2rad(1.0)
    loop_translation_sigma_floor_m: float = 0.04
    loop_translation_sigma_ceiling_m: float = 0.20
    loop_rotation_sigma_rad: float = np.deg2rad(3.0)
    correlation: CorrelationConfig = field(default_factory=CorrelationConfig)
    episode: EpisodeConfig = field(default_factory=EpisodeConfig)
    optimizer: OptimizerConfig = field(default_factory=OptimizerConfig)

    def __post_init__(self):
        values = (
            self.sequential_translation_sigma_m,
            self.sequential_rotation_sigma_rad,
            self.loop_translation_sigma_floor_m,
            self.loop_translation_sigma_ceiling_m,
            self.loop_rotation_sigma_rad,
        )
        if any(value <= 0.0 for value in values):
            raise ValueError("graph sigmas must be positive")
        if self.loop_translation_sigma_floor_m > self.loop_translation_sigma_ceiling_m:
            raise ValueError("loop sigma floor exceeds ceiling")


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _loop_edges(selected, all_candidates, episode_assignments, config):
    audit_ids = {
        id(candidate): index for index,
        candidate in enumerate(all_candidates)}
    output = []
    for candidate, episode in zip(selected, episode_assignments):
        sigma = min(config.loop_translation_sigma_ceiling_m, max(
            config.loop_translation_sigma_floor_m, candidate.inlier_rmse_m), )
        output.append(GraphEdge(
            source_id=candidate.candidate_id,
            target_id=candidate.query_id,
            measurement=candidate.measurement,
            kind="loop",
            translation_sigma_m=sigma,
            rotation_sigma_rad=config.loop_rotation_sigma_rad,
            audit_id=audit_ids[id(candidate)],
            correlation_scale=episode["correlation_scale"],
        ))
    return output


def run_global_pose_graph(
    keyframes_path, loop_frontend_path, session_dir, output_dir,
    revision_id, config=None,
):
    config = config or PipelineConfig()
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=False)
    nodes = load_keyframes(keyframes_path)
    candidates = load_loop_candidates(loop_frontend_path, nodes)
    selected, correlation_audit = audit_correlated_loops(
        candidates, config.correlation)
    episode_assignments, episode_audit = audit_loop_episodes(
        selected, config.episode)
    sequential = build_sequential_edges(
        nodes,
        config.sequential_translation_sigma_m,
        config.sequential_rotation_sigma_rad,
    )
    loops = _loop_edges(selected, candidates, episode_assignments, config)
    optimized = optimize_pose_graph(nodes, sequential, loops, config.optimizer)
    active_loops = [
        edge for edge in loops
        if optimized.loop_status[edge.audit_id]["decision"] == "active"
    ]
    original = {node.keyframe_id: node.original_pose for node in nodes}
    metrics = graph_residual_metrics(
        original, optimized.corrected_poses, sequential, active_loops
    )
    metrics.update(optimized.metrics)
    metrics.update({
        "graph_nodes": len(nodes),
        "sequential_edges": len(sequential),
        "frontend_verified_loop_edges": len(candidates),
        "correlation_clusters": len({item["cluster_id"] for item in correlation_audit}),
        "correlated_redundant_loop_edges": sum(
            item["decision"] == "correlated_redundant" for item in correlation_audit
        ),
        "selected_loop_edges": len(loops),
        "physical_loop_episodes": episode_audit["episode_count"],
        "physical_loop_episode_sizes": episode_audit["episode_sizes"],
        "effective_loop_evidence": episode_audit["effective_independent_evidence"],
        "active_loop_edges_after_robust_gate": len(active_loops),
        "absolute_accuracy_evaluated": False,
        "evaluation_scope": "internal_loop_and_map_consistency_only",
    })
    selected_episode_by_audit = {
        edge.audit_id: episode for edge, episode in zip(
            loops, episode_assignments)}
    for item in correlation_audit:
        status = optimized.loop_status.get(item["audit_id"])
        if item["audit_id"] in selected_episode_by_audit:
            item["physical_episode"] = selected_episode_by_audit[item["audit_id"]]
        item["optimizer"] = status or {
            "decision": "not_submitted_correlated_redundancy",
        }
    audit_payload = {
        "schema_version": 1,
        "frontend_verified_edges": len(candidates),
        "correlation_clusters": metrics["correlation_clusters"],
        "selected_edges": len(loops),
        "physical_loop_episodes": episode_audit["episode_count"],
        "effective_independent_evidence": episode_audit["effective_independent_evidence"],
        "active_edges": len(active_loops),
        "entries": correlation_audit,
    }
    audit_path = output / "loop_edge_audit.json"
    audit_path.write_text(
        json.dumps(
            audit_payload,
            indent=2,
            sort_keys=True,
            allow_nan=False) + "\n",
        encoding="utf-8",
    )
    config_payload = asdict(config)
    config_hash = hashlib.sha256(
        json.dumps(config_payload, sort_keys=True).encode("utf-8")
    ).hexdigest()
    session = Path(session_dir)
    revision = create_pose_revision(
        output / "pose_revision",
        revision_id,
        session / "poses" / "original.csv",
        session / "poses" / "trajectory_original.csv",
        nodes,
        optimized.corrected_poses,
        {
            "keyframes_sha256": _sha256(keyframes_path),
            "loop_frontend_sha256": _sha256(loop_frontend_path),
            "loop_edge_audit_sha256": _sha256(audit_path),
            "pipeline_config_sha256": config_hash,
        },
    )
    metrics_path = output / "optimization_metrics.json"
    metrics_path.write_text(
        json.dumps(metrics, indent=2, sort_keys=True, allow_nan=False) + "\n",
        encoding="utf-8",
    )
    return {**revision, "audit": audit_path, "metrics": metrics_path}
