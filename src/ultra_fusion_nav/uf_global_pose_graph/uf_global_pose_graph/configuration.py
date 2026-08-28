"""YAML configuration loading with explicit degree-to-radian conversion."""

import numpy as np
import yaml

from .graph import CorrelationConfig, EpisodeConfig
from .optimizer import OptimizerConfig
from .pipeline import PipelineConfig


def load_pipeline_config(path):
    payload = yaml.safe_load(open(path, encoding="utf-8")) or {}
    graph = dict(payload.get("graph", {}))
    correlation = dict(payload.get("correlation", {}))
    optimizer = dict(payload.get("optimizer", {}))
    episode = dict(payload.get("physical_loop_episode", {}))
    if "sequential_rotation_sigma_deg" in graph:
        graph["sequential_rotation_sigma_rad"] = np.deg2rad(
            graph.pop("sequential_rotation_sigma_deg")
        )
    if "loop_rotation_sigma_deg" in graph:
        graph["loop_rotation_sigma_rad"] = np.deg2rad(
            graph.pop("loop_rotation_sigma_deg")
        )
    if "rotation_similarity_deg" in correlation:
        correlation["rotation_similarity_rad"] = np.deg2rad(
            correlation.pop("rotation_similarity_deg")
        )
    for degree_name, radian_name in (
        ("maximum_rotation_correction_deg", "maximum_rotation_correction_rad"),
        ("maximum_sequential_rotation_strain_deg",
         "maximum_sequential_rotation_strain_rad"),
    ):
        if degree_name in optimizer:
            optimizer[radian_name] = np.deg2rad(optimizer.pop(degree_name))
    return PipelineConfig(
        **graph,
        correlation=CorrelationConfig(**correlation),
        episode=EpisodeConfig(**episode),
        optimizer=OptimizerConfig(**optimizer),
    )
