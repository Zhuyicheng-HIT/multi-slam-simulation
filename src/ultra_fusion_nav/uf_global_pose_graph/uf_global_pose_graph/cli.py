"""Command-line entrypoint for offline global pose graph and map rebuild."""

import argparse
import hashlib
import json
from pathlib import Path

import yaml

from .configuration import load_pipeline_config
from .metrics import compare_point_clouds
from .pipeline import PipelineConfig, run_global_pose_graph


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _build_map(session, revision, output, config_path=None):
    from uf_map_maintenance.builder import build_map_revision
    from uf_map_maintenance.voxel_map import MaintenanceConfig

    config = MaintenanceConfig()
    if config_path:
        payload = yaml.safe_load(Path(config_path).read_text(encoding="utf-8"))
        config = MaintenanceConfig(**payload["map"])
    return build_map_revision(
        session,
        revision["poses"],
        output,
        config,
        trajectory_revision=revision["trajectory"],
    )


def _map_validation(original_map, corrected_output, repeat_output=None):
    from uf_map_maintenance.pcd import read_binary_pcd

    corrected_map = Path(corrected_output) / "cleaned_map.pcd"
    payload = {
        "corrected_map_sha256": _sha256(corrected_map),
        "corrected_build_metrics": json.loads(
            (Path(corrected_output) /
             "metrics.json").read_text(
                encoding="utf-8")),
    }
    if original_map:
        payload["original_vs_corrected"] = compare_point_clouds(
            read_binary_pcd(original_map), read_binary_pcd(corrected_map)
        )
    if repeat_output:
        products = (
            "raw_scan_pose_map.pcd",
            "deskewed_map.pcd",
            "voxelized_map.pcd",
            "cleaned_map.pcd")
        hashes = {
            name: (
                _sha256(
                    Path(corrected_output) /
                    name),
                _sha256(
                    Path(repeat_output) /
                    name))
            for name in products
        }
        payload["deterministic_product_hashes"] = {
            name: {
                "first": pair[0],
                "repeat": pair[1],
                "equal": pair[0] == pair[1]}
            for name, pair in hashes.items()
        }
        payload["rebuild_deterministic"] = all(
            left == right for left, right in hashes.values())
    return payload


def main(argv=None):
    parser = argparse.ArgumentParser(
        description="Optimize an independent offline global SE(3) pose graph")
    parser.add_argument("--keyframes", required=True, type=Path)
    parser.add_argument("--loop-frontend", required=True, type=Path)
    parser.add_argument("--session", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--revision", default="loop-0001")
    parser.add_argument("--config", type=Path)
    parser.add_argument("--map-output", type=Path)
    parser.add_argument("--repeat-map-output", type=Path)
    parser.add_argument("--map-config", type=Path)
    parser.add_argument("--original-map", type=Path)
    arguments = parser.parse_args(argv)
    config = load_pipeline_config(
        arguments.config) if arguments.config else PipelineConfig()
    result = run_global_pose_graph(
        arguments.keyframes,
        arguments.loop_frontend,
        arguments.session,
        arguments.output,
        arguments.revision,
        config,
    )
    if arguments.map_output:
        _build_map(
            arguments.session,
            result,
            arguments.map_output,
            arguments.map_config)
        if arguments.repeat_map_output:
            _build_map(
                arguments.session,
                result,
                arguments.repeat_map_output,
                arguments.map_config)
        validation = _map_validation(
            arguments.original_map,
            arguments.map_output,
            arguments.repeat_map_output,
        )
        validation_path = arguments.output / "map_validation.json"
        validation_path.write_text(
            json.dumps(
                validation,
                indent=2,
                sort_keys=True,
                allow_nan=False) + "\n",
            encoding="utf-8",
        )
        result["map_validation"] = validation_path
    print(json.dumps({name: str(path)
          for name, path in result.items()}, sort_keys=True))
    return 0
