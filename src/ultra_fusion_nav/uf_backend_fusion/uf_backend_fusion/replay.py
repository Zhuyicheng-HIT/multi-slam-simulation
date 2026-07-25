"""Replay extracted factors with fixed or scheduler-derived weights."""

import csv
import json
from pathlib import Path

import numpy as np

from .window import SlidingWindowBackend


def _decision(frame: dict, factor_name: str, dynamic: bool):
    if not dynamic:
        return None
    factor = frame.get(factor_name)
    return None if factor is None else factor.get("decision")


def _percentile(values, fraction):
    if not values:
        return None
    return float(np.percentile(np.asarray(values, dtype=float), fraction * 100.0))


def replay_factors(data: dict, dynamic: bool, window_size: int = 20):
    backend = SlidingWindowBackend(max_states=window_size)
    estimates = []
    frames = data["frames"]
    for index, frame in enumerate(frames):
        local_index = backend.add_state()
        if index == 0:
            prior = np.zeros(15, dtype=float)
            prior[:3] = np.asarray(frame["lidar_pose"]["position"], dtype=float)
            prior[3:6] = np.asarray(frame["lidar_pose"]["rotation"], dtype=float)
            backend.add_prior(local_index, prior, covariance=np.full(15, 1.0e-4))
        lidar = frame["lidar_pose"]
        backend.add_lidar_pose(
            local_index,
            lidar["position"],
            lidar["rotation"],
            covariance=np.array([0.05 ** 2] * 3 + [0.03 ** 2] * 3),
            decision=_decision(frame, "lidar_pose", dynamic),
        )
        if frame.get("gnss") is not None:
            gnss = frame["gnss"]
            backend.add_gnss(
                local_index,
                gnss["position"],
                covariance=gnss["covariance"],
                decision=_decision(frame, "gnss", dynamic),
            )
        if index > 0 and frame.get("optical_flow") is not None:
            flow = frame["optical_flow"]
            backend.add_optical_flow(
                local_index - 1,
                local_index,
                flow["delta_position"],
                covariance=flow["covariance"],
                decision=_decision(frame, "optical_flow", dynamic),
            )
        states = backend.optimize()
        estimates.append(backend.state(local_index))
    positions = np.asarray([state[:3] for state in estimates], dtype=float)
    reference = [frame.get("reference_position") for frame in frames]
    valid_reference = [item is not None for item in reference]
    reference_rmse = None
    if any(valid_reference):
        reference_array = np.asarray(
            [reference[index] for index in range(len(reference)) if valid_reference[index]],
            dtype=float,
        )
        estimate_array = positions[np.asarray(valid_reference, dtype=bool)]
        reference_rmse = float(
            np.sqrt(np.mean(np.sum((estimate_array - reference_array) ** 2, axis=1)))
        )
    gnss_residuals = []
    flow_residuals = []
    for index, frame in enumerate(frames):
        if frame.get("gnss") is not None:
            gnss_residuals.append(
                float(np.linalg.norm(
                    positions[index] - np.asarray(frame["gnss"]["position"], dtype=float)
                ))
            )
        if index > 0 and frame.get("optical_flow") is not None:
            measured = np.asarray(frame["optical_flow"]["delta_position"], dtype=float)
            predicted = positions[index] - positions[index - 1]
            flow_residuals.append(float(np.linalg.norm(predicted - measured)))
    return {
        "variant": "scheduler_weighted" if dynamic else "fixed_weight",
        "frame_count": len(frames),
        "factor_count": backend.factor_count,
        "position_rmse_vs_evaluation_reference_m": reference_rmse,
        "gnss_residual_p50_m": _percentile(gnss_residuals, 0.50),
        "gnss_residual_p95_m": _percentile(gnss_residuals, 0.95),
        "gnss_residual_max_m": max(gnss_residuals) if gnss_residuals else None,
        "optical_flow_residual_p50_m": _percentile(flow_residuals, 0.50),
        "optical_flow_residual_p95_m": _percentile(flow_residuals, 0.95),
        "optical_flow_residual_max_m": max(flow_residuals) if flow_residuals else None,
        "last_cost": backend.last_cost,
        "positions": positions.tolist(),
        "factor_summary": [record.__dict__ for record in backend.factor_summary()],
    }


def write_replay_outputs(data: dict, output_dir: str, window_size: int = 20):
    output = Path(output_dir)
    output.mkdir(parents=True, exist_ok=True)
    reports = [
        replay_factors(data, dynamic=False, window_size=window_size),
        replay_factors(data, dynamic=True, window_size=window_size),
    ]
    table_path = output / "ablation_table.csv"
    with table_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(
            stream,
            fieldnames=[
                "variant", "frame_count", "factor_count",
                "position_rmse_vs_evaluation_reference_m", "last_cost",
                "gnss_residual_p50_m", "gnss_residual_p95_m",
                "optical_flow_residual_p50_m", "optical_flow_residual_p95_m",
            ],
        )
        writer.writeheader()
        for report in reports:
            writer.writerow({key: report[key] for key in writer.fieldnames})
    for report in reports:
        (output / f"{report['variant']}.json").write_text(
            json.dumps(report, indent=2, allow_nan=False) + "\n", encoding="utf-8"
        )
    summary = {
        "schema_version": 1,
        "source_bag": data.get("source_bag"),
        "reference_topic": data.get("reference_topic"),
        "reference_is_evaluation_only": data.get("reference_is_evaluation_only", True),
        "variants": reports,
    }
    (output / "replay_report.json").write_text(
        json.dumps(summary, indent=2, allow_nan=False) + "\n", encoding="utf-8"
    )
    return summary
