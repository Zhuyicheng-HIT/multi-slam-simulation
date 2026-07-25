"""Small deterministic ablation for the first backend increment.

This is a contract test, not a replacement for rosbag replay. It injects one
GNSS jump and compares fixed weighting with a scheduler decision that disables
the bad GNSS factor.
"""

import argparse
import csv
from pathlib import Path

import numpy as np

from .window import SlidingWindowBackend


def _run(dynamic: bool, seed: int = 7) -> float:
    rng = np.random.default_rng(seed)
    truth = np.array([[0.18 * index, -0.03 * index, 1.2] for index in range(20)])
    backend = SlidingWindowBackend(max_states=24)
    backend.add_state()
    backend.add_prior(0, np.r_[truth[0], np.zeros(12)], covariance=1.0e-4)
    for index in range(20):
        if index > 0:
            backend.add_state()
            backend.add_optical_flow(
                index - 1, index, truth[index] - truth[index - 1],
                covariance=np.full(3, 0.02 ** 2),
            )
        lidar = truth[index] + rng.normal(0.0, 0.04, size=3)
        backend.add_gnss(index, lidar, covariance=np.full(3, 0.04 ** 2))
        gnss = truth[index] + rng.normal(0.0, 0.12, size=3)
        if index == 10:
            gnss += np.array([4.0, -2.0, 0.0])
        decision = None
        if dynamic and index == 10:
            decision = {
                "factor_enabled": False,
                "reliability_weight": 0.0,
                "covariance_inflation": 20.0,
            }
        backend.add_gnss(index, gnss, covariance=np.full(3, 0.12 ** 2), decision=decision)
    estimate = np.array([state[:3] for state in backend.optimize()])
    return float(np.sqrt(np.mean(np.sum((estimate - truth) ** 2, axis=1))))


def run_ablation(output: str) -> list[dict[str, object]]:
    rows = [
        {"variant": "fixed_weight", "position_rmse_m": _run(dynamic=False)},
        {"variant": "scheduler_weighted", "position_rmse_m": _run(dynamic=True)},
    ]
    path = Path(output)
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=["variant", "position_rmse_m"])
        writer.writeheader()
        writer.writerows(rows)
    return rows


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output", default="ablation_table.csv")
    args = parser.parse_args()
    rows = run_ablation(args.output)
    for row in rows:
        print(f"{row['variant']}: position_rmse_m={row['position_rmse_m']:.6f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
