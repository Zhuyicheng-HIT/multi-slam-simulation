#!/usr/bin/env python3
import argparse
import json
import math
from pathlib import Path

import numpy as np


def read_tum(path):
    rows = []
    for line in Path(path).read_text(encoding="ascii").splitlines():
        if not line or line.startswith("#"):
            continue
        values = [float(value) for value in line.split()]
        if len(values) != 8:
            raise ValueError(f"Expected 8 TUM columns in {path}: {line}")
        rows.append(values)
    return np.asarray(rows, dtype=np.float64)


def match_rows(estimate, truth, max_delta_s=0.05):
    matches = []
    truth_times = truth[:, 0]
    for row in estimate:
        index = int(np.searchsorted(truth_times, row[0]))
        candidates = [i for i in (index - 1, index) if 0 <= i < len(truth)]
        if not candidates:
            continue
        best = min(candidates, key=lambda i: abs(truth_times[i] - row[0]))
        if abs(truth_times[best] - row[0]) <= max_delta_s:
            matches.append((row, truth[best]))
    return matches


def rigid_align(source, target):
    source_center = np.mean(source, axis=0)
    target_center = np.mean(target, axis=0)
    covariance = (source - source_center).T @ (target - target_center)
    u, _, vt = np.linalg.svd(covariance)
    rotation = vt.T @ u.T
    if np.linalg.det(rotation) < 0.0:
        vt[-1] *= -1.0
        rotation = vt.T @ u.T
    translation = target_center - rotation @ source_center
    return rotation, translation


def quat_multiply(a, b):
    x1, y1, z1, w1 = a
    x2, y2, z2, w2 = b
    return np.asarray([
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    ])


def quat_inverse(q):
    q = np.asarray(q)
    norm = float(np.dot(q, q))
    return np.asarray([-q[0], -q[1], -q[2], q[3]]) / max(norm, 1.0e-12)


def quat_angle_deg(q):
    q = np.asarray(q) / max(np.linalg.norm(q), 1.0e-12)
    return math.degrees(2.0 * math.acos(min(1.0, abs(float(q[3])))))


def evaluate(estimate, truth, max_delta_s=0.05):
    matches = match_rows(estimate, truth, max_delta_s)
    if len(matches) < 3:
        raise ValueError("Fewer than three timestamp-matched poses")
    est = np.asarray([pair[0] for pair in matches])
    ref = np.asarray([pair[1] for pair in matches])
    rotation, translation = rigid_align(est[:, 1:4], ref[:, 1:4])
    aligned = (rotation @ est[:, 1:4].T).T + translation
    ate = np.linalg.norm(aligned - ref[:, 1:4], axis=1)
    rpe_translation = []
    rpe_rotation = []
    for index in range(len(matches) - 1):
        est_delta = aligned[index + 1] - aligned[index]
        ref_delta = ref[index + 1, 1:4] - ref[index, 1:4]
        rpe_translation.append(float(np.linalg.norm(est_delta - ref_delta)))
        dq_est = quat_multiply(quat_inverse(est[index, 4:8]), est[index + 1, 4:8])
        dq_ref = quat_multiply(quat_inverse(ref[index, 4:8]), ref[index + 1, 4:8])
        rpe_rotation.append(quat_angle_deg(quat_multiply(quat_inverse(dq_ref), dq_est)))
    return {
        "matched_poses": len(matches),
        "ate_rmse_m": float(np.sqrt(np.mean(np.square(ate)))),
        "ate_median_m": float(np.median(ate)),
        "ate_max_m": float(np.max(ate)),
        "rpe_translation_rmse_m": float(np.sqrt(np.mean(np.square(rpe_translation)))),
        "rpe_rotation_rmse_deg": float(np.sqrt(np.mean(np.square(rpe_rotation)))),
    }


def main():
    parser = argparse.ArgumentParser(description="Compute aligned ATE and adjacent-pose RPE from TUM trajectories")
    parser.add_argument("--estimate", required=True)
    parser.add_argument("--truth", required=True)
    parser.add_argument("--max-delta", type=float, default=0.05)
    parser.add_argument("--output")
    args = parser.parse_args()
    report = evaluate(read_tum(args.estimate), read_tum(args.truth), args.max_delta)
    text = json.dumps(report, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
