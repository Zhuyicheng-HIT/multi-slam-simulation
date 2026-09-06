#!/usr/bin/env python3
"""Offline trajectory scoring with a pre-event-only alignment transform."""
import argparse
import json
from pathlib import Path

import numpy as np

from evaluate_lio_trajectory import match_rows, read_tum


def percentile(values, q):
    return float(np.percentile(values, q)) if len(values) else None


def summary(errors):
    errors = np.asarray(errors, dtype=np.float64)
    if errors.size == 0:
        return {"count": 0, "rmse": None, "p95": None, "max": None}
    return {
        "count": int(errors.size),
        "rmse": float(np.sqrt(np.mean(errors * errors))),
        "p95": percentile(errors, 95),
        "max": float(np.max(errors)),
    }


def yaw_from_quaternion(rows):
    x, y, z, w = rows[:, 0], rows[:, 1], rows[:, 2], rows[:, 3]
    return np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def initial_yaw_translation(est, ref, mask):
    est_yaw = yaw_from_quaternion(est[mask, 4:8])
    ref_yaw = yaw_from_quaternion(ref[mask, 4:8])
    delta = np.angle(np.mean(np.exp(1j * (ref_yaw - est_yaw))))
    c, s = np.cos(delta), np.sin(delta)
    rotation = np.asarray([[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]])
    translation = np.mean(ref[mask, 1:4], axis=0) - (
        rotation @ np.mean(est[mask, 1:4], axis=0))
    return rotation, translation


def evaluate(estimate, truth, align_end_s, event_start_s, event_end_s, max_delta_s):
    pairs = match_rows(estimate, truth, max_delta_s)
    if len(pairs) < 3:
        raise ValueError("Fewer than three timestamp-matched poses")
    est = np.asarray([p[0] for p in pairs])
    ref = np.asarray([p[1] for p in pairs])
    pre = est[:, 0] <= align_end_s
    if np.count_nonzero(pre) < 3:
        raise ValueError("Fewer than three pre-event samples for alignment")
    rotation, translation = initial_yaw_translation(est, ref, pre)
    aligned = (rotation @ est[:, 1:4].T).T + translation
    raw_error = np.linalg.norm(est[:, 1:4] - ref[:, 1:4], axis=1)
    raw_delta = est[:, 1:4] - ref[:, 1:4]
    raw_xy = np.linalg.norm(raw_delta[:, :2], axis=1)
    raw_z = np.abs(raw_delta[:, 2])
    aligned_delta = aligned - ref[:, 1:4]
    xy = np.linalg.norm(aligned_delta[:, :2], axis=1)
    z = np.abs(aligned_delta[:, 2])
    phases = {
        "BEFORE": est[:, 0] < event_start_s,
        "DURING": (est[:, 0] >= event_start_s) & (est[:, 0] < event_end_s),
        "AFTER": est[:, 0] >= event_end_s,
    }
    phase_metrics = {
        name: {"xy": summary(xy[mask]), "z": summary(z[mask])}
        for name, mask in phases.items()
    }
    # Measure drift relative to the last nominal samples; this is a diagnostic
    # increment, not a second trajectory fit.
    pre_recent = pre & (est[:, 0] >= align_end_s - 1.0)
    if np.count_nonzero(pre_recent) < 1:
        pre_recent = pre
    nominal_error = np.mean(aligned_delta[pre_recent], axis=0)
    additional = aligned_delta - nominal_error
    additional_xy = np.linalg.norm(additional[:, :2], axis=1)
    additional_z = np.abs(additional[:, 2])
    post_event = est[:, 0] >= event_start_s
    over_20 = np.flatnonzero(post_event & (xy > 0.20))
    first_over_20 = float(est[over_20[0], 0] - event_start_s) if len(over_20) else None
    if np.any(post_event):
        post_times = est[post_event, 0]
        under = xy[post_event] <= 0.20
        longest_under_20 = 0.0
        start = None
        for t, ok in zip(post_times, under):
            if ok and start is None:
                start = t
            elif not ok and start is not None:
                longest_under_20 = max(longest_under_20, t - start)
                start = None
        if start is not None:
            longest_under_20 = max(longest_under_20, post_times[-1] - start)
    else:
        longest_under_20 = None
    return {
        "matched_poses": int(len(pairs)),
        "alignment_samples": int(np.count_nonzero(pre)),
        "alignment_end_s": align_end_s,
        "event_start_s": event_start_s,
        "event_end_s": event_end_s,
        "raw_3d": summary(raw_error),
        "raw_xy": summary(raw_xy),
        "raw_z": summary(raw_z),
        "fixed_initial_alignment": {
            "xy": summary(xy),
            "z": summary(z),
            "phase": phase_metrics,
        },
        "additional_drift_from_nominal": {
            "xy": summary(additional_xy),
            "z": summary(additional_z),
            "phase": {
                name: {"xy": summary(additional_xy[mask]), "z": summary(additional_z[mask])}
                for name, mask in phases.items()
            },
        },
        "fault_threshold_diagnostics": {
            "first_xy_over_0_20_after_event_s": first_over_20,
            "longest_contiguous_xy_under_0_20_after_event_s": longest_under_20,
        },
        "initial_rotation": rotation.tolist(),
        "initial_translation": translation.tolist(),
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--estimate", required=True)
    parser.add_argument("--truth", required=True)
    parser.add_argument("--align-end", type=float, required=True)
    parser.add_argument("--event-start", type=float, required=True)
    parser.add_argument("--event-end", type=float, required=True)
    parser.add_argument("--max-delta", type=float, default=0.05)
    parser.add_argument("--output")
    args = parser.parse_args()
    report = evaluate(
        read_tum(args.estimate), read_tum(args.truth), args.align_end,
        args.event_start, args.event_end, args.max_delta)
    text = json.dumps(report, indent=2)
    if args.output:
        Path(args.output).write_text(text + "\n", encoding="utf-8")
    print(text)


if __name__ == "__main__":
    main()
