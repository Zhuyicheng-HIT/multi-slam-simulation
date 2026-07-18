#!/usr/bin/env python3
import argparse
import csv
import json
import math
from pathlib import Path

import numpy as np

from uf_aiding.gnss_reanchor import ACTIVE, SmoothGnssReanchor


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-json", required=True)
    parser.add_argument("--output-csv", required=True)
    args = parser.parse_args()

    reanchor = SmoothGnssReanchor(
        jump_threshold_m=3.0,
        outage_timeout_s=1.5,
        reacquire_samples=5,
        reanchor_duration_s=3.0,
        max_degradation_score=0.45,
    )
    rows = []
    for index in range(101):
        timestamp = index * 0.2
        reference = np.asarray([0.2 * timestamp, 0.0, 3.0])
        noise = np.asarray([0.03 * math.sin(timestamp), 0.02 * math.cos(timestamp), 0.0])
        phase = "normal"
        valid = True
        score = 0.05
        gnss = reference + np.asarray([100.0, 20.0, 0.0]) + noise
        if 5.0 <= timestamp < 8.0:
            phase = "outage"
            valid = False
            score = 1.0
            gnss = None
        elif 8.0 <= timestamp < 10.0:
            phase = "jump"
            score = 0.9
            gnss = reference + np.asarray([125.0, 5.0, 0.0])
        elif timestamp >= 10.0:
            phase = "recovered"
            gnss = reference + np.asarray([104.0, -18.0, 0.0]) + noise

        result = reanchor.update(timestamp, gnss, reference, score, valid)
        position_error = float("nan")
        output_x = float("nan")
        if result.position is not None:
            output_x = float(result.position[0])
            position_error = float(np.linalg.norm(result.position - reference))
        rows.append({
            "time_s": timestamp,
            "phase": phase,
            "state": result.state,
            "accepted": int(result.accepted),
            "blend": result.blend,
            "innovation_m": result.innovation_m,
            "reference_x_m": float(reference[0]),
            "output_x_m": output_x,
            "position_error_m": position_error,
            "reason": result.reason,
        })

    outage_or_jump = [row for row in rows if row["phase"] in ("outage", "jump")]
    accepted = [row for row in rows if row["accepted"]]
    recovered = [row for row in rows if row["phase"] == "recovered" and row["accepted"]]
    errors = [row["position_error_m"] for row in accepted if math.isfinite(row["position_error_m"])]
    recovered_blends = [row["blend"] for row in recovered]
    first_recovered = recovered[0] if recovered else None
    passed = bool(
        accepted
        and not any(row["accepted"] for row in outage_or_jump)
        and first_recovered is not None
        and abs(first_recovered["blend"]) <= 1.0e-9
        and all(b + 1.0e-9 >= a for a, b in zip(recovered_blends[:-1], recovered_blends[1:]))
        and max(errors) <= 0.15
        and rows[-1]["state"] == ACTIVE
        and abs(rows[-1]["blend"] - 1.0) <= 1.0e-9
    )
    result = {
        "samples": len(rows),
        "accepted_samples": len(accepted),
        "outage_or_jump_accepted": sum(row["accepted"] for row in outage_or_jump),
        "first_recovered_accept_time_s": None if first_recovered is None else first_recovered["time_s"],
        "first_recovered_blend": None if first_recovered is None else first_recovered["blend"],
        "max_accepted_position_error_m": max(errors) if errors else None,
        "final_state": rows[-1]["state"],
        "final_blend": rows[-1]["blend"],
        "passed": passed,
    }

    output_json = Path(args.output_json)
    output_json.parent.mkdir(parents=True, exist_ok=True)
    output_json.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    output_csv = Path(args.output_csv)
    output_csv.parent.mkdir(parents=True, exist_ok=True)
    with output_csv.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=rows[0].keys())
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps(result, indent=2))
    return 0 if passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
