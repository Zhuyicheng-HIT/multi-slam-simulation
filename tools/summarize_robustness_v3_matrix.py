#!/usr/bin/env python3
"""Aggregate all V3 A/B runs and derive evidence-based operating boundaries."""

import argparse
import csv
import json
from pathlib import Path
import statistics


LEVELS = {"light": 1, "medium": 2, "heavy": 3}


def nested(report, *keys, default=None):
    value = report
    for key in keys:
        if not isinstance(value, dict) or key not in value:
            return default
        value = value[key]
    return value


def level(profile):
    return next((name for name in LEVELS if profile.endswith("_" + name)), None)


def continuous(report, nominal_ate):
    ate = nested(report, "trajectory", "ate_rmse_m")
    completeness = report.get("trajectory_completeness")
    # An explicit V3 engineering criterion, not a paper claim: the estimator
    # must keep publishing for >=90% of the frozen route, preserve transaction
    # integrity, and remain within 2x the nominal aligned ATE.
    return bool(
        report.get("pass_invariants")
        and completeness is not None and completeness >= 0.90
        and report.get("trajectory_continuous", False)
        and ate is not None and nominal_ate is not None
        and ate <= max(2.0 * nominal_ate, nominal_ate + 0.10)
    )


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--matrix-root", required=True)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    root = Path(args.matrix_root)
    rows = list(csv.DictReader(
        (root / "matrix_manifest.tsv").open(encoding="utf-8"),
        delimiter="\t",
    ))
    reports = {}
    missing = []
    for row in rows:
        path = Path(row["output"]) / "robustness_report.json"
        if not path.exists():
            missing.append({**row, "report": str(path)})
            continue
        reports[(row["profile"], row["frs"])] = json.loads(
            path.read_text(encoding="utf-8")
        )
    nominal_on = reports.get(("nominal", "on"), {})
    nominal_ate = nested(nominal_on, "trajectory", "ate_rmse_m")
    comparisons = {}
    boundaries = {}
    danger = []
    for profile in sorted({key[0] for key in reports}):
        on = reports.get((profile, "on"))
        off = reports.get((profile, "off"))
        if on:
            category = profile.rsplit("_", 1)[0] if level(profile) else profile
            if level(profile) and continuous(on, nominal_ate):
                previous = boundaries.get(category)
                if previous is None or LEVELS[level(profile)] > LEVELS[previous]:
                    boundaries[category] = level(profile)
        if not on or not off:
            continue
        on_ate = nested(on, "trajectory", "ate_rmse_m")
        off_ate = nested(off, "trajectory", "ate_rmse_m")
        on_comp = on.get("trajectory_completeness")
        off_comp = off.get("trajectory_completeness")
        comparison = {
            "frs_on_continuous": continuous(on, nominal_ate),
            "frs_off_continuous": continuous(off, nominal_ate),
            "ate_gain_m": (
                off_ate - on_ate if on_ate is not None and off_ate is not None else None
            ),
            "completeness_gain": (
                on_comp - off_comp if on_comp is not None and off_comp is not None else None
            ),
            "frs_on_errors": on.get("errors", {}),
            "frs_off_errors": off.get("errors", {}),
            "frs_on_solver_median_ms": nested(on, "solver_ms", "median"),
            "frs_off_solver_median_ms": nested(off, "solver_ms", "median"),
        }
        comparisons[profile] = comparison
        danger_score = 0.0
        if not comparison["frs_off_continuous"]:
            danger_score += 10.0
        if comparison["ate_gain_m"] is not None:
            danger_score += max(0.0, comparison["ate_gain_m"])
        if comparison["completeness_gain"] is not None:
            danger_score += max(0.0, comparison["completeness_gain"])
        danger.append((danger_score, profile))
    solver_values = [
        nested(report, "solver_ms", "median")
        for report in reports.values()
        if nested(report, "solver_ms", "median") is not None
    ]
    output = {
        "schema_version": 1,
        "run_count": len(reports),
        "missing_reports": missing,
        "nominal_ate_rmse_m": nominal_ate,
        "continuity_criterion": {
            "trajectory_completeness_minimum": 0.90,
            "maximum_odom_gap_s": 1.0,
            "transaction_errors_integrity_rejects_rollbacks": 0,
            "ate_limit": "max(2 * nominal, nominal + 0.10 m)",
            "status": "V3 engineering criterion; not a paper threshold",
        },
        "frs_ab": comparisons,
        "single_fault_boundaries": boundaries,
        "most_dangerous_profile": max(danger)[1] if danger else None,
        "solver_median_across_runs_ms": (
            statistics.median(solver_values) if solver_values else None
        ),
        "all_invariants_pass": all(
            report.get("pass_invariants", False) for report in reports.values()
        ),
        "reports": {
            f"{profile}:{frs}": report for (profile, frs), report in reports.items()
        },
    }
    Path(args.output).write_text(
        json.dumps(output, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(json.dumps({
        "run_count": output["run_count"],
        "missing": len(missing),
        "boundaries": boundaries,
        "most_dangerous_profile": output["most_dangerous_profile"],
        "all_invariants_pass": output["all_invariants_pass"],
    }, sort_keys=True))


if __name__ == "__main__":
    main()
