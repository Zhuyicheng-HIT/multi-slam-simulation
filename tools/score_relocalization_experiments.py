#!/usr/bin/env python3
"""Score verified relocalization runs without hiding evidence confidence."""

from __future__ import annotations

import argparse
import json
import math
from collections import defaultdict
from pathlib import Path


def clamp(value, lower=0.0, upper=1.0):
    return min(upper, max(lower, float(value)))


def inverse_linear(value, good, bad):
    if bad <= good:
        raise ValueError("inverse score limits must be increasing")
    return clamp((bad - float(value)) / (bad - good))


def load_json(path):
    with path.open("r", encoding="utf-8") as stream:
        return json.load(stream)


def runtime_is_complete(termination_reason):
    return str(termination_reason) in {"duration_complete", "early_landing"}


def runtime_evidence_is_complete(termination_reason, landed_disarmed):
    reason = str(termination_reason)
    if reason == "duration_complete":
        return True
    return reason == "early_landing" and bool(landed_disarmed)


def backend_integrity_counts(backend):
    post_reset_fields = {
        "optimization_errors": (
            "relocalization_post_reset_optimization_errors"
        ),
        "optimization_rollbacks": (
            "relocalization_post_reset_optimization_rollbacks"
        ),
        "native_consumed_without_state_commit": (
            "relocalization_post_reset_native_without_commit"
        ),
        "native_worker_errors": (
            "relocalization_post_reset_native_worker_errors"
        ),
        "native_worker_queue_discarded": (
            "relocalization_post_reset_native_queue_discarded"
        ),
        "native_worker_queue_overflow": (
            "relocalization_post_reset_native_queue_overflow"
        ),
    }
    post_reset = {
        name: int(backend.get(field, -1))
        for name, field in post_reset_fields.items()
    }
    if all(value >= 0 for value in post_reset.values()):
        return post_reset, "post_relocalization_delta"
    return {
        name: int(backend.get(name, 0)) for name in post_reset_fields
    }, "run_cumulative_fallback"


def applied_initialization_policy(backend, kind):
    explicit = backend.get(f"relocalization_{kind}_policy_applied")
    if explicit not in (None, ""):
        return str(explicit), "explicit_diagnostic"
    configured = str(backend.get(
        f"relocalization_{kind}_policy", "unknown"
    ))
    reason = str(backend.get("relocalization_stationary_reason", "unknown"))
    if kind == "velocity":
        if configured == "rotate":
            return "rotate", "inferred_legacy_diagnostic"
        if configured == "stationary_zero":
            return (
                "stationary_zero" if reason == "ok" else "rotate",
                "inferred_legacy_diagnostic",
            )
    if kind == "bias":
        if configured == "preserve":
            return "preserve", "inferred_legacy_diagnostic"
        if configured == "stationary_imu":
            return (
                "stationary_imu" if reason == "ok" else "preserve",
                "inferred_legacy_diagnostic",
            )
    return "not_recorded", "missing"


def parse_run_spec(spec):
    fields = str(spec).split("=", 2)
    if len(fields) != 3:
        raise ValueError("run must be SCENARIO=LABEL=LOG_DIR")
    scenario, label, directory = fields
    if not scenario.strip() or not label.strip() or not directory.strip():
        raise ValueError("run scenario, label, and directory must be non-empty")
    return scenario.strip(), label.strip(), Path(directory).expanduser().resolve()


def extract_run(scenario, label, directory):
    trigger = load_json(directory / "relocalization_trigger.json")
    accuracy = load_json(directory / "unified_accuracy.json")
    runtime = load_json(directory / "unified_runtime_metrics.json")
    transactions = list(trigger.get("completed_transactions", []))
    transaction = transactions[-1] if transactions else {}
    causal = accuracy.get("causal_ate", {})
    three_d = causal.get("three_dimensional", {})
    endpoint = causal.get("endpoint_error_m", {})
    backend = runtime.get("backend_latest", {})
    velocity_applied, velocity_evidence = applied_initialization_policy(
        backend, "velocity"
    )
    bias_applied, bias_evidence = applied_initialization_policy(backend, "bias")
    route_log = directory / "guided_s_curve_waypoints.log"
    landed = False
    if route_log.is_file():
        landed = "LAND completed and FCU disarm confirmed" in route_log.read_text(
            encoding="utf-8", errors="replace"
        )
    success = bool(trigger.get("success")) and bool(transactions)
    termination_reason = str(runtime.get("termination_reason", "missing"))
    integrity_counts, integrity_evidence = backend_integrity_counts(backend)
    whole_run_integrity_counts = {
        "optimization_errors": int(backend.get("optimization_errors", 0)),
        "optimization_rollbacks": int(backend.get(
            "optimization_rollbacks", 0
        )),
        "native_consumed_without_state_commit": int(backend.get(
            "native_consumed_without_state_commit", 0
        )),
        "native_worker_errors": int(backend.get("native_worker_errors", 0)),
        "native_worker_queue_discarded": int(backend.get(
            "native_worker_queue_discarded", 0
        )),
        "native_worker_queue_overflow": int(backend.get(
            "native_worker_queue_overflow", 0
        )),
    }
    optimization_errors = integrity_counts["optimization_errors"]
    optimization_rollbacks = integrity_counts["optimization_rollbacks"]
    native_without_commit = integrity_counts[
        "native_consumed_without_state_commit"
    ]
    native_worker_errors = integrity_counts["native_worker_errors"]
    native_queue_discarded = integrity_counts[
        "native_worker_queue_discarded"
    ]
    native_queue_overflow = integrity_counts["native_worker_queue_overflow"]
    backend_integrity_clean = all(value == 0 for value in (
        optimization_errors,
        optimization_rollbacks,
        native_without_commit,
        native_worker_errors,
        native_queue_discarded,
        native_queue_overflow,
    ))
    return {
        "scenario": scenario,
        "label": label,
        "directory": str(directory),
        "success": success,
        "reason": str(trigger.get("reason", "missing")),
        "candidate_id": transaction.get("candidate_id"),
        "motion_profile": str(
            transaction.get("motion_profile", trigger.get("motion_profile", "hold"))
        ),
        "motion_steps": int(transaction.get("motion_steps_executed", 0)),
        "motion_distance_m": float(transaction.get("motion_distance_m", 0.0)),
        "motion_duration_s": float(transaction.get("motion_duration_s", 0.0)),
        "search_wall_s": float(transaction.get(
            "final_search_wall_s", transaction.get("recovery_wall_s", math.inf)
        )),
        "recovery_wall_s": float(transaction.get("recovery_wall_s", math.inf)),
        "rmse_m": float(three_d.get("rmse_m", math.inf)),
        "p95_m": float(three_d.get("p95_m", math.inf)),
        "max_m": float(three_d.get("max_m", math.inf)),
        "endpoint_m": float(endpoint.get("norm", math.inf)),
        "accuracy_passed": bool(accuracy.get("acceptance", {}).get("passed")),
        "runtime_complete": runtime_evidence_is_complete(
            termination_reason, landed
        ),
        "runtime_termination": termination_reason,
        "accuracy_sim_duration_s": float(accuracy.get("sim_duration_s", 0.0)),
        "runtime_sim_duration_s": float(runtime.get("sim_duration_s", 0.0)),
        "landed_disarmed": landed,
        "optimization_errors": optimization_errors,
        "optimization_rollbacks": optimization_rollbacks,
        "native_consumed_without_state_commit": native_without_commit,
        "native_worker_errors": native_worker_errors,
        "native_worker_queue_discarded": native_queue_discarded,
        "native_worker_queue_overflow": native_queue_overflow,
        "backend_integrity_clean": backend_integrity_clean,
        "backend_integrity_evidence": integrity_evidence,
        "whole_run_backend_integrity_clean": all(
            value == 0 for value in whole_run_integrity_counts.values()
        ),
        "whole_run_backend_integrity_counts": whole_run_integrity_counts,
        "velocity_policy_requested": str(backend.get(
            "relocalization_velocity_policy_requested",
            backend.get("relocalization_velocity_policy", "unknown"),
        )),
        "velocity_policy_applied": velocity_applied,
        "bias_policy_requested": str(backend.get(
            "relocalization_bias_policy_requested",
            backend.get("relocalization_bias_policy", "unknown"),
        )),
        "bias_policy_applied": bias_applied,
        "initialization_evidence": (
            velocity_evidence
            if velocity_evidence == bias_evidence
            else f"velocity:{velocity_evidence};bias:{bias_evidence}"
        ),
        "stationary_reason": str(backend.get(
            "relocalization_stationary_reason", "not_recorded"
        )),
    }


def score_run(run):
    success_score = 35.0 if run["success"] else 0.0
    accuracy_score = 35.0 * (
        0.45 * inverse_linear(run["rmse_m"], 0.03, 0.20)
        + 0.35 * inverse_linear(run["p95_m"], 0.05, 0.30)
        + 0.20 * inverse_linear(run["endpoint_m"], 0.03, 0.25)
    )
    latency_score = 15.0 * inverse_linear(run["recovery_wall_s"], 5.0, 35.0)
    motion_score = 10.0 * (
        0.55 * inverse_linear(run["motion_distance_m"], 0.0, 2.5)
        + 0.45 * inverse_linear(run["motion_duration_s"], 0.0, 20.0)
    )
    safety_score = 5.0 * (
        int(run["runtime_complete"])
        + int(run["landed_disarmed"])
    ) / 2.0
    integrity_events = (
        int(run.get("optimization_errors", 0))
        + int(run.get("native_worker_errors", 0))
        + int(run.get("native_worker_queue_discarded", 0))
        + int(run.get("native_worker_queue_overflow", 0))
        + max(
            int(run.get("optimization_rollbacks", 0)),
            int(run.get("native_consumed_without_state_commit", 0)),
        )
    )
    integrity_penalty = 15.0 * clamp(integrity_events / 5.0)
    if not run["success"]:
        accuracy_score = 0.0
        latency_score = 0.0
    raw_score = (
        success_score + accuracy_score + latency_score + motion_score
        + safety_score
    )
    score = max(0.0, raw_score - integrity_penalty)
    relocalization_candidate_eligible = all((
        bool(run["success"]),
        bool(run.get("accuracy_passed", True)),
        bool(run["runtime_complete"]),
        bool(run["landed_disarmed"]),
        integrity_events == 0,
    ))
    deployment_eligible = (
        relocalization_candidate_eligible
        and bool(run.get("whole_run_backend_integrity_clean", False))
    )
    output = dict(run)
    output.update({
        "score": round(score, 3),
        "raw_performance_score": round(raw_score, 3),
        "backend_integrity_events": integrity_events,
        "relocalization_candidate_eligible": (
            relocalization_candidate_eligible
        ),
        "deployment_eligible": deployment_eligible,
        "score_components": {
            "success": round(success_score, 3),
            "accuracy": round(accuracy_score, 3),
            "latency": round(latency_score, 3),
            "motion_cost": round(motion_score, 3),
            "safety_completion": round(safety_score, 3),
            "backend_integrity_penalty": round(-integrity_penalty, 3),
        },
    })
    return output


def summarize(scored):
    groups = defaultdict(list)
    for run in scored:
        groups[(run["scenario"], run["label"])].append(run)
    summaries = []
    for (scenario, label), runs in groups.items():
        count = len(runs)
        successes = sum(run["success"] for run in runs)
        scores = [run["score"] for run in runs]
        candidate_ids = sorted({
            run["candidate_id"] for run in runs if run["candidate_id"] is not None
        })
        complete_evidence = all(
            run.get("runtime_complete", False)
            and run.get("landed_disarmed", False)
            for run in runs
        )
        deployment_eligible = sum(
            bool(run.get("deployment_eligible", False)) for run in runs
        )
        candidate_eligible = sum(
            bool(run.get("relocalization_candidate_eligible", False))
            for run in runs
        )
        summaries.append({
            "scenario": scenario,
            "label": label,
            "run_count": count,
            "success_rate": successes / count,
            "mean_score": sum(scores) / count,
            "minimum_score": min(scores),
            "candidate_ids": candidate_ids,
            "deployment_eligible_rate": deployment_eligible / count,
            "relocalization_candidate_eligible_rate": (
                candidate_eligible / count
            ),
            "evidence_confidence": (
                "partial_run" if not complete_evidence
                else "screening_only" if count < 3
                else "preliminary" if count < 5
                else "comparative"
            ),
        })
    return sorted(
        summaries,
        key=lambda item: (item["scenario"], -item["mean_score"], item["label"]),
    )


def markdown_report(report):
    lines = [
        "# Relocalization experiment scores",
        "",
        "Performance score is 100 points: success 35, accuracy 35, recovery "
        "latency 15, motion cost 10, and safe completion 5. Up to 15 points are "
        "then deducted for backend integrity events. Evidence confidence and "
        "relocalization-candidate and whole-system deployment eligibility are "
        "reported separately.",
        "",
        "| Scenario | Logic | Runs | Success | Mean | Minimum | Reloc eligible | Deploy eligible | Confidence | Candidates |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | ---: | --- | --- |",
    ]
    for summary in report["summaries"]:
        candidates = ",".join(str(value) for value in summary["candidate_ids"]) or "-"
        lines.append(
            f"| {summary['scenario']} | {summary['label']} | "
            f"{summary['run_count']} | {summary['success_rate']:.0%} | "
            f"{summary['mean_score']:.1f} | {summary['minimum_score']:.1f} | "
            f"{summary['relocalization_candidate_eligible_rate']:.0%} | "
            f"{summary['deployment_eligible_rate']:.0%} | "
            f"{summary['evidence_confidence']} | {candidates} |"
        )
    lines.extend([
        "",
        "## Runs",
        "",
        "| Scenario | Logic | Score | RMSE m | P95 m | Endpoint m | Recovery s | Motion m/s | Reloc integrity | Reloc eligible | Deploy eligible | Applied init | Evidence |",
        "| --- | --- | ---: | ---: | ---: | ---: | ---: | --- | ---: | ---: | ---: | --- | --- |",
    ])
    for run in sorted(
        report["runs"], key=lambda item: (item["scenario"], -item["score"])
    ):
        lines.append(
            f"| {run['scenario']} | {run['label']} | {run['score']:.1f} | "
            f"{run['rmse_m']:.4f} | {run['p95_m']:.4f} | "
            f"{run['endpoint_m']:.4f} | {run['recovery_wall_s']:.2f} | "
            f"{run['motion_distance_m']:.2f}/{run['motion_duration_s']:.2f} | "
            f"{run['backend_integrity_events']} | "
            f"{'yes' if run['relocalization_candidate_eligible'] else 'no'} | "
            f"{'yes' if run['deployment_eligible'] else 'no'} | "
            f"{run['velocity_policy_applied']}+{run['bias_policy_applied']} | "
            f"{run['initialization_evidence']};"
            f"{run['backend_integrity_evidence']};"
            f"{run['runtime_termination']} |"
        )
    lines.extend([
        "",
        "A one-run result is only an initial screening signal. Scenario-specific "
        "recommendations require repeated runs and must not be inferred from the "
        "aggregate score alone.",
        "",
    ])
    return "\n".join(lines)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--run", action="append", required=True,
        help="SCENARIO=LABEL=LOG_DIR; repeat for each completed run",
    )
    parser.add_argument("--output", required=True)
    parser.add_argument("--markdown-output")
    args = parser.parse_args()
    scored = [score_run(extract_run(*parse_run_spec(spec))) for spec in args.run]
    report = {"schema_version": 1, "runs": scored, "summaries": summarize(scored)}
    output = Path(args.output).expanduser().resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(report, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    if args.markdown_output:
        markdown = Path(args.markdown_output).expanduser().resolve()
        markdown.parent.mkdir(parents=True, exist_ok=True)
        markdown.write_text(markdown_report(report), encoding="utf-8")
    print(json.dumps({
        "runs": len(scored),
        "summaries": len(report["summaries"]),
        "output": str(output),
    }, sort_keys=True))


if __name__ == "__main__":
    main()
