
#!/usr/bin/env python3
"""Offline validation for RTAB-Map mapping and cross-session localization."""

import argparse
import csv
import hashlib
import json
import math
import re
import sqlite3
import struct
from pathlib import Path

import numpy as np
import yaml


def wrap_angle(value):
    return math.atan2(math.sin(value), math.cos(value))


def sha256(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def rows(path):
    if not path.is_file():
        return []
    with path.open(newline="", encoding="utf-8") as handle:
        return list(csv.DictReader(handle))


def number(row, key, default=0.0):
    try:
        return float(row.get(key, default))
    except (TypeError, ValueError):
        return float(default)


def integer(row, key, default=0):
    return int(round(number(row, key, default)))


def load_context(path):
    result = {}
    if not path.is_file():
        return result
    for line in path.read_text(encoding="utf-8").splitlines():
        if "=" in line:
            key, value = line.split("=", 1)
            result[key] = value
    return result


def log_counts(path):
    text = (path.read_text(encoding="utf-8", errors="ignore")
            if path.is_file() else "")
    return {
        "lost": len(re.findall(r"Odometry lost|lost=true", text, re.I)),
        "reset": len(re.findall(
            r"Odometry automatically reset|resetting odometry|Odometry reset",
            text, re.I)),
    }


def lost_transitions(health):
    count = 0
    previous = False
    for row in health:
        current = bool(integer(row, "lost"))
        if current and not previous:
            count += 1
        previous = current
    return count


def database_summary(database):
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    result = {
        "node_count": connection.execute("SELECT count(*) FROM Node").fetchone()[0],
        "word_count": connection.execute("SELECT count(*) FROM Word").fetchone()[0],
        "feature_count": connection.execute(
            "SELECT count(*) FROM Feature").fetchone()[0],
        "map_ids": [row[0] for row in connection.execute(
            "SELECT DISTINCT map_id FROM Node ORDER BY map_id")],
        "global_closure_links": connection.execute(
            "SELECT count(*) FROM Link WHERE type=1").fetchone()[0],
        "local_space_links": connection.execute(
            "SELECT count(*) FROM Link WHERE type=2").fetchone()[0],
    }
    connection.close()
    return result


def node_pose(database, node_id):
    if node_id <= 0:
        return None
    connection = sqlite3.connect(f"file:{database}?mode=ro", uri=True)
    row = connection.execute(
        "SELECT map_id,pose FROM Node WHERE id=?", (node_id,)).fetchone()
    connection.close()
    if row is None or row[1] is None or len(row[1]) != 48:
        return None
    values = struct.unpack("<12f", row[1])
    return {
        "node_id": node_id, "map_id": int(row[0]),
        "x": float(values[3]), "y": float(values[7]),
        "z": float(values[11]),
        "yaw_rad": math.atan2(values[4], values[0]),
    }


def finite_summary(values):
    array = np.asarray(values, dtype=float)
    array = array[np.isfinite(array)]
    if not len(array):
        return {}
    return {
        "count": int(len(array)), "mean": float(np.mean(array)),
        "median": float(np.median(array)),
        "p95": float(np.percentile(array, 95)),
        "min": float(np.min(array)), "max": float(np.max(array)),
    }


def analyze_reference(args):
    monitor = Path(args.monitor_dir)
    database = Path(args.database).resolve()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    gt = rows(monitor / "ground_truth.csv")
    info = rows(monitor / "info_events.csv")
    health = rows(monitor / "odometry_health.csv")
    if not gt:
        raise RuntimeError("Reference monitor did not capture ground truth")
    origin_row = gt[0]
    origin = {
        "x": number(origin_row, "x"), "y": number(origin_row, "y"),
        "z": number(origin_row, "z"),
        "yaw_rad": number(origin_row, "yaw_rad"),
    }
    db = database_summary(database)
    accepted = [row for row in info if (
        integer(row, "loop_closure_id") > 0
        or integer(row, "proximity_detection_id") > 0)]
    geometric = [row for row in accepted
                 if number(row, "visual_inliers") >= 10.0]
    logs = log_counts(Path(args.rtabmap_log))
    lost = lost_transitions(health) + logs["lost"]
    validation = bool(
        db["node_count"] > 0 and db["word_count"] > 0
        and db["global_closure_links"] > 0 and geometric
        and lost == 0 and logs["reset"] == 0)
    summary = {
        "mode": "reference", "validation_pass": validation,
        "database_path": str(database), "database_sha256": sha256(database),
        "database_size_bytes": database.stat().st_size,
        "origin": origin, "database": db,
        "info_event_count": len(info),
        "accepted_event_count": len(accepted),
        "geometrically_validated_event_count": len(geometric),
        "accepted_node_ids": sorted({
            integer(row, "loop_closure_id")
            or integer(row, "proximity_detection_id") for row in geometric}),
        "maximum_geometry_inliers": max(
            (number(row, "visual_inliers") for row in accepted), default=0.0),
        "lost_events": lost, "reset_events": logs["reset"],
        "failure_reasons": [],
    }
    if not validation:
        for condition, reason in (
                (db["node_count"] <= 0, "empty node table"),
                (db["word_count"] <= 0, "empty visual-word table"),
                (db["global_closure_links"] <= 0,
                 "no GlobalClosure link in database"),
                (not geometric, "no geometrically validated closure event"),
                (lost > 0, "odometry lost"),
                (logs["reset"] > 0, "odometry reset")):
            if condition:
                summary["failure_reasons"].append(reason)
    (output / "reference_metadata.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    lines = [
        "# Session 1 reference map validation", "",
        f"- Validation: **{'PASS' if validation else 'FAIL'}**",
        f"- Database SHA-256: `{summary['database_sha256']}`",
        f"- Nodes / words / features: {db['node_count']} / "
        f"{db['word_count']} / {db['feature_count']}",
        f"- GlobalClosure database links: {db['global_closure_links']}",
        f"- Geometrically validated live events: {len(geometric)}",
        f"- Maximum geometry inliers: {summary['maximum_geometry_inliers']:.0f}",
        f"- lost / reset: {lost} / {logs['reset']}",
    ]
    if summary["failure_reasons"]:
        lines.extend(["", "Failure reasons: " + "; ".join(
            summary["failure_reasons"])])
    (output / "reference_summary.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")
    print(output / "reference_summary.md")
    return 0 if validation else 1


def nearest_alignment(samples, elapsed):
    return min(samples, key=lambda row: abs(number(row, "elapsed_s") - elapsed))


def relative_ground_truth(row, origin):
    dx = number(row, "gt_x") - float(origin["x"])
    dy = number(row, "gt_y") - float(origin["y"])
    c = math.cos(float(origin["yaw_rad"]))
    s = math.sin(float(origin["yaw_rad"]))
    return (
        c * dx + s * dy,
        -s * dx + c * dy,
        number(row, "gt_z") - float(origin["z"]),
        wrap_angle(number(row, "gt_yaw_rad") - float(origin["yaw_rad"])),
    )


def alignment_error(row, origin):
    ground_truth = relative_ground_truth(row, origin)
    position = math.sqrt(
        (number(row, "map_x") - ground_truth[0]) ** 2
        + (number(row, "map_y") - ground_truth[1]) ** 2
        + (number(row, "map_z") - ground_truth[2]) ** 2)
    yaw_error = abs(math.degrees(wrap_angle(
        number(row, "map_yaw_rad") - ground_truth[3])))
    return position, yaw_error, ground_truth


def analyze_session(args):
    monitor = Path(args.monitor_dir)
    database = Path(args.database).resolve()
    output = Path(args.output_dir)
    output.mkdir(parents=True, exist_ok=True)
    reference = json.loads(Path(args.reference_metadata).read_text(
        encoding="utf-8"))
    config = yaml.safe_load(Path(args.conditions_config).read_text(
        encoding="utf-8"))
    thresholds = config["success"]
    condition = config["conditions"][args.condition]
    info = rows(monitor / "info_events.csv")
    alignment = rows(monitor / "alignment_samples.csv")
    health = rows(monitor / "odometry_health.csv")
    stages = rows(monitor / "stage_events.csv")
    context = load_context(Path(args.run_context))
    logs = log_counts(Path(args.rtabmap_log))
    db = database_summary(database)
    min_inliers = float(thresholds["min_geometry_inliers"])
    first_info_s = number(info[0], "elapsed_s") if info else None
    candidate_events = [row for row in info if integer(row, "candidate_id") > 0]
    accepted_all = [row for row in info if (
        integer(row, "loop_closure_id") > 0
        or integer(row, "proximity_detection_id") > 0)]
    accepted_geometry = [row for row in accepted_all
                         if number(row, "visual_inliers") >= min_inliers]
    accepted = accepted_geometry[0] if accepted_geometry else None
    first_candidate = candidate_events[0] if candidate_events else None
    accepted_s = number(accepted, "elapsed_s") if accepted else None
    position_threshold = float(thresholds["max_stable_position_error_m"])
    yaw_threshold = float(thresholds["max_stable_yaw_error_deg"])
    stable_required = int(thresholds["stable_sample_count"])
    origin = reference["origin"]

    evaluated = []
    for row in alignment:
        if not math.isfinite(number(row, "gt_x", float("nan"))):
            continue
        position_error, yaw_error, gt_relative = alignment_error(row, origin)
        evaluated.append((row, position_error, yaw_error, gt_relative))
    after_accept = [item for item in evaluated
                    if accepted_s is not None
                    and number(item[0], "elapsed_s") >= accepted_s]
    initial = after_accept[0] if after_accept else None
    stable = None
    consecutive = 0
    for item in after_accept:
        if item[1] <= position_threshold and item[2] <= yaw_threshold:
            consecutive += 1
            if consecutive >= stable_required:
                stable = item
                break
        else:
            consecutive = 0
    final_window = after_accept[-min(20, len(after_accept)):] if after_accept else []
    final_position = finite_summary([item[1] for item in final_window])
    final_yaw = finite_summary([item[2] for item in final_window])

    accepted_id = 0
    if accepted:
        accepted_id = (integer(accepted, "loop_closure_id")
                       or integer(accepted, "proximity_detection_id"))
    matched_pose = node_pose(database, accepted_id)
    candidate_distance = None
    if matched_pose and accepted and evaluated:
        sample = nearest_alignment(alignment, accepted_s)
        gt_relative = relative_ground_truth(sample, origin)
        candidate_distance = math.dist(
            (matched_pose["x"], matched_pose["y"], matched_pose["z"]),
            gt_relative[:3])

    map_jumps = []
    abnormal_jumps = []
    previous = None
    for item in after_accept:
        elapsed = number(item[0], "elapsed_s")
        position = tuple(number(item[0], key) for key in (
            "map_x", "map_y", "map_z"))
        if previous is not None:
            jump = math.dist(previous[1], position)
            map_jumps.append(jump)
            if (elapsed >= accepted_s + 2.0
                    and jump > float(thresholds["abnormal_jump_m"])):
                abnormal_jumps.append(jump)
        previous = (elapsed, position)
    map_to_odom_jumps = []
    previous_transform = None
    for row in info:
        transform = tuple(number(row, key) for key in (
            "map_to_odom_x", "map_to_odom_y", "map_to_odom_z"))
        if previous_transform is not None:
            map_to_odom_jumps.append(math.dist(previous_transform, transform))
        previous_transform = transform

    lost = lost_transitions(health) + logs["lost"]
    reset = logs["reset"]
    motion_exit = int(context.get("motion_exit_code", "1") or 1)
    reference_hash_before = context.get("reference_sha256_before", "")
    reference_hash_after = context.get("reference_sha256_after", "")
    mother_unchanged = bool(
        reference_hash_before and reference_hash_before == reference_hash_after
        and reference_hash_before == reference["database_sha256"])
    complete = bool(
        info and alignment and health and stages and motion_exit == 0
        and db["node_count"] > 0 and db["word_count"] > 0
        and mother_unchanged)
    false_relocalization = bool(accepted and (
        stable is None
        or (candidate_distance is not None
            and candidate_distance > float(
                thresholds["max_candidate_node_distance_m"]))
        or abnormal_jumps))
    success = bool(
        complete and accepted and stable is not None
        and not false_relocalization and lost == 0 and reset == 0)
    reasons = []
    if not info:
        reasons.append("no RTAB-Map Info events")
    if not alignment:
        reasons.append("no synchronized RTAB/GT alignment samples")
    if motion_exit != 0:
        reasons.append(f"motion exit code {motion_exit}")
    if not mother_unchanged:
        reasons.append("reference mother hash changed or was not verified")
    if accepted is None:
        reasons.append("no geometry-validated relocalization event")
    if accepted is not None and stable is None:
        reasons.append("map alignment did not meet stable error bounds")
    if candidate_distance is not None and candidate_distance > float(
            thresholds["max_candidate_node_distance_m"]):
        reasons.append("accepted node is geometrically inconsistent with GT")
    if abnormal_jumps:
        reasons.append("post-alignment map pose contains >1 m jump")
    if lost:
        reasons.append("visual odometry lost")
    if reset:
        reasons.append("visual odometry reset")

    summary = {
        "mode": "session", "condition": args.condition,
        "condition_definition": condition,
        "validation_complete": complete,
        "relocalization_success": success,
        "false_relocalization": false_relocalization,
        "failure_reasons": reasons,
        "database": db, "database_sha256_after": sha256(database),
        "reference_mother_unchanged": mother_unchanged,
        "info_event_count": len(info),
        "candidate_event_count": len(candidate_events),
        "accepted_event_count": len(accepted_all),
        "geometry_accepted_event_count": len(accepted_geometry),
        "rejected_candidate_count": sum(
            number(row, "rejected_hypothesis") > 0 for row in info),
        "first_candidate_id": integer(first_candidate, "candidate_id")
        if first_candidate else 0,
        "accepted_node_id": accepted_id,
        "accepted_map_id": integer(accepted, "map_id") if accepted else None,
        "matched_node_pose": matched_pose,
        "matched_node_distance_to_gt_m": candidate_distance,
        "geometry_inliers": number(accepted, "visual_inliers")
        if accepted else 0.0,
        "visual_matches": number(accepted, "visual_matches")
        if accepted else 0.0,
        "visual_words": number(accepted, "visual_words")
        if accepted else 0.0,
        "closure_transform": ({
            key: number(accepted, key) for key in (
                "closure_x", "closure_y", "closure_z", "closure_yaw_rad")
        } if accepted else None),
        "time_to_first_candidate_s": (
            number(first_candidate, "elapsed_s") - first_info_s
            if first_candidate and first_info_s is not None else None),
        "time_to_accepted_closure_s": (
            accepted_s - first_info_s
            if accepted_s is not None and first_info_s is not None else None),
        "time_to_stable_alignment_s": (
            number(stable[0], "elapsed_s") - first_info_s
            if stable is not None and first_info_s is not None else None),
        "initial_position_error_m": initial[1] if initial else None,
        "initial_yaw_error_deg": initial[2] if initial else None,
        "stable_position_error_m": stable[1] if stable else None,
        "stable_yaw_error_deg": stable[2] if stable else None,
        "final_position_error_m": final_position,
        "final_yaw_error_deg": final_yaw,
        "maximum_map_pose_step_m": max(map_jumps, default=0.0),
        "maximum_map_to_odom_jump_m": max(map_to_odom_jumps, default=0.0),
        "abnormal_post_alignment_jump_count": len(abnormal_jumps),
        "lost_events": lost, "reset_events": reset,
        "tf_backward_jumps": sum(
            integer(row, "backward_jump") for row in rows(
                monitor / "tf_events.csv")),
    }
    (output / "relocalization_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True), encoding="utf-8")
    status = "SUCCESS" if success else ("FAIL" if complete else "INVALID")
    lines = [
        f"# Cross-session result: {args.condition}", "",
        f"- Status: **{status}**",
        f"- Candidate / accepted / rejected events: {len(candidate_events)} / "
        f"{len(accepted_geometry)} / {summary['rejected_candidate_count']}",
        f"- Accepted node / map: {accepted_id} / {summary['accepted_map_id']}",
        f"- Geometry inliers / visual words: {summary['geometry_inliers']:.0f} / "
        f"{summary['visual_words']:.0f}",
        f"- Candidate / accepted / stable latency: "
        f"{summary['time_to_first_candidate_s']} / "
        f"{summary['time_to_accepted_closure_s']} / "
        f"{summary['time_to_stable_alignment_s']} s",
        f"- Stable position / yaw error: {summary['stable_position_error_m']} m / "
        f"{summary['stable_yaw_error_deg']} deg",
        f"- Max map->odom jump: {summary['maximum_map_to_odom_jump_m']:.3f} m",
        f"- Abnormal post-alignment >1 m jumps: "
        f"{summary['abnormal_post_alignment_jump_count']}",
        f"- lost / reset / TF backward: {lost} / {reset} / "
        f"{summary['tf_backward_jumps']}",
        f"- Reference mother unchanged: {mother_unchanged}",
    ]
    if reasons:
        lines.extend(["", "Reasons: " + "; ".join(reasons)])
    (output / "relocalization_summary.md").write_text(
        "\n".join(lines) + "\n", encoding="utf-8")
    print(output / "relocalization_summary.md")
    return 0 if complete else 2


def main(argv=None):
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="mode", required=True)
    reference = subparsers.add_parser("reference")
    reference.add_argument("monitor_dir")
    reference.add_argument("database")
    reference.add_argument("output_dir")
    reference.add_argument("--rtabmap-log", required=True)
    session = subparsers.add_parser("session")
    session.add_argument("monitor_dir")
    session.add_argument("database")
    session.add_argument("output_dir")
    session.add_argument("--reference-metadata", required=True)
    session.add_argument("--conditions-config", required=True)
    session.add_argument("--condition", required=True)
    session.add_argument("--rtabmap-log", required=True)
    session.add_argument("--run-context", required=True)
    args = parser.parse_args(argv)
    if args.mode == "reference":
        return analyze_reference(args)
    return analyze_session(args)


if __name__ == "__main__":
    raise SystemExit(main())
