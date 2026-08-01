
#!/usr/bin/env python3
"""Read-only, per-run RTAB-Map visual-word and graph diagnostics."""

import argparse
import csv
import hashlib
import math
import re
import sqlite3
import struct
import subprocess
import zlib
from collections import Counter, defaultdict
from pathlib import Path

import numpy as np


LINK_TYPES = {
    0: "Neighbor",
    1: "GlobalClosure",
    2: "LocalSpaceClosure",
    3: "LocalTimeClosure",
    4: "UserClosure",
    5: "VirtualClosure",
    6: "NeighborMerged",
    7: "PosePrior",
    8: "Landmark",
}

SELECTED_STATISTICS = {
    "current_words": "Keypoint/Current_frame/words",
    "dictionary_words": "Keypoint/Dictionary_size/words",
    "indexed_words": "Keypoint/Indexed_words/words",
    "accepted_hypothesis_id": "Loop/Accepted_hypothesis_id/",
    "highest_hypothesis_id": "Loop/Highest_hypothesis_id/",
    "highest_hypothesis_value": "Loop/Highest_hypothesis_value/",
    "hypothesis_ratio": "Loop/Hypothesis_ratio/",
    "rejected_hypothesis": "Loop/RejectedHypothesis/",
    "visual_matches": "Loop/Visual_matches/",
    "visual_inliers": "Loop/Visual_inliers/",
    "visual_words": "Loop/Visual_words/",
    "map_optimization_ms": "Timing/Map_optimization/ms",
    "add_loop_closure_ms": "Timing/Add_loop_closure_link/ms",
    "likelihood_ms": "Timing/Likelihood_computation/ms",
    "posterior_ms": "Timing/Posterior_computation/ms",
    "hypotheses_creation_ms": "Timing/Hypotheses_creation/ms",
    "hypotheses_validation_ms": "Timing/Hypotheses_validation/ms",
    "proximity_by_space_ms": "Timing/Proximity_by_space/ms",
    "memory_update_ms": "Timing/Memory_update/ms",
    "rtab_total_ms": "Timing/Total/ms",
    "ros_total_ms": "RtabmapROS/TimeTotal/ms",
}


def file_hash(path):
    digest = hashlib.sha256()
    with path.open("rb") as handle:
        for chunk in iter(lambda: handle.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def pose_fields(blob):
    if blob is None or len(blob) != 48:
        return ("", "", "", "")
    values = struct.unpack("<12f", blob)
    return (
        values[3], values[7], values[11],
        math.degrees(math.atan2(values[4], values[0])),
    )


def scalar(connection, query, parameters=()):
    return connection.execute(query, parameters).fetchone()[0]


def write_rows(path, fields, rows):
    with path.open("w", newline="", encoding="utf-8") as handle:
        writer = csv.DictWriter(handle, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def decode_statistics(blob):
    if not blob:
        return {}
    try:
        text = zlib.decompress(blob).decode("utf-8")
    except (UnicodeDecodeError, zlib.error):
        return {}
    values = {}
    for item in text.split(";"):
        if ":" not in item:
            continue
        key, value = item.rsplit(":", 1)
        try:
            values[key] = float(value)
        except ValueError:
            continue
    return values


def numeric_summary(values):
    array = np.asarray([
        value for value in values if value is not None and math.isfinite(value)
    ], dtype=float)
    if not len(array):
        return {}
    return {
        "count": len(array), "mean": float(np.mean(array)),
        "median": float(np.median(array)), "p95": float(np.percentile(array, 95)),
        "max": float(np.max(array)),
    }


def quaternion_yaw(sample):
    qx, qy, qz, qw = sample[4:8]
    return math.atan2(
        2.0 * (qw * qz + qx * qy),
        1.0 - 2.0 * (qy * qy + qz * qz),
    )


def read_tum(path):
    samples = []
    if not path.is_file():
        return samples
    for line in path.read_text(encoding="utf-8").splitlines():
        if not line or line.startswith("#"):
            continue
        fields = line.split()
        if len(fields) < 8:
            continue
        samples.append((
            float(fields[0]), *(float(value) for value in fields[1:8]),
            int(fields[8]) if len(fields) > 8 else 0,
        ))
    return samples


def read_ground_truth(path):
    samples = []
    if not path.is_file():
        return samples
    with path.open(newline="", encoding="utf-8") as handle:
        for row in csv.DictReader(handle):
            samples.append((
                float(row["stamp"]), float(row["x"]), float(row["y"]),
                float(row["z"]), float(row["qx"]), float(row["qy"]),
                float(row["qz"]), float(row["qw"]), 0,
            ))
    return samples


def associate(reference, estimate, tolerance_s=0.12):
    if not reference or not estimate:
        return [], []
    reference = sorted(reference)
    stamps = np.asarray([sample[0] for sample in reference])
    matched_reference = []
    matched_estimate = []
    for sample in estimate:
        index = int(np.searchsorted(stamps, sample[0]))
        candidates = [item for item in (index - 1, index)
                      if 0 <= item < len(reference)]
        if not candidates:
            continue
        best = min(candidates, key=lambda item: abs(
            reference[item][0] - sample[0]))
        if abs(reference[best][0] - sample[0]) <= tolerance_s:
            matched_reference.append(reference[best])
            matched_estimate.append(sample)
    return matched_reference, matched_estimate


def trajectory_metrics(reference, estimate):
    reference, estimate = associate(reference, estimate)
    if len(reference) < 3:
        return {"matched": len(reference)}
    reference_position = np.asarray([sample[1:4] for sample in reference])
    estimate_position = np.asarray([sample[1:4] for sample in estimate])
    estimate_center = estimate_position.mean(axis=0)
    reference_center = reference_position.mean(axis=0)
    covariance = ((estimate_position - estimate_center).T
                  @ (reference_position - reference_center))
    left, _, right_t = np.linalg.svd(covariance)
    rotation = left @ right_t
    if np.linalg.det(rotation) < 0.0:
        left[:, -1] *= -1.0
        rotation = left @ right_t
    aligned = (estimate_position - estimate_center) @ rotation + reference_center
    error = aligned - reference_position
    norm = np.linalg.norm(error, axis=1)
    rpe = np.linalg.norm(
        np.diff(aligned, axis=0) - np.diff(reference_position, axis=0), axis=1)
    reference_yaw = np.asarray([quaternion_yaw(sample) for sample in reference])
    estimate_yaw = np.asarray([quaternion_yaw(sample) for sample in estimate])
    yaw_offset = math.atan2(
        np.sin(reference_yaw - estimate_yaw).mean(),
        np.cos(reference_yaw - estimate_yaw).mean())
    yaw_error = np.arctan2(
        np.sin(reference_yaw - estimate_yaw - yaw_offset),
        np.cos(reference_yaw - estimate_yaw - yaw_offset))
    return {
        "matched": len(reference),
        "ate_rmse_m": float(np.sqrt(np.mean(norm ** 2))),
        "rpe_translation_rmse_m": float(np.sqrt(np.mean(rpe ** 2))),
        "horizontal_rmse_m": float(np.sqrt(np.mean(
            np.linalg.norm(error[:, :2], axis=1) ** 2))),
        "height_rmse_m": float(np.sqrt(np.mean(error[:, 2] ** 2))),
        "yaw_rmse_deg": float(np.degrees(np.sqrt(np.mean(yaw_error ** 2)))),
    }


def closure_metrics(samples):
    if len(samples) < 2:
        return {}
    first, last = samples[0], samples[-1]
    return {
        "horizontal_closure_m": math.hypot(last[1] - first[1], last[2] - first[2]),
        "height_closure_m": abs(last[3] - first[3]),
        "yaw_closure_deg": abs(math.degrees(math.atan2(
            math.sin(quaternion_yaw(last) - quaternion_yaw(first)),
            math.cos(quaternion_yaw(last) - quaternion_yaw(first))))),
        "largest_pose_step_m": max((math.dist(a[1:4], b[1:4])
                                    for a, b in zip(samples, samples[1:])),
                                   default=0.0),
    }


def write_pose_accuracy(output_dir, odometry_path, optimized_path):
    ground_truth = read_ground_truth(output_dir / "ground_truth.csv")
    streams = {
        "ground_truth": ground_truth,
        "odometry_only": read_tum(odometry_path),
        "optimized_graph": read_tum(optimized_path),
    }
    rows = []
    for name, samples in streams.items():
        metrics = closure_metrics(samples)
        if name == "ground_truth":
            accuracy = {
                "matched": len(samples), "ate_rmse_m": 0.0,
                "rpe_translation_rmse_m": 0.0, "horizontal_rmse_m": 0.0,
                "height_rmse_m": 0.0, "yaw_rmse_deg": 0.0,
            }
        else:
            accuracy = trajectory_metrics(ground_truth, samples)
        rows.append({"stream": name, **metrics, **accuracy})
    fields = [
        "stream", "matched", "horizontal_closure_m", "height_closure_m",
        "yaw_closure_deg", "largest_pose_step_m", "ate_rmse_m",
        "rpe_translation_rmse_m", "horizontal_rmse_m", "height_rmse_m",
        "yaw_rmse_deg",
    ]
    write_rows(output_dir / "loop_accuracy.csv", fields, [
        {key: row.get(key, "") for key in fields} for row in rows])

    raw_by_id = {sample[8]: sample for sample in streams["odometry_only"]}
    optimized_by_id = {sample[8]: sample for sample in streams["optimized_graph"]}
    corrections = []
    for node_id in sorted(raw_by_id.keys() & optimized_by_id.keys()):
        raw = raw_by_id[node_id]
        optimized = optimized_by_id[node_id]
        corrections.append({
            "node_id": node_id, "stamp": raw[0],
            "translation_correction_m": math.dist(raw[1:4], optimized[1:4]),
            "horizontal_correction_m": math.hypot(
                raw[1] - optimized[1], raw[2] - optimized[2]),
            "height_correction_m": abs(raw[3] - optimized[3]),
            "yaw_correction_deg": abs(math.degrees(math.atan2(
                math.sin(quaternion_yaw(optimized) - quaternion_yaw(raw)),
                math.cos(quaternion_yaw(optimized) - quaternion_yaw(raw))))),
        })
    write_rows(output_dir / "pose_corrections.csv", [
        "node_id", "stamp", "translation_correction_m",
        "horizontal_correction_m", "height_correction_m", "yaw_correction_deg",
    ], corrections)
    return rows, corrections


def export_poses(database, output_dir, opt, output_name, log_handle):
    command = [
        "rtabmap-export", "--poses", "--poses_format", "11",
        "--opt", str(opt), "--output", output_name,
        "--output_dir", str(output_dir), str(database),
    ]
    log_handle.write("$ " + " ".join(command) + "\n")
    try:
        result = subprocess.run(
            command, text=True, stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT, check=False)
    except FileNotFoundError:
        log_handle.write("rtabmap-export was not found on PATH\n\n")
        return 127, output_dir / f"{output_name}.tum"
    log_handle.write(result.stdout + "\n")
    target = output_dir / f"{output_name}.tum"
    for generated in (
            output_dir / f"{output_name}_poses.txt",
            output_dir / f"{output_name}.txt"):
        if generated.is_file():
            generated.replace(target)
            break
    return result.returncode, target


def main(argv=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("database")
    parser.add_argument("output_dir")
    parser.add_argument("--loop-csv", default="")
    args = parser.parse_args(argv)

    database = Path(args.database).expanduser().resolve()
    output_dir = Path(args.output_dir).expanduser().resolve()
    output_dir.mkdir(parents=True, exist_ok=True)
    if not database.is_file():
        raise FileNotFoundError(database)
    hash_before = file_hash(database)
    connection = sqlite3.connect(
        f"file:{database}?mode=ro", uri=True)

    admin = connection.execute(
        "SELECT version FROM Admin LIMIT 1").fetchone()
    info = connection.execute(
        "SELECT dictionary_size, parameters FROM Info LIMIT 1").fetchone()
    nodes = connection.execute(
        "SELECT id,map_id,weight,stamp,pose,label,time_enter FROM Node ORDER BY id"
    ).fetchall()
    features = connection.execute(
        "SELECT node_id,word_id,descriptor_size,length(descriptor) "
        "FROM Feature ORDER BY node_id"
    ).fetchall()
    words = connection.execute(
        "SELECT id,descriptor_size,length(descriptor) FROM Word ORDER BY id"
    ).fetchall()
    links = connection.execute(
        "SELECT from_id,to_id,type,transform FROM Link ORDER BY from_id,to_id,type"
    ).fetchall()
    statistics = connection.execute(
        "SELECT id,stamp,data FROM Statistics ORDER BY id").fetchall()
    node_weight = {node_id: weight for node_id, _, weight, *_ in nodes}

    statistics_by_node = {}
    statistics_rows = []
    for node_id, stamp, blob in statistics:
        decoded = decode_statistics(blob)
        selected = {
            output_name: decoded.get(database_name)
            for output_name, database_name in SELECTED_STATISTICS.items()
        }
        statistics_by_node[node_id] = selected
        statistics_rows.append({
            "node_id": node_id, "stamp": stamp, **selected,
        })
    statistics_fields = ["node_id", "stamp", *SELECTED_STATISTICS.keys()]
    write_rows(output_dir / "statistics_selected.csv", statistics_fields, [
        {key: row.get(key, "") if row.get(key) is not None else ""
         for key in statistics_fields} for row in statistics_rows])

    features_by_node = defaultdict(list)
    first_node_by_word = {}
    for node_id, word_id, descriptor_size, descriptor_bytes in features:
        features_by_node[node_id].append(
            (word_id, descriptor_size or 0, descriptor_bytes or 0))
        if word_id > 0 and word_id not in first_node_by_word:
            first_node_by_word[word_id] = node_id

    node_rows = []
    for node_id, map_id, weight, stamp, pose, label, time_enter in nodes:
        node_features = features_by_node[node_id]
        positive = [item for item in node_features if item[0] > 0]
        distinct = {item[0] for item in positive}
        descriptor_sizes = [item[1] for item in node_features if item[1] > 0]
        x, y, z, yaw = pose_fields(pose)
        node_rows.append({
            "node_id": node_id, "map_id": map_id, "weight": weight,
            "is_keyframe": int(weight is not None and weight >= 0),
            "stamp": stamp, "x": x, "y": y, "z": z, "yaw_deg": yaw,
            "label": label or "", "time_enter": time_enter or "",
            "feature_count": len(node_features),
            "positive_word_refs": len(positive),
            "negative_word_refs": sum(item[0] < 0 for item in node_features),
            "distinct_words": len(distinct),
            "new_words": sum(
                first_node_by_word[word_id] == node_id for word_id in distinct),
            "reused_words": sum(
                first_node_by_word[word_id] < node_id for word_id in distinct),
            "descriptor_count": sum(item[1] > 0 for item in node_features),
            "descriptor_size_min": min(descriptor_sizes) if descriptor_sizes else "",
            "descriptor_size_max": max(descriptor_sizes) if descriptor_sizes else "",
            "empty_features": int(not node_features),
        })
    write_rows(output_dir / "nodes_words.csv", list(node_rows[0].keys()) if node_rows else [
        "node_id", "map_id", "weight", "is_keyframe", "stamp",
        "feature_count", "positive_word_refs", "distinct_words",
    ], node_rows)

    link_rows = []
    raw_link_counts = Counter()
    keyframe_edge_counts = Counter()
    canonical_keyframe_edges = set()
    for from_id, to_id, type_id, transform in links:
        type_name = LINK_TYPES.get(type_id, f"Unknown({type_id})")
        raw_link_counts[type_name] += 1
        from_keyframe = bool(
            node_weight.get(from_id) is not None and node_weight[from_id] >= 0)
        to_keyframe = bool(
            node_weight.get(to_id) is not None and node_weight[to_id] >= 0)
        active_candidate = from_keyframe and to_keyframe
        canonical = (min(from_id, to_id), max(from_id, to_id), type_id)
        first_canonical = active_candidate and canonical not in canonical_keyframe_edges
        if first_canonical:
            canonical_keyframe_edges.add(canonical)
            keyframe_edge_counts[type_name] += 1
        x, y, z, yaw = pose_fields(transform)
        link_rows.append({
            "from_id": from_id, "to_id": to_id, "type_id": type_id,
            "type": type_name, "x": x, "y": y, "z": z, "yaw_deg": yaw,
            "from_keyframe": int(from_keyframe),
            "to_keyframe": int(to_keyframe),
            "keyframe_edge_candidate": int(active_candidate),
            "first_canonical_keyframe_edge": int(first_canonical),
        })
    write_rows(output_dir / "links.csv", [
        "from_id", "to_id", "type_id", "type", "x", "y", "z", "yaw_deg",
        "from_keyframe", "to_keyframe", "keyframe_edge_candidate",
        "first_canonical_keyframe_edge",
    ], link_rows)

    loop_source = Path(args.loop_csv).expanduser() if args.loop_csv else None
    loop_rows = []
    loop_fields = [
        "stamp_ns", "stamp_s", "ref_id", "candidate_id",
        "candidate_similarity", "candidate_likelihood",
        "candidate_raw_likelihood", "posterior_best", "likelihood_best",
        "raw_likelihood_best", "visual_matches", "geometric_inliers",
        "loop_closure_id", "proximity_detection_id", "rejected_hypothesis",
        "map_id", "update_time_ms", "loop_detection_time_ms",
        "optimization_time_ms", "optimization_error",
        "optimization_max_error", "stage", "historical_candidate",
        "appearance_candidate", "acceptance_type",
    ]
    if loop_source and loop_source.is_file():
        with loop_source.open(newline="", encoding="utf-8") as handle:
            for row in csv.DictReader(handle):
                candidate = int(float(row.get("candidate_id") or 0))
                ref_id = int(float(row.get("ref_id") or 0))
                node_statistics = statistics_by_node.get(ref_id, {})
                if not row.get("visual_matches"):
                    row["visual_matches"] = node_statistics.get(
                        "visual_matches", "")
                if not row.get("geometric_inliers"):
                    row["geometric_inliers"] = node_statistics.get(
                        "visual_inliers", "")
                if not row.get("optimization_time_ms"):
                    row["optimization_time_ms"] = node_statistics.get(
                        "map_optimization_ms", "")
                if not row.get("loop_detection_time_ms"):
                    timing_values = [node_statistics.get(name)
                                     for name in (
                                         "likelihood_ms", "posterior_ms",
                                         "hypotheses_creation_ms",
                                         "hypotheses_validation_ms")]
                    timing_values = [value for value in timing_values
                                     if value is not None]
                    row["loop_detection_time_ms"] = sum(timing_values)
                row["historical_candidate"] = int(
                    candidate > 0 and ref_id > 0 and candidate != ref_id)
                if int(float(row.get("loop_closure_id") or 0)) > 0:
                    row["acceptance_type"] = "GlobalClosure"
                elif int(float(row.get("proximity_detection_id") or 0)) > 0:
                    row["acceptance_type"] = "Proximity"
                elif float(row.get("rejected_hypothesis") or 0) > 0:
                    row["acceptance_type"] = "Rejected"
                elif candidate > 0:
                    row["acceptance_type"] = "Candidate"
                else:
                    row["acceptance_type"] = "None"
                row["appearance_candidate"] = int(
                    candidate > 0 and (
                        float(row.get("candidate_likelihood") or 0) > 0.0
                        or float(row.get("posterior_best") or 0) > 0.0
                        or row["acceptance_type"] in (
                            "GlobalClosure", "Rejected")))
                loop_rows.append({key: row.get(key, "") for key in loop_fields})
    write_rows(output_dir / "loop_candidates.csv", loop_fields, loop_rows)

    if info and info[1]:
        (output_dir / "database_parameters.txt").write_text(
            info[1], encoding="utf-8")

    descriptor_sizes = Counter(size for _, size, _ in words)
    summary = {
        "database_path": database,
        "database_sha256_before": hash_before,
        "database_version": admin[0] if admin else "",
        "database_size_bytes": database.stat().st_size,
        "node_count": len(nodes),
        "keyframe_count_weight_ge_0": sum(
            weight is not None and weight >= 0 for _, _, weight, *_ in nodes),
        "intermediate_node_count_weight_lt_0": sum(
            weight is not None and weight < 0 for _, _, weight, *_ in nodes),
        "map_count": len({map_id for _, map_id, *_ in nodes}),
        "map_ids": ";".join(str(value) for value in sorted(
            {map_id for _, map_id, *_ in nodes})),
        "feature_rows": len(features),
        "feature_nodes": len(features_by_node),
        "empty_feature_nodes": sum(not features_by_node[node_id]
                                   for node_id, *_ in nodes),
        "positive_word_references": sum(word_id > 0 for _, word_id, *_ in features),
        "negative_word_references": sum(word_id < 0 for _, word_id, *_ in features),
        "distinct_positive_word_references": len({
            word_id for _, word_id, *_ in features if word_id > 0}),
        "word_table_count": len(words),
        "info_dictionary_size": info[0] if info else "",
        "word_descriptor_sizes_bytes": ";".join(
            f"{size}:{count}" for size, count in sorted(descriptor_sizes.items())),
        "feature_descriptor_rows": sum(
            (descriptor_size or 0) > 0 for _, _, descriptor_size, _ in features),
        "raw_sqlite_link_rows": len(links),
        "raw_sqlite_links_with_intermediate_endpoint": sum(
            not row["keyframe_edge_candidate"] for row in link_rows),
        "canonical_keyframe_edge_candidates": len(canonical_keyframe_edges),
        **{f"raw_sqlite_link_rows_{name}": raw_link_counts[name]
           for name in LINK_TYPES.values()},
        **{f"canonical_keyframe_edge_candidates_{name}": keyframe_edge_counts[name]
           for name in LINK_TYPES.values()},
        "statistics_rows": len(statistics),
        "loop_candidate_events": sum(
            int(row.get("appearance_candidate") or 0) > 0 for row in loop_rows),
        "global_accepted_events": sum(
            row.get("acceptance_type") == "GlobalClosure" for row in loop_rows),
        "proximity_accepted_events": sum(
            row.get("acceptance_type") == "Proximity" for row in loop_rows),
        "rejected_candidate_events": sum(
            row.get("acceptance_type") == "Rejected" for row in loop_rows),
    }
    for output_name in (
            "map_optimization_ms", "add_loop_closure_ms", "likelihood_ms",
            "posterior_ms", "hypotheses_creation_ms",
            "hypotheses_validation_ms", "proximity_by_space_ms",
            "memory_update_ms", "rtab_total_ms", "ros_total_ms"):
        values = [row.get(output_name) for row in statistics_rows]
        for statistic_name, value in numeric_summary(values).items():
            summary[f"statistics_{output_name}_{statistic_name}"] = value
    connection.close()

    try:
        info_result = subprocess.run(
            ["rtabmap-info", str(database)], text=True,
            stdout=subprocess.PIPE, stderr=subprocess.STDOUT, check=False)
        info_text = info_result.stdout
        info_status = info_result.returncode
    except FileNotFoundError:
        info_text = "rtabmap-info was not found on PATH\n"
        info_status = 127
    (output_dir / "database_info.txt").write_text(info_text, encoding="utf-8")
    summary["rtabmap_info_status"] = info_status
    plain_info = re.sub(r"\x1b\[[0-9;]*m", "", info_text)
    ltm = re.search(r"LTM:\s+(\d+) nodes and (\d+) words", plain_info)
    if ltm:
        summary["rtabmap_info_ltm_nodes"] = int(ltm.group(1))
        summary["rtabmap_info_ltm_words"] = int(ltm.group(2))
    for type_name in LINK_TYPES.values():
        match = re.search(
            rf"^\s*{re.escape(type_name)}:\s*(\d+)",
            plain_info, re.MULTILINE)
        if match:
            summary[f"rtabmap_info_active_links_{type_name}"] = int(
                match.group(1))

    with (output_dir / "pose_export.log").open("w", encoding="utf-8") as log:
        raw_status, raw_path = export_poses(
            database, output_dir, 3, "odometry_only", log)
        optimized_status, optimized_path = export_poses(
            database, output_dir, 0, "optimized_graph", log)
    summary["odometry_pose_export_status"] = raw_status
    summary["optimized_pose_export_status"] = optimized_status
    summary["odometry_pose_file"] = raw_path.name if raw_path.is_file() else ""
    summary["optimized_pose_file"] = (
        optimized_path.name if optimized_path.is_file() else "")
    accuracy_rows, corrections = write_pose_accuracy(
        output_dir, raw_path, optimized_path)
    for row in accuracy_rows:
        stream = row["stream"]
        for key, value in row.items():
            if key != "stream":
                summary[f"{stream}_{key}"] = value
    summary["maximum_pose_correction_m"] = max(
        (row["translation_correction_m"] for row in corrections), default=0.0)
    summary["maximum_pose_correction_yaw_deg"] = max(
        (row["yaw_correction_deg"] for row in corrections), default=0.0)
    hash_after = file_hash(database)
    summary["database_sha256_after"] = hash_after
    summary["database_unchanged_by_diagnostics"] = int(hash_before == hash_after)
    if hash_before != hash_after:
        raise RuntimeError("Database changed during read-only diagnostics")

    with (output_dir / "database_summary.csv").open(
            "w", newline="", encoding="utf-8") as handle:
        writer = csv.writer(handle)
        writer.writerow(["metric", "value"])
        writer.writerows(summary.items())
    print(f"Read-only database diagnostics: {output_dir}")


if __name__ == "__main__":
    main()

