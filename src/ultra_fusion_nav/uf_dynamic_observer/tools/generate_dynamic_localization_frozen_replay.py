#!/usr/bin/env python3

"""Generate the phase-locked DYN-LOC-007 MID360/IMU replay matrix.

The detector never receives the phase labels or dynamic truth written beside
the bags.  Those fields are evaluator-only.  This tool reuses the frozen
DYN-INTEGRATION-005 sensor model instead of creating a second LiDAR/IMU
contract.
"""

import argparse
import hashlib
import importlib.util
import json
import math
from pathlib import Path
import shutil


def load_frozen_generator():
    source = Path(__file__).with_name("generate_clean_gateway_frozen_replay.py")
    spec = importlib.util.spec_from_file_location("dyn_loc_frozen_base", source)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


BASE = load_frozen_generator()
BASE_TRAJECTORY = BASE.trajectory
SCAN_COUNT = 90
BEFORE_END = 25
DURING_END = 55
SCENARIOS = [
    "static_baseline",
    "person_crossing",
    "multiple_targets",
    "small_fast_target",
    "slow_target",
    "opening_closing_door",
    "large_dynamic_occlusion",
    "radial_motion",
    "moving_then_stops",
    "near_wall_motion",
    "occlusion_appear",
    "c1_persistent_occlusion",
    "c2_same_view_reobservation",
    "c3_natural_multiview_reobservation",
]
CURRENT_SCENARIO = ""


def phase(frame):
    if frame < BEFORE_END:
        return "BEFORE"
    if frame < DURING_END:
        return "DURING"
    return "AFTER"


def dynamic_localization_trajectory(seconds):
    pose, acceleration, yaw_rate = BASE_TRAJECTORY(seconds)
    if CURRENT_SCENARIO != "c2_same_view_reobservation":
        return pose, acceleration, yaw_rate
    # C2 intentionally revisits the vacated surface from effectively the same
    # view.  It remains a normal straight low-altitude traverse, not an active
    # exploration manoeuvre.
    u = max(0.0, seconds - 2.0)
    x = 0.16 * u
    return (x, 0.0, 1.2, 0.0), (0.0, 0.0, 0.0), 0.0


def add_target(points, x, y, half_width=0.25, height=1.6, spacing=0.20):
    BASE.add_box(points, x, y, half_width, height, True, spacing)


def dynamic_localization_world(name, frame):
    points = []
    BASE.add_static_environment(points)
    during = BEFORE_END <= frame < DURING_END
    progress = frame - BEFORE_END

    if name == "person_crossing" and during:
        add_target(points, 5.0, -3.2 + 0.22 * progress)
    elif name == "multiple_targets" and during:
        add_target(points, 4.5, -3.2 + 0.22 * progress)
        add_target(points, 6.0, 3.2 - 0.22 * progress)
    elif name == "small_fast_target" and during:
        add_target(points, 4.0, -4.2 + 0.39 * progress, 0.12, 0.45, 0.12)
    elif name == "slow_target" and during:
        add_target(points, 5.0, -1.0 + 0.055 * progress, 0.30, 1.0)
    elif name == "opening_closing_door":
        if frame < BEFORE_END or frame >= DURING_END:
            angle = 0.0
            moving = False
        else:
            half = (DURING_END - BEFORE_END) // 2
            local = frame - BEFORE_END
            angle = 1.2 * (local / half if local < half else (2.0 - local / half))
            moving = True
        for radius_index in range(12):
            radius = 0.10 + radius_index * 0.10
            for z_index in range(13):
                points.append(
                    (
                        6.0 + radius * math.cos(angle),
                        -1.0 + radius * math.sin(angle),
                        0.10 + z_index * 0.18,
                        moving,
                    )
                )
    elif name == "large_dynamic_occlusion" and during:
        add_target(points, 4.2, -2.7 + 0.18 * progress, 1.25, 2.5, 0.25)
    elif name == "radial_motion" and during:
        midpoint = 0.5 * (DURING_END - BEFORE_END)
        x = 8.6 - 0.22 * progress if progress < midpoint else 5.3 + 0.22 * (progress - midpoint)
        add_target(points, x, 0.6, 0.28, 1.5)
    elif name == "moving_then_stops" and during:
        y = -2.8 + 0.20 * min(progress, 14)
        add_target(points, 5.0, y, 0.28, 1.5)
    elif name == "near_wall_motion" and during:
        add_target(points, 9.35, -3.0 + 0.19 * progress, 0.24, 1.5)
    elif name == "occlusion_appear":
        BASE.add_box(points, 4.2, 0.0, 0.70, 2.5, False, 0.25)
        visible_interval = (BEFORE_END <= frame < 38) or (43 <= frame < DURING_END)
        if visible_interval:
            y = 1.0 - 0.13 * max(0, frame - BEFORE_END)
            add_target(points, 5.3, y, 0.25, 1.5)
    elif name == "c1_persistent_occlusion" and frame >= BEFORE_END:
        # The occluder intentionally remains through the final frame.
        add_target(points, 4.2, 0.0, 1.35, 2.6, 0.25)
    elif name in ("c2_same_view_reobservation", "c3_natural_multiview_reobservation") and during:
        add_target(points, 4.2, 0.0, 1.35, 2.6, 0.25)
    return points


def generate(root, name):
    global CURRENT_SCENARIO
    CURRENT_SCENARIO = name
    record = BASE.generate_scenario(root, name)
    truth_path = root / record["truth"]
    truth = json.loads(truth_path.read_text(encoding="utf-8"))
    truth.update(
        {
            "schema": "dyn_loc_007_truth_v1",
            "truth_role": "evaluator_only",
            "detector_truth_access": False,
            "phase_contract": {
                "BEFORE": [0, BEFORE_END - 1],
                "DURING": [BEFORE_END, DURING_END - 1],
                "AFTER": [DURING_END, SCAN_COUNT - 1],
            },
            "phase_by_frame": [phase(index) for index in range(SCAN_COUNT)],
            "persistent_occlusion": name == "c1_persistent_occlusion",
            "same_view_reobservation": name == "c2_same_view_reobservation",
            "natural_multiview_reobservation": name == "c3_natural_multiview_reobservation",
        }
    )
    truth_path.write_text(json.dumps(truth, sort_keys=True) + "\n", encoding="utf-8")
    record["truth_sha256"] = hashlib.sha256(truth_path.read_bytes()).hexdigest()
    return record


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("output")
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    root = Path(args.output).resolve()
    if root.exists():
        if not args.force:
            raise SystemExit(f"output already exists: {root}")
        shutil.rmtree(root)
    root.mkdir(parents=True)

    BASE.SCAN_COUNT = SCAN_COUNT
    BASE.world_scene = dynamic_localization_world
    BASE.trajectory = dynamic_localization_trajectory
    records = [generate(root, name) for name in SCENARIOS]
    manifest = {
        "schema": "dyn_loc_007_frozen_replay_v1",
        "generator": Path(__file__).name,
        "truth_role": "evaluator_only",
        "detector_truth_access": False,
        "low_altitude_near_constant_height": True,
        "scenarios": records,
    }
    manifest_path = root / "manifest.json"
    manifest_path.write_text(
        json.dumps(manifest, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    print(manifest_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
