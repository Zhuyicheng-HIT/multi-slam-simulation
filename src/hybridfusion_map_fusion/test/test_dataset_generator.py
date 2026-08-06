import csv
from pathlib import Path
import subprocess
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]


def pcd_points(path):
    for line in path.read_text(encoding="ascii").splitlines():
        if line.startswith("POINTS "):
            return int(line.split()[1])
    raise AssertionError(f"POINTS header missing from {path}")


def test_unit_dataset_is_complete_and_deterministic(tmp_path):
    script = ROOT / "scripts" / "generate_hybridfusion_dataset.py"
    command = [
        sys.executable, str(script), "--output", str(tmp_path),
        "--preset", "unit", "--seed", "20260805",
    ]
    subprocess.run(command, check=True, capture_output=True, text=True)
    first_visual = (tmp_path / "visual_map.pcd").read_bytes()
    first_lidar = (tmp_path / "lidar_map.pcd").read_bytes()
    subprocess.run(command, check=True, capture_output=True, text=True)
    assert (tmp_path / "visual_map.pcd").read_bytes() == first_visual
    assert (tmp_path / "lidar_map.pcd").read_bytes() == first_lidar

    manifest = yaml.safe_load((tmp_path / "dataset.yaml").read_text(encoding="utf-8"))
    dataset = manifest["dataset"]
    assert dataset["visual_frame"] == "rtabmap_map"
    assert dataset["lidar_frame"] == "camera_init"
    assert dataset["generated_not_measured"] is True
    assert len(dataset["truth_lidar_to_visual"]) == 6
    assert pcd_points(tmp_path / "visual_map.pcd") > 1000
    assert pcd_points(tmp_path / "lidar_map.pcd") > 1000

    with (tmp_path / "ground_truth_route.csv").open(newline="", encoding="utf-8") as stream:
        rows = list(csv.DictReader(stream))
    assert len(rows) == dataset["keyframes"] == 10
    assert len(list((tmp_path / "visual_keyframes").glob("*.pcd"))) == 10


def test_tracked_fixture_contract_matches_generator_defaults():
    fixture = yaml.safe_load(
        (ROOT / "test" / "data" / "deterministic_scene.yaml").read_text(encoding="utf-8"))
    assert fixture["seed"] == 20260805
    assert fixture["scene_extent_m"] == [29.0, 24.0, 6.5]
    assert fixture["required_structures"] == [
        "ground", "main_building", "annex", "facades", "roofs", "curb", "columns"]
