#!/usr/bin/env python3
"""Run fast, dependency-light checks for pull requests.

The full ROS/Gazebo stack is intentionally left to a runner with those
system dependencies.  This smoke test catches malformed source and assets,
encoding regressions, shell syntax errors, and a small protocol/path suite.
"""

from __future__ import annotations

import subprocess
import sys
import xml.etree.ElementTree as ET
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
TEXT_SUFFIXES = {
    ".c", ".cc", ".cpp", ".h", ".hpp", ".ini", ".json", ".md", ".py",
    ".rst", ".sdf", ".sh", ".txt", ".xml", ".yaml", ".yml", ".csv",
}
MOJIBAKE_MARKERS = ("\ufffd", "Ã", "Â", "â", "ð", "ï¿½")


def tracked_files() -> list[Path]:
    result = subprocess.run(
        ["git", "ls-files", "-z"], cwd=ROOT, check=True, capture_output=True
    )
    return [ROOT / item for item in result.stdout.decode().split("\0") if item]


def check_text(files: list[Path]) -> None:
    failures: list[str] = []
    for path in files:
        if path.suffix.lower() not in TEXT_SUFFIXES:
            continue
        # This checker contains the marker strings it is looking for.
        if path.resolve() == Path(__file__).resolve():
            continue
        try:
            text = path.read_text(encoding="utf-8")
        except UnicodeDecodeError as exc:
            failures.append(f"{path.relative_to(ROOT)}: invalid UTF-8 ({exc})")
            continue
        if any(marker in text for marker in MOJIBAKE_MARKERS):
            failures.append(f"{path.relative_to(ROOT)}: suspicious replacement/mojibake text")
        if "<<<<<<<" in text or ">>>>>>>" in text or "=======" in text:
            failures.append(f"{path.relative_to(ROOT)}: merge-conflict marker")
    if failures:
        raise SystemExit("\n".join(failures))


def check_python(files: list[Path]) -> None:
    for path in files:
        if path.suffix == ".py":
            compile(path.read_text(encoding="utf-8"), str(path), "exec")


def check_structured(files: list[Path]) -> None:
    try:
        import yaml
    except ImportError as exc:  # pragma: no cover - CI installs PyYAML
        raise SystemExit("PyYAML is required for the smoke test") from exc
    for path in files:
        suffix = path.suffix.lower()
        if suffix in {".xml", ".sdf", ".world"}:
            ET.parse(path)
        elif suffix in {".yaml", ".yml"}:
            yaml.safe_load(path.read_text(encoding="utf-8"))


def check_shell(files: list[Path]) -> None:
    for path in files:
        if path.suffix == ".sh":
            source = path.read_bytes().replace(b"\r\n", b"\n")
            subprocess.run(["bash", "-n"], cwd=ROOT, input=source, check=True)


def run_protocol_smoke() -> None:
    tests = [
        "src/multi_slam_uav_sim/test/test_mid360_protocol.py",
        "src/multi_slam_uav_sim/test/test_micolink_protocol.py",
        "src/multi_slam_uav_sim/test/test_mtf01p_protocol.py",
        "src/multi_slam_uav_sim/test/test_s_curve_path.py",
    ]
    env = {**__import__("os").environ, "PYTHONPATH": str(ROOT / "src/multi_slam_uav_sim")}
    subprocess.run([sys.executable, "-m", "pytest", "-q", *tests], cwd=ROOT, env=env, check=True)


def main() -> int:
    files = tracked_files()
    check_text(files)
    check_python(files)
    check_structured(files)
    check_shell(files)
    run_protocol_smoke()
    print(f"CI smoke checks passed for {len(files)} tracked files")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
