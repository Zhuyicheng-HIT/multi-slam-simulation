#!/usr/bin/env python3
"""Sample host and validation-process resource use until interrupted."""

from __future__ import annotations

import argparse
import csv
import json
import math
import signal
import statistics
import subprocess
import time
from pathlib import Path

import psutil


CATEGORIES = {
    "gazebo": ("gz sim", "gz-server"),
    "sitl": ("arducopter",),
    "mavros": ("mavros_node",),
    "fast_lio": ("fastlio_mapping",),
    "backend": ("online_backend_fusion",),
    "vision": ("visual_frontend", "d435i_rgbd_bridge"),
    "rosbag": ("ros2 bag record", "rosbag2_transport"),
}


def percentile(values: list[float], fraction: float) -> float | None:
    if not values:
        return None
    ordered = sorted(values)
    position = fraction * (len(ordered) - 1)
    lower = math.floor(position)
    upper = math.ceil(position)
    if lower == upper:
        return ordered[lower]
    return ordered[lower] + (ordered[upper] - ordered[lower]) * (
        position - lower
    )


def summarize(values: list[float]) -> dict[str, float | int | None]:
    return {
        "count": len(values),
        "p50": statistics.median(values) if values else None,
        "p95": percentile(values, 0.95),
        "max": max(values) if values else None,
    }


def category(command: str) -> str:
    for name, patterns in CATEGORIES.items():
        if any(pattern in command for pattern in patterns):
            return name
    return "other"


def gpu_sample() -> tuple[float | None, float | None]:
    try:
        output = subprocess.check_output(
            [
                "nvidia-smi",
                "--query-gpu=utilization.gpu,memory.used",
                "--format=csv,noheader,nounits",
            ],
            stderr=subprocess.DEVNULL,
            text=True,
            timeout=2.0,
        )
        rows = [line.split(",") for line in output.splitlines() if line]
        utilization = [float(row[0].strip()) for row in rows]
        memory = [float(row[1].strip()) for row in rows]
        return sum(utilization), sum(memory)
    except (FileNotFoundError, subprocess.SubprocessError, ValueError):
        return None, None


def descendants(root_pid: int) -> list[psutil.Process]:
    try:
        root = psutil.Process(root_pid)
        return [root, *root.children(recursive=True)]
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return []


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--root-pid", type=int, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--samples-output", type=Path, required=True)
    parser.add_argument("--interval", type=float, default=1.0)
    args = parser.parse_args()
    if args.interval <= 0.0:
        parser.error("--interval must be positive")

    stop = False

    def request_stop(_signum, _frame):
        nonlocal stop
        stop = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.samples_output.parent.mkdir(parents=True, exist_ok=True)

    tracked: dict[int, psutil.Process] = {}
    series: dict[str, list[float]] = {
        "host_cpu_percent": [],
        "host_memory_used_mib": [],
        "validation_cpu_percent": [],
        "validation_rss_mib": [],
        "gpu_utilization_percent": [],
        "gpu_memory_used_mib": [],
    }
    for name in (*CATEGORIES, "other"):
        series[f"{name}_cpu_percent"] = []
        series[f"{name}_rss_mib"] = []

    fieldnames = ["wall_monotonic_s", "process_count", *series]
    psutil.cpu_percent(interval=None)
    started = time.monotonic()
    with args.samples_output.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.DictWriter(stream, fieldnames=fieldnames)
        writer.writeheader()
        while not stop:
            sample_started = time.monotonic()
            processes = descendants(args.root_pid)
            if not processes:
                break
            live_pids = {process.pid for process in processes}
            tracked = {
                pid: process for pid, process in tracked.items()
                if pid in live_pids and process.is_running()
            }
            for process in processes:
                if process.pid == psutil.Process().pid or process.pid in tracked:
                    continue
                try:
                    process.cpu_percent(interval=None)
                    tracked[process.pid] = process
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    pass

            category_cpu = {name: 0.0 for name in (*CATEGORIES, "other")}
            category_rss = {name: 0.0 for name in (*CATEGORIES, "other")}
            for process in list(tracked.values()):
                try:
                    command = " ".join(process.cmdline()).lower()
                    name = category(command)
                    category_cpu[name] += process.cpu_percent(interval=None)
                    category_rss[name] += process.memory_info().rss / (1024.0**2)
                except (psutil.NoSuchProcess, psutil.AccessDenied):
                    tracked.pop(process.pid, None)

            memory = psutil.virtual_memory()
            gpu_utilization, gpu_memory = gpu_sample()
            row = {
                "wall_monotonic_s": sample_started,
                "process_count": len(tracked),
                "host_cpu_percent": psutil.cpu_percent(interval=None),
                "host_memory_used_mib": memory.used / (1024.0**2),
                "validation_cpu_percent": sum(category_cpu.values()),
                "validation_rss_mib": sum(category_rss.values()),
                "gpu_utilization_percent": gpu_utilization,
                "gpu_memory_used_mib": gpu_memory,
            }
            for name in (*CATEGORIES, "other"):
                row[f"{name}_cpu_percent"] = category_cpu[name]
                row[f"{name}_rss_mib"] = category_rss[name]
            writer.writerow(row)
            stream.flush()
            for key in series:
                value = row[key]
                if value is not None:
                    series[key].append(float(value))
            remaining = args.interval - (time.monotonic() - sample_started)
            if remaining > 0.0:
                time.sleep(remaining)

    report = {
        "schema_version": 1,
        "root_pid": args.root_pid,
        "wall_duration_s": time.monotonic() - started,
        "sample_interval_s": args.interval,
        "samples": len(series["host_cpu_percent"]),
        "statistics": {key: summarize(values) for key, values in series.items()},
        "samples_output": str(args.samples_output),
    }
    args.output.write_text(
        json.dumps(report, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
