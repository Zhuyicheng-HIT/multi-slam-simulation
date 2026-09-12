#!/usr/bin/env python3

import argparse
import json
import os
from pathlib import Path
import shutil
import signal
import statistics
import subprocess
import time

import psutil


def stop_process(process, timeout=8.0):
    if process is None or process.poll() is not None:
        return
    try:
        os.killpg(process.pid, signal.SIGINT)
        process.wait(timeout=timeout)
    except (ProcessLookupError, subprocess.TimeoutExpired):
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGTERM)
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3.0)


def process_resources(process):
    if process is None or process.poll() is not None:
        return 0.0, 0
    try:
        root = psutil.Process(process.pid)
        family = [root] + root.children(recursive=True)
        cpu_seconds = 0.0
        rss = 0
        for member in family:
            try:
                times = member.cpu_times()
                cpu_seconds += times.user + times.system
                rss += member.memory_info().rss
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        return cpu_seconds, rss
    except (psutil.NoSuchProcess, psutil.AccessDenied):
        return 0.0, 0


def launch(command, log_path):
    stream = log_path.open("w", encoding="utf-8")
    process = subprocess.Popen(
        command,
        stdout=stream,
        stderr=subprocess.STDOUT,
        start_new_session=True,
        text=True,
    )
    process._dyn_log_stream = stream
    return process


def close_log(process):
    stream = getattr(process, "_dyn_log_stream", None)
    if stream is not None:
        stream.close()


def run_branch(args, manifest_root, output_root, scenario, branch):
    namespace = f"/dyn_ab/{scenario}/{branch}"
    run_dir = output_root / scenario / branch
    run_dir.mkdir(parents=True)
    bag_output = run_dir / "output"
    config = Path(args.config).resolve()
    gateway_config = Path(args.gateway_config).resolve()
    input_bag = manifest_root / scenario / "input"
    raw_topic = f"{namespace}/raw"
    imu_topic = f"{namespace}/imu"
    lidar_topic = raw_topic if branch == "raw" else f"{namespace}/clean"
    odom_topic = f"{namespace}/odom"
    factor_topic = f"{namespace}/native_lidar_factor"
    map_topic = f"{namespace}/map"
    state_topic = f"{namespace}/previous_state"
    status_topic = f"{namespace}/gateway_status"
    diagnostics_topic = f"{namespace}/gateway_diagnostics"
    latency_output = run_dir / "fast_lio_latency.json"

    fastlio_command = [
        "ros2",
        "run",
        "fast_lio",
        "fastlio_mapping",
        "--ros-args",
        "--params-file",
        str(config),
        "-p",
        f"common.lid_topic:={lidar_topic}",
        "-p",
        f"common.imu_topic:={imu_topic}",
        "-p",
        f"native_factor_export.topic:={factor_topic}",
        "-p",
        f"previous_state_export.enable:={'true' if branch == 'clean' else 'false'}",
        "-p",
        f"previous_state_export.topic:={state_topic}",
        "-r",
        f"/Odometry:={odom_topic}",
        "-r",
        f"/Laser_map:={map_topic}",
        "-r",
        f"/cloud_registered:={namespace}/cloud",
        "-r",
        f"/cloud_registered_body:={namespace}/cloud_body",
        "-r",
        f"/cloud_effected:={namespace}/effect",
        "-r",
        f"/path:={namespace}/path",
    ]
    gateway_command = None
    if branch == "clean":
        gateway_command = [
            "ros2",
            "run",
            "uf_dynamic_observer",
            "clean_scan_gateway_node",
            "--ros-args",
            "--params-file",
            str(gateway_config),
            "-p",
            "enabled:=true",
            "-p",
            f"raw_topic:={raw_topic}",
            "-p",
            f"clean_topic:={lidar_topic}",
            "-p",
            f"imu_topic:={imu_topic}",
            "-p",
            f"previous_state_topic:={state_topic}",
            "-r",
            f"/dynamic_observer/clean/status:={status_topic}",
            "-r",
            f"/dynamic_observer/clean/diagnostics:={diagnostics_topic}",
        ]

    latency_probe_command = [
        "ros2",
        "run",
        "uf_dynamic_observer",
        "fast_lio_latency_probe.py",
        "--lidar-topic",
        lidar_topic,
        "--odom-topic",
        odom_topic,
        "--output",
        str(latency_output),
    ]

    # Re-record the raw input only to establish callback wall-time latency. The
    # player delay below gives DDS discovery time before the first frozen event.
    record_topics = [raw_topic, imu_topic, odom_topic, factor_topic, map_topic]
    if branch == "clean":
        record_topics.extend([lidar_topic, state_topic, status_topic, diagnostics_topic])
    record_command = ["ros2", "bag", "record", "-o", str(bag_output)] + record_topics
    play_command = [
        "ros2",
        "bag",
        "play",
        str(input_bag),
        "--rate",
        str(args.rate),
        "--delay",
        str(args.player_delay),
        "--remap",
        f"/frozen/livox/lidar:={raw_topic}",
        f"/frozen/livox/imu:={imu_topic}",
    ]

    fastlio = gateway = latency_probe = recorder = player = None
    cpu_samples = {"fast_lio": [], "gateway": []}
    rss_samples = {"fast_lio": [], "gateway": []}
    sample_times = []
    return_codes = {}
    try:
        fastlio = launch(fastlio_command, run_dir / "fast_lio.log")
        if gateway_command is not None:
            gateway = launch(gateway_command, run_dir / "gateway.log")
        latency_probe = launch(latency_probe_command, run_dir / "latency_probe.log")
        time.sleep(args.startup_wait)
        if (
            fastlio.poll() is not None
            or latency_probe.poll() is not None
            or (gateway is not None and gateway.poll() is not None)
        ):
            raise RuntimeError("estimator or gateway exited during startup")
        recorder = launch(record_command, run_dir / "record.log")
        time.sleep(args.discovery_wait)
        player = launch(play_command, run_dir / "play.log")
        time.sleep(args.player_delay + 0.1)
        while player.poll() is None:
            fast_cpu, fast_rss = process_resources(fastlio)
            gateway_cpu, gateway_rss = process_resources(gateway)
            cpu_samples["fast_lio"].append(fast_cpu)
            rss_samples["fast_lio"].append(fast_rss)
            if gateway is not None:
                cpu_samples["gateway"].append(gateway_cpu)
                rss_samples["gateway"].append(gateway_rss)
            sample_times.append(time.monotonic())
            time.sleep(0.10)
        return_codes["player"] = player.returncode
        time.sleep(args.drain_wait)
    finally:
        stop_process(recorder)
        stop_process(latency_probe)
        stop_process(gateway)
        stop_process(fastlio)
        for name, process in [
            ("recorder", recorder),
            ("latency_probe", latency_probe),
            ("gateway", gateway),
            ("fast_lio", fastlio),
        ]:
            if process is not None:
                return_codes[name] = process.returncode
                close_log(process)
        if player is not None:
            close_log(player)

    def summarize(values):
        return {
            "mean": statistics.fmean(values) if values else 0.0,
            "max": max(values) if values else 0.0,
            "samples": len(values),
        }

    def summarize_cpu(values):
        intervals = []
        for index in range(1, min(len(values), len(sample_times))):
            elapsed = sample_times[index] - sample_times[index - 1]
            if elapsed > 0.0:
                intervals.append(100.0 * (values[index] - values[index - 1]) / elapsed)
        return summarize(intervals)

    runtime = {
        "scenario": scenario,
        "branch": branch,
        "input_bag": str(input_bag),
        "output_bag": str(bag_output),
        "playback_rate": args.rate,
        "cpu_percent": {name: summarize_cpu(values) for name, values in cpu_samples.items()},
        "rss_mib": {
            name: summarize([value / (1024.0 * 1024.0) for value in values])
            for name, values in rss_samples.items()
        },
        "return_codes": return_codes,
        "fast_lio_callback_latency_ms": (
            json.loads(latency_output.read_text(encoding="utf-8"))
            if latency_output.exists()
            else None
        ),
    }
    (run_dir / "runtime.json").write_text(
        json.dumps(runtime, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    if return_codes.get("player") != 0:
        raise RuntimeError(f"player failed for {scenario}/{branch}: {return_codes}")
    return runtime


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("manifest")
    parser.add_argument("output")
    parser.add_argument("--config", required=True)
    parser.add_argument("--gateway-config", required=True)
    parser.add_argument("--scenarios", nargs="*")
    parser.add_argument("--branches", nargs="*", choices=["raw", "clean"])
    parser.add_argument("--rate", type=float, default=1.0)
    parser.add_argument("--player-delay", type=float, default=4.0)
    parser.add_argument("--startup-wait", type=float, default=1.0)
    parser.add_argument("--discovery-wait", type=float, default=0.8)
    parser.add_argument("--drain-wait", type=float, default=0.8)
    parser.add_argument("--force", action="store_true")
    args = parser.parse_args()
    manifest_path = Path(args.manifest).resolve()
    manifest_root = manifest_path.parent
    manifest = json.loads(manifest_path.read_text(encoding="utf-8"))
    available = [entry["scenario"] for entry in manifest["scenarios"]]
    scenarios = args.scenarios or available
    branches = args.branches or ["raw", "clean"]
    output_root = Path(args.output).resolve()
    if output_root.exists():
        if not args.force:
            raise SystemExit(f"output already exists: {output_root}")
        shutil.rmtree(output_root)
    output_root.mkdir(parents=True)
    records = []
    for scenario in scenarios:
        if scenario not in available:
            raise SystemExit(f"scenario is not in manifest: {scenario}")
        for branch in branches:
            print(f"RUN {scenario} {branch}", flush=True)
            records.append(
                run_branch(args, manifest_root, output_root, scenario, branch)
            )
    summary = {"manifest": str(manifest_path), "runs": records}
    (output_root / "run_summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n", encoding="utf-8"
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
