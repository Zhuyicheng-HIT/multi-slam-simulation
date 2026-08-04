#!/usr/bin/env python3
"""Read-only attitude-jitter analysis for ArduPilot DataFlash BIN logs."""

import argparse
from collections import Counter, defaultdict
import json
import math
from pathlib import Path
import sys

import numpy as np
from pymavlink import mavutil


HIGH_FREQUENCY_HZ = 3.0
DOMINANT_FREQUENCY_MIN_HZ = 0.5
MAX_UNIFORM_SAMPLES = 2_000_000


def finite_float(value):
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    return result if math.isfinite(result) else None


def message_time_s(message):
    value = finite_float(getattr(message, "TimeUS", None))
    return value * 1.0e-6 if value is not None else None


def message_text(value):
    if isinstance(value, bytes):
        value = value.decode("ascii", errors="replace")
    return str(value).rstrip("\x00").strip()


def basic_metrics(values):
    values = np.asarray(values, dtype=np.float64)
    values = values[np.isfinite(values)]
    if values.size == 0:
        return {
            "samples": 0,
            "mean": None,
            "std": None,
            "rms": None,
            "p95_abs": None,
            "min": None,
            "max": None,
        }
    return {
        "samples": int(values.size),
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "rms": float(np.sqrt(np.mean(np.square(values)))),
        "p95_abs": float(np.percentile(np.abs(values), 95.0)),
        "min": float(np.min(values)),
        "max": float(np.max(values)),
    }


def prepare_signal(records, start_s, end_s):
    if not records:
        return np.empty(0), np.empty(0)
    array = np.asarray(records, dtype=np.float64)
    valid = (
        np.isfinite(array[:, 0])
        & np.isfinite(array[:, 1])
        & (array[:, 0] >= start_s)
        & (array[:, 0] <= end_s)
    )
    array = array[valid]
    if array.size == 0:
        return np.empty(0), np.empty(0)
    order = np.argsort(array[:, 0], kind="stable")
    array = array[order]
    unique_times, unique_indices = np.unique(array[:, 0], return_index=True)
    return unique_times, array[unique_indices, 1]


def uniform_signal(times, values):
    if len(times) < 4 or times[-1] <= times[0]:
        return None, None, None
    duration_s = float(times[-1] - times[0])
    sample_rate_hz = float((len(times) - 1) / duration_s)
    if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
        return None, None, None
    sample_count = int(round(duration_s * sample_rate_hz)) + 1
    if sample_count > MAX_UNIFORM_SAMPLES:
        sample_count = MAX_UNIFORM_SAMPLES
        sample_rate_hz = float((sample_count - 1) / duration_s)
    grid = np.linspace(times[0], times[-1], sample_count, dtype=np.float64)
    return grid, np.interp(grid, times, values), sample_rate_hz


def spectral_metrics(times, values):
    grid, uniform_values, sample_rate_hz = uniform_signal(times, values)
    empty = {
        "uniform_sample_rate_hz": None,
        "dominant_frequency_hz": None,
        "dominant_search_min_hz": DOMINANT_FREQUENCY_MIN_HZ,
        "high_frequency_cutoff_hz": HIGH_FREQUENCY_HZ,
        "rms_above_3hz": None,
    }
    if grid is None or len(grid) < 8:
        return empty

    normalized_time = np.linspace(-1.0, 1.0, len(grid), dtype=np.float64)
    slope, intercept = np.polyfit(normalized_time, uniform_values, 1)
    detrended = uniform_values - (slope * normalized_time + intercept)
    if not np.all(np.isfinite(detrended)):
        return empty

    frequencies = np.fft.rfftfreq(len(detrended), d=1.0 / sample_rate_hz)
    spectrum = np.fft.rfft(detrended)
    high_pass_spectrum = spectrum.copy()
    high_pass_spectrum[frequencies <= HIGH_FREQUENCY_HZ] = 0.0
    high_frequency_signal = np.fft.irfft(high_pass_spectrum, n=len(detrended))

    dominant_frequency_hz = None
    if np.std(detrended) > 1.0e-12:
        windowed_amplitude = np.abs(np.fft.rfft(detrended * np.hanning(len(detrended))))
        search = frequencies >= DOMINANT_FREQUENCY_MIN_HZ
        if np.any(search):
            candidate_indices = np.flatnonzero(search)
            dominant_index = candidate_indices[int(np.argmax(windowed_amplitude[search]))]
            dominant_frequency_hz = float(frequencies[dominant_index])

    return {
        "uniform_sample_rate_hz": float(sample_rate_hz),
        "dominant_frequency_hz": dominant_frequency_hz,
        "dominant_search_min_hz": DOMINANT_FREQUENCY_MIN_HZ,
        "high_frequency_cutoff_hz": HIGH_FREQUENCY_HZ,
        "rms_above_3hz": float(
            np.sqrt(np.mean(np.square(high_frequency_signal)))
        ),
    }


def signal_metrics(records, start_s, end_s):
    times, values = prepare_signal(records, start_s, end_s)
    result = basic_metrics(values)
    if len(times) >= 2:
        periods = np.diff(times)
        result.update({
            "duration_s": float(times[-1] - times[0]),
            "effective_sample_rate_hz": float((len(times) - 1) / (times[-1] - times[0])),
            "median_sample_period_ms": float(np.median(periods) * 1.0e3),
            "max_sample_gap_ms": float(np.max(periods) * 1.0e3),
        })
    else:
        result.update({
            "duration_s": 0.0,
            "effective_sample_rate_hz": None,
            "median_sample_period_ms": None,
            "max_sample_gap_ms": None,
        })
    result.update(spectral_metrics(times, values))
    return result


def find_airborne_segment(altitude_candidates, threshold_m, fallback_bounds):
    priority = (
        ("SIM2.PD", "ground_truth", altitude_candidates["SIM2.PD"]),
        ("POS.RelHomeAlt", "estimator", altitude_candidates["POS.RelHomeAlt"]),
        ("CTUN.Alt", "controller", altitude_candidates["CTUN.Alt"]),
        ("XKF1.PD", "estimator", altitude_candidates["XKF1.PD"]),
    )
    warnings = []
    for source, source_class, records in priority:
        times, altitude = prepare_signal(records, -math.inf, math.inf)
        if len(times) < 4:
            continue
        baseline_end_s = min(times[0] + 5.0, times[-1])
        baseline_values = altitude[times <= baseline_end_s]
        baseline_m = float(np.median(baseline_values))
        relative_altitude = altitude - baseline_m
        airborne_indices = np.flatnonzero(relative_altitude > threshold_m)
        if len(airborne_indices) < 2:
            continue

        groups = []
        group_start = 0
        # Join short threshold crossings so vibration near the boundary does not split a flight.
        for index in range(1, len(airborne_indices)):
            previous = airborne_indices[index - 1]
            current = airborne_indices[index]
            if times[current] - times[previous] > 1.0:
                groups.append(airborne_indices[group_start:index])
                group_start = index
        groups.append(airborne_indices[group_start:])
        group = max(groups, key=lambda item: times[item[-1]] - times[item[0]])
        start_s = float(times[group[0]])
        end_s = float(times[group[-1]])
        if end_s - start_s < 1.0:
            continue
        if source_class != "ground_truth":
            warnings.append(
                f"Airborne interval uses {source}, which is not independent ground truth."
            )
        return {
            "airborne_detected": True,
            "source": source,
            "source_class": source_class,
            "start_s": start_s,
            "end_s": end_s,
            "duration_s": end_s - start_s,
            "start_time_us": int(round(start_s * 1.0e6)),
            "end_time_us": int(round(end_s * 1.0e6)),
            "altitude_baseline_m": baseline_m,
            "airborne_height_threshold_m": threshold_m,
            "maximum_relative_altitude_m": float(np.max(relative_altitude)),
            "warnings": warnings,
        }

    start_s, end_s = fallback_bounds
    warnings.append(
        "No airborne interval exceeded the altitude threshold; metrics use the full available log."
    )
    return {
        "airborne_detected": False,
        "source": "full_log_fallback",
        "source_class": "unknown",
        "start_s": float(start_s),
        "end_s": float(end_s),
        "duration_s": float(max(0.0, end_s - start_s)),
        "start_time_us": int(round(start_s * 1.0e6)),
        "end_time_us": int(round(end_s * 1.0e6)),
        "altitude_baseline_m": None,
        "airborne_height_threshold_m": threshold_m,
        "maximum_relative_altitude_m": None,
        "warnings": warnings,
    }


def rate_axis_metrics(records, desired_index, actual_index, start_s, end_s):
    if not records:
        return {
            "desired": signal_metrics([], start_s, end_s),
            "actual": signal_metrics([], start_s, end_s),
            "tracking_error_actual_minus_desired": signal_metrics([], start_s, end_s),
            "tracking_correlation": None,
            "actual_to_desired_std_ratio": None,
        }
    array = np.asarray(records, dtype=np.float64)
    valid = (
        np.all(np.isfinite(array[:, [0, desired_index, actual_index]]), axis=1)
        & (array[:, 0] >= start_s)
        & (array[:, 0] <= end_s)
    )
    selected = array[valid]
    desired_records = [(row[0], row[desired_index]) for row in selected]
    actual_records = [(row[0], row[actual_index]) for row in selected]
    error_records = [
        (row[0], row[actual_index] - row[desired_index]) for row in selected
    ]
    desired = signal_metrics(desired_records, start_s, end_s)
    actual = signal_metrics(actual_records, start_s, end_s)
    error = signal_metrics(error_records, start_s, end_s)

    correlation = None
    std_ratio = None
    if len(selected) >= 3:
        desired_values = selected[:, desired_index]
        actual_values = selected[:, actual_index]
        desired_std = float(np.std(desired_values))
        actual_std = float(np.std(actual_values))
        if desired_std > 1.0e-12 and actual_std > 1.0e-12:
            correlation = float(np.corrcoef(desired_values, actual_values)[0, 1])
            std_ratio = actual_std / desired_std
    return {
        "desired": desired,
        "actual": actual,
        "tracking_error_actual_minus_desired": error,
        "tracking_correlation": correlation,
        "actual_to_desired_std_ratio": std_ratio,
    }


def read_log(path):
    message_counts = Counter()
    imu = defaultdict(list)
    rate_records = []
    sim2_velocity = defaultdict(list)
    vibe = defaultdict(list)
    clip_records = []
    altitude_candidates = defaultdict(list)
    parameters = {}
    all_times = []

    connection = mavutil.mavlink_connection(str(path), robust_parsing=True)
    try:
        while True:
            message = connection.recv_match(blocking=False)
            if message is None:
                break
            message_type = message.get_type()
            message_counts[message_type] += 1
            time_s = message_time_s(message)
            if time_s is not None:
                all_times.append(time_s)

            if message_type == "IMU" and int(getattr(message, "I", -1)) == 0:
                for field in ("GyrX", "GyrY"):
                    value = finite_float(getattr(message, field, None))
                    if time_s is not None and value is not None:
                        imu[field].append((time_s, value))
            elif message_type == "RATE":
                values = [
                    finite_float(getattr(message, field, None))
                    for field in ("RDes", "R", "PDes", "P")
                ]
                if time_s is not None and all(value is not None for value in values):
                    rate_records.append((time_s, *values))
            elif message_type == "SIM2":
                for field in ("VN", "VE", "VD"):
                    value = finite_float(getattr(message, field, None))
                    if time_s is not None and value is not None:
                        sim2_velocity[field].append((time_s, value))
                down = finite_float(getattr(message, "PD", None))
                if time_s is not None and down is not None:
                    altitude_candidates["SIM2.PD"].append((time_s, -down))
            elif message_type == "VIBE" and int(getattr(message, "IMU", -1)) == 0:
                for field in ("VibeX", "VibeY", "VibeZ"):
                    value = finite_float(getattr(message, field, None))
                    if time_s is not None and value is not None:
                        vibe[field].append((time_s, value))
                clip = finite_float(getattr(message, "Clip", None))
                if time_s is not None and clip is not None:
                    clip_records.append((time_s, clip))
            elif message_type == "POS":
                value = finite_float(getattr(message, "RelHomeAlt", None))
                if time_s is not None and value is not None:
                    altitude_candidates["POS.RelHomeAlt"].append((time_s, value))
            elif message_type == "CTUN":
                value = finite_float(getattr(message, "Alt", None))
                if time_s is not None and value is not None:
                    altitude_candidates["CTUN.Alt"].append((time_s, value))
            elif message_type == "XKF1" and int(getattr(message, "C", -1)) == 0:
                value = finite_float(getattr(message, "PD", None))
                if time_s is not None and value is not None:
                    altitude_candidates["XKF1.PD"].append((time_s, -value))
            elif message_type == "PARM":
                name = message_text(getattr(message, "Name", ""))
                if name.startswith(("ATC_RAT_RLL_", "ATC_RAT_PIT_")):
                    value = finite_float(getattr(message, "Value", None))
                    default = finite_float(getattr(message, "Default", None))
                    parameters[name] = {"value": value, "default": default}
    finally:
        connection.close()

    if not all_times:
        raise RuntimeError("No timestamped DataFlash messages were decoded")
    return {
        "message_counts": message_counts,
        "imu": imu,
        "rate": rate_records,
        "sim2_velocity": sim2_velocity,
        "vibe": vibe,
        "clip": clip_records,
        "altitude_candidates": altitude_candidates,
        "parameters": parameters,
        "bounds": (min(all_times), max(all_times)),
    }


def clip_metrics(records, start_s, end_s):
    times, values = prepare_signal(records, start_s, end_s)
    if len(values) == 0:
        return {
            "samples": 0,
            "first_counter": None,
            "last_counter": None,
            "maximum_counter": None,
            "counter_increase_sum": None,
            "samples_with_nonzero_counter": 0,
        }
    positive_increments = np.maximum(np.diff(values), 0.0)
    return {
        "samples": int(len(values)),
        "first_counter": int(round(values[0])),
        "last_counter": int(round(values[-1])),
        "maximum_counter": int(round(np.max(values))),
        "counter_increase_sum": int(round(np.sum(positive_increments))),
        "samples_with_nonzero_counter": int(np.count_nonzero(values)),
    }


def vector_high_frequency_rms(axis_reports):
    components = [report.get("rms_above_3hz") for report in axis_reports]
    if any(value is None for value in components):
        return None
    return float(math.sqrt(sum(value * value for value in components)))


def analyze(path, airborne_height_m):
    data = read_log(path)
    segment = find_airborne_segment(
        data["altitude_candidates"], airborne_height_m, data["bounds"]
    )
    start_s = segment["start_s"]
    end_s = segment["end_s"]

    imu_roll = signal_metrics(data["imu"]["GyrX"], start_s, end_s)
    imu_pitch = signal_metrics(data["imu"]["GyrY"], start_s, end_s)
    rate_roll = rate_axis_metrics(data["rate"], 1, 2, start_s, end_s)
    rate_pitch = rate_axis_metrics(data["rate"], 3, 4, start_s, end_s)

    sim2_reports = {
        axis.lower(): signal_metrics(data["sim2_velocity"][axis], start_s, end_s)
        for axis in ("VN", "VE", "VD")
    }
    horizontal_records = []
    vn_times, vn = prepare_signal(data["sim2_velocity"]["VN"], start_s, end_s)
    ve_times, ve = prepare_signal(data["sim2_velocity"]["VE"], start_s, end_s)
    if len(vn_times) and np.array_equal(vn_times, ve_times):
        horizontal_records = list(zip(vn_times, np.hypot(vn, ve)))
    sim2_reports["horizontal_speed"] = signal_metrics(
        horizontal_records, start_s, end_s
    )
    sim2_reports["horizontal_vector_rms_above_3hz"] = vector_high_frequency_rms(
        [sim2_reports["vn"], sim2_reports["ve"]]
    )
    sim2_reports["three_axis_vector_rms_above_3hz"] = vector_high_frequency_rms(
        [sim2_reports["vn"], sim2_reports["ve"], sim2_reports["vd"]]
    )

    vibe_reports = {
        axis.lower(): signal_metrics(data["vibe"][axis], start_s, end_s)
        for axis in ("VibeX", "VibeY", "VibeZ")
    }
    vibe_reports["clip_counter"] = clip_metrics(data["clip"], start_s, end_s)

    selected_counts = {
        "IMU0": len(data["imu"]["GyrX"]),
        "RATE": len(data["rate"]),
        "SIM2": len(data["sim2_velocity"]["VN"]),
        "VIBE_IMU0": len(data["vibe"]["VibeX"]),
        "PARM_ATC_RAT_RLL_PIT": len(data["parameters"]),
    }
    return {
        "schema_version": 1,
        "input": {
            "path": str(path.resolve()),
            "size_bytes": path.stat().st_size,
            "read_only": True,
        },
        "flight_segment": segment,
        "message_counts": {
            key: int(value) for key, value in sorted(data["message_counts"].items())
        },
        "selected_message_counts": selected_counts,
        "metrics": {
            "imu0_gyro": {
                "unit": "rad/s",
                "axis_mapping": {"roll": "IMU.GyrX", "pitch": "IMU.GyrY"},
                "roll": imu_roll,
                "pitch": imu_pitch,
            },
            "rate_tracking": {
                "unit": "deg/s",
                "roll": rate_roll,
                "pitch": rate_pitch,
            },
            "sim2_true_velocity": {
                "unit": "m/s",
                **sim2_reports,
            },
            "vibe_imu0": {
                "vibration_unit": "m/s^2",
                **vibe_reports,
            },
        },
        "parameters": {
            "source": "PARM messages in DataFlash log",
            "atc_rate_roll_pitch": dict(sorted(data["parameters"].items())),
        },
        "method": {
            "frequency_analysis": (
                "Irregular samples are linearly resampled at their effective rate, "
                "linearly detrended, and analyzed with NumPy FFT."
            ),
            "dominant_frequency_search_min_hz": DOMINANT_FREQUENCY_MIN_HZ,
            "high_frequency_cutoff_hz": HIGH_FREQUENCY_HZ,
            "high_frequency_filter": "ideal FFT high-pass after linear detrending",
        },
    }


def parse_args(argv):
    parser = argparse.ArgumentParser(
        description=(
            "Analyze roll/pitch attitude jitter in an ArduPilot DataFlash BIN without "
            "modifying the input log."
        )
    )
    parser.add_argument("bin", type=Path, help="ArduPilot DataFlash .BIN log")
    parser.add_argument("--output", required=True, type=Path, help="Output JSON report")
    parser.add_argument(
        "--airborne-height-m",
        type=float,
        default=1.0,
        help="Relative-height threshold used to select the flight interval (default: 1.0)",
    )
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv if argv is not None else sys.argv[1:])
    if not args.bin.is_file():
        print(f"Input BIN does not exist: {args.bin}", file=sys.stderr)
        return 2
    if args.airborne_height_m <= 0.0:
        print("--airborne-height-m must be positive", file=sys.stderr)
        return 2
    try:
        report = analyze(args.bin, args.airborne_height_m)
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", encoding="utf-8") as stream:
            json.dump(report, stream, ensure_ascii=True, indent=2, allow_nan=False)
            stream.write("\n")
    except Exception as error:
        print(f"Analysis failed: {error}", file=sys.stderr)
        return 1
    print(f"Wrote attitude-jitter report: {args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
