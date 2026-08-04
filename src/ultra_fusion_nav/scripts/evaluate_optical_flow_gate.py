#!/usr/bin/env python3
import argparse
import json
import re
from pathlib import Path


def evaluate_gate(calibration, lio_report, gazebo_log):
    match = re.findall(r"FLOW_ACCURACY .*?passed=(True|False)", gazebo_log)
    gazebo_sensor_passed = bool(match and match[-1] == "True")
    lio_crosscheck_passed = bool(calibration.get("passed", False))
    trajectory = lio_report.get("trajectory", {})
    lio_reference_valid = bool(trajectory.get("coupling_reference_valid", False))

    if not gazebo_sensor_passed:
        classification = "sensor_failed"
        passed = False
    elif lio_crosscheck_passed:
        classification = "sensor_and_lio_crosscheck_passed"
        passed = True
    elif lio_reference_valid:
        classification = "lio_crosscheck_failed_with_valid_reference"
        passed = False
    else:
        classification = "sensor_passed_lio_crosscheck_inconclusive"
        passed = True

    return {
        "gazebo_sensor_passed": gazebo_sensor_passed,
        "lio_crosscheck_passed": lio_crosscheck_passed,
        "lio_reference_valid": lio_reference_valid,
        "classification": classification,
        "passed": passed,
        "lio_crosscheck": calibration.get("best"),
        "lio_reference": {
            "position_rmse_m": trajectory.get("position_rmse_m"),
            "yaw_rmse_deg": trajectory.get("yaw_rmse_deg"),
            "fast_yaw_vs_fcu_gyro_corr": trajectory.get("fast_yaw_vs_fcu_gyro_corr"),
            "estimated_fcu_imu_lag_s": trajectory.get("estimated_fcu_imu_lag_s"),
        },
    }


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--calibration", required=True)
    parser.add_argument("--lio-report", required=True)
    parser.add_argument("--gazebo-log", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--require-pass", action="store_true")
    args = parser.parse_args()

    calibration = json.loads(Path(args.calibration).read_text(encoding="utf-8"))
    lio_report = json.loads(Path(args.lio_report).read_text(encoding="utf-8"))
    gazebo_log = Path(args.gazebo_log).read_text(encoding="utf-8", errors="replace")
    result = evaluate_gate(calibration, lio_report, gazebo_log)
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, indent=2) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2))
    return 0 if result["passed"] or not args.require_pass else 1


if __name__ == "__main__":
    raise SystemExit(main())
