import argparse

from .replay import write_replay_outputs
from .rosbag_factors import extract_factors


def extract_main() -> int:
    parser = argparse.ArgumentParser(description="Extract Ultra-Fusion factors from a ROS 2 bag")
    parser.add_argument("--bag", required=True, help="rosbag2 directory")
    parser.add_argument("--output", required=True, help="factor JSON path")
    args = parser.parse_args()
    result = extract_factors(args.bag, args.output)
    print(f"frames={result['frame_count']} output={args.output}")
    print(f"streams={result['streams']}")
    print(f"imu_preintegration={result['imu_preintegration_summary']}")
    return 0


def replay_main() -> int:
    parser = argparse.ArgumentParser(description="Replay fixed and scheduler-weighted factors")
    parser.add_argument("--factors", required=True, help="factor JSON path")
    parser.add_argument("--output-dir", required=True)
    parser.add_argument("--window-size", type=int, default=20)
    args = parser.parse_args()
    import json
    with open(args.factors, encoding="utf-8") as stream:
        data = json.load(stream)
    summary = write_replay_outputs(data, args.output_dir, args.window_size)
    for report in summary["variants"]:
        print(
            f"{report['variant']}: frames={report['frame_count']} "
            f"factors={report['factor_count']} "
            f"reference_rmse={report['position_rmse_vs_evaluation_reference_m']}"
        )
    return 0
