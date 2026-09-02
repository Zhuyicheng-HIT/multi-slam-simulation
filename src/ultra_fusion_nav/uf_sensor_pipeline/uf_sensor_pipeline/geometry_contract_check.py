"""Validate the real MID360S/D435i geometry contract without publishing TF."""

import argparse
import json
from typing import Optional, Sequence

from .geometry_contract import closure_report, load_geometry_contract


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("contract", help="YAML path or package:// URI")
    parser.add_argument("--json", action="store_true", help="emit machine-readable output")
    parser.add_argument(
        "--require-complete",
        action="store_true",
        help="exit 2 while body/camera closure is incomplete",
    )
    return parser


def main(argv: Optional[Sequence[str]] = None) -> int:
    args = _parser().parse_args(argv)
    contract = load_geometry_contract(args.contract)
    closure = closure_report(contract)
    closed_statuses = ("MEASURED", "DERIVED")
    result = {
        "frames": {
            "body": contract.frames.body,
            "lidar": contract.frames.lidar,
            "camera_optical": contract.frames.camera_optical,
        },
        "body_lidar_rotation_usable": True,
        "body_lidar_translation_available": contract.body_lidar.translation is not None,
        "body_lidar_translation_measured": (
            contract.body_lidar.translation_status == "measured"
        ),
        "body_lidar_translation_status": contract.body_lidar.translation_status,
        "camera_lidar_calibrated": contract.camera_lidar.status == "calibrated",
        "body_camera_status": (
            contract.body_camera.status if contract.body_camera is not None else "missing"
        ),
        "hardware_tf_publishable": closure.status in closed_statuses,
        "body_filter_enabled": bool(contract.body_envelope.get("enabled", False)),
        "closure": {
            "status": closure.status,
            "missing": list(closure.missing),
            "translation_residual_m": closure.translation_residual_m,
            "rotation_residual_rad": closure.rotation_residual_rad,
        },
    }
    if args.json:
        print(json.dumps(result, sort_keys=True))
    else:
        print(f"geometry closure: {closure.status}")
        if closure.missing:
            print("missing: " + ", ".join(closure.missing))
        print(f"hardware TF publishable: {result['hardware_tf_publishable']}")
        print(f"body filter enabled: {result['body_filter_enabled']}")
    return 2 if args.require_complete and closure.status not in closed_statuses else 0


if __name__ == "__main__":
    raise SystemExit(main())
