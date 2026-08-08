#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
MATRIX_DIR=${MATRIX_DIR:-$REPO_ROOT/logs/tmp/robustness_v3_1_lidar_temporal}
PROFILE_PATH=${PROFILE_PATH:-$REPO_ROOT/src/ultra_fusion_nav/uf_sensor_pipeline/config/robustness_v3_1_lidar_temporal.yaml}
BAG_DIR=${BAG_DIR:-$REPO_ROOT/logs/tmp/robustness_v3_frozen_clock_replay_c}
REFERENCE=${REFERENCE:-$REPO_ROOT/logs/tmp/robustness_v3_nominal_c_clock/estimate.tum}
REPLAY_RATE=${REPLAY_RATE:-1.0}
mkdir -p "$MATRIX_DIR"

suffixes=(0 p0005 n0005 p001 n001 p002 n002 p005 n005 p010 n010 p020 n020)
for scope in a1_coherent a2_mismatch; do
  for suffix in "${suffixes[@]}"; do
    profile="${scope}_${suffix}"
    output="$MATRIX_DIR/$profile"
    if [[ -s "$output/robustness_report.json" && ${RESUME_EXISTING:-0} == 1 ]]; then
      continue
    fi
    mkdir -p "$output"
    printf 'temporal profile=%s\n' "$profile"
    PROFILE_PATH="$PROFILE_PATH" PROFILE="$profile" FRS_MODE=on \
      BAG_DIR="$BAG_DIR" TRUTH_PATH="$REFERENCE" \
      REFERENCE_ESTIMATE_PATH="$REFERENCE" REPLAY_RATE="$REPLAY_RATE" \
      OUTPUT_DIR="$output" ROS_DOMAIN_ID=184 \
      "$SCRIPT_DIR/run_robustness_v3_replay.sh" \
      >"$output/driver.log" 2>&1
  done
done

python3 "$SCRIPT_DIR/summarize_robustness_v3_1_temporal.py" \
  --matrix-dir "$MATRIX_DIR" --output "$MATRIX_DIR/temporal_summary.json"
