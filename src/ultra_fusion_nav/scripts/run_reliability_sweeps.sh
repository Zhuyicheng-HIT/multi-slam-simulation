#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
RUN_ID=${RUN_ID:-$(date +%Y%m%d_%H%M%S)}
OUTPUT_DIR=${OUTPUT_DIR:-$REPO_ROOT/logs/uf_stage3_sweep_${RUN_ID}}
mkdir -p "$OUTPUT_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

python3 "$SCRIPT_DIR/validate_reliability_sweeps.py" \
  --output-json "$OUTPUT_DIR/sweep_validation.json" \
  --output-csv "$OUTPUT_DIR/sweep_scores.csv"
python3 "$SCRIPT_DIR/plot_reliability_sweeps.py" \
  --input "$OUTPUT_DIR/sweep_scores.csv" \
  --output "$OUTPUT_DIR/sweep_scores.png"
printf 'Stage 3 sweep validation: %s\n' "$OUTPUT_DIR"
