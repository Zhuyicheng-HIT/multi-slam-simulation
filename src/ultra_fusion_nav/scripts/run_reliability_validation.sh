#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
RUN_ID=${RUN_ID:-$(date +%Y%m%d_%H%M%S)}
OUTPUT_DIR=${OUTPUT_DIR:-$REPO_ROOT/logs/uf_stage3_${RUN_ID}}
mkdir -p "$OUTPUT_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

ros2 run uf_reliability reliability_monitor \
  >"$OUTPUT_DIR/reliability_monitor.stdout.log" \
  2>"$OUTPUT_DIR/reliability_monitor.stderr.log" &
monitor_pid=$!
cleanup() {
  kill -TERM "$monitor_pid" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

sleep 2
python3 "$SCRIPT_DIR/validate_reliability_runtime.py" \
  --output "$OUTPUT_DIR/score_validation.json"
python3 "$SCRIPT_DIR/plot_reliability_scores.py" \
  --input "$OUTPUT_DIR/score_validation.json" \
  --output "$OUTPUT_DIR/score_validation.png"
printf 'Stage 3 validation: %s\n' "$OUTPUT_DIR"
