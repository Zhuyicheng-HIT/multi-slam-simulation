#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
RUN_ID=${RUN_ID:-$(date +%Y%m%d_%H%M%S)}
OUTPUT_DIR=${OUTPUT_DIR:-$REPO_ROOT/logs/uf_stage4_gnss_${RUN_ID}}
mkdir -p "$OUTPUT_DIR"

set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

python3 "$SCRIPT_DIR/validate_gnss_reanchor.py" \
  --output-json "$OUTPUT_DIR/reanchor_validation.json" \
  --output-csv "$OUTPUT_DIR/reanchor_timeline.csv"
python3 "$SCRIPT_DIR/plot_gnss_reanchor.py" \
  --input "$OUTPUT_DIR/reanchor_timeline.csv" \
  --output "$OUTPUT_DIR/reanchor_timeline.png"
printf 'Stage 4 GNSS re-anchor validation: %s\n' "$OUTPUT_DIR"
