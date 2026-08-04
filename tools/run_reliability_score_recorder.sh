#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"

exec python3 "$REPO_ROOT/src/ultra_fusion_nav/scripts/record_reliability_scores.py" "$@"
