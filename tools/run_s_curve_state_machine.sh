#!/usr/bin/env bash
set -Eeuo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
TARGET=install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_s_curve_state_machine.sh
"$SCRIPT_DIR/ensure_built.sh" "$TARGET"
exec bash "$REPO_ROOT/$TARGET" "$@"
