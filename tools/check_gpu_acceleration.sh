#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
PKG_ROOT="$REPO_ROOT/src/multi_slam_uav_sim"

source "$PKG_ROOT/scripts/env.sh"
exec bash "$PKG_ROOT/scripts/check_gpu_acceleration.sh" "$@"
