#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

export DEMO_ROUTE=figure8
export DEMO_GUI=1
export DEMO_RVIZ=1
export KEEP_DEMO_OPEN=1

exec bash "$SCRIPT_DIR/run_map_fusion_demo.sh" "$@"
