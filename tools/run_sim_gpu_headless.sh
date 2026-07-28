#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

export GAZEBO_GPU_ADAPTER="${GAZEBO_GPU_ADAPTER:-NVIDIA}"
export REQUIRE_GAZEBO_GPU="${REQUIRE_GAZEBO_GPU:-1}"
export HEADLESS="${HEADLESS:-1}"
export SHOW_FLOW_WINDOW="${SHOW_FLOW_WINDOW:-0}"
export FLOW_DEBUG="${FLOW_DEBUG:-false}"
export ENABLE_D435_BRIDGE="${ENABLE_D435_BRIDGE:-0}"
export ENABLE_D435_POINTCLOUD="${ENABLE_D435_POINTCLOUD:-false}"

exec bash "$SCRIPT_DIR/run_sim_with_flow.sh" "$@"
