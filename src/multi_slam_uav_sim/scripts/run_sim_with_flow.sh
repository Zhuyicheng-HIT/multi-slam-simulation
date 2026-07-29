#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

export ENABLE_GAZEBO_FLOW="${ENABLE_GAZEBO_FLOW:-1}"
export SHOW_FLOW_WINDOW="${SHOW_FLOW_WINDOW:-1}"
export FLOW_DEBUG="${FLOW_DEBUG:-true}"
export RECTANGLE_FLOW_TEST="${RECTANGLE_FLOW_TEST:-0}"
export AUTO_FLIGHT="${AUTO_FLIGHT:-0}"

exec bash "$SCRIPT_DIR/run_apm_sensor_stack.sh"
