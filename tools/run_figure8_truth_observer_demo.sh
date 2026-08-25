#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Diagnostic isolation only: Gazebo truth closes the mission-control loop.
# The complete estimator and map still run, but neither unified odometry nor
# ExternalNav can control or pause the aircraft.
export DEMO_ROUTE=figure8
export DEMO_GUI=1
export DEMO_RVIZ=1
export KEEP_DEMO_OPEN=1
export DEMO_ROUTE_FEEDBACK_SOURCE=gazebo_truth
export DEMO_EXTERNAL_NAV_ENABLED=0
export DEMO_LOCALIZATION_SAFETY_ENABLED=false
export DEMO_PERFORMANCE_PROFILING_ENABLED=1

exec bash "$SCRIPT_DIR/run_map_fusion_demo.sh" "$@"
