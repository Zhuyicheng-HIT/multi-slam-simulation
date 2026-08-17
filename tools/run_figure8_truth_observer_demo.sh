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
# Experimental observer-only global-height handoff. GNSS Z corrects the
# fusion_map-to-camera_init gauge while the LiDAR/RGB-D map remains local.
export Z_GAUGE_ENABLED=1
export Z_GAUGE_TARGET_HISTORY_SIZE=1
export Z_GAUGE_UPDATE_TIME_CONSTANT_S=0.60
export Z_GAUGE_MAXIMUM_CORRECTION_RATE_MPS=1.0

exec bash "$SCRIPT_DIR/run_map_fusion_demo.sh" "$@"
