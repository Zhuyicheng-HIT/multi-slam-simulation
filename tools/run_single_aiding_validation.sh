#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
AIDING_MODALITY=${1:-${AIDING_MODALITY:-}}
SINGLE_AIDING_SCENE_PROFILE=${SINGLE_AIDING_SCENE_PROFILE:-}

case "$AIDING_MODALITY" in
  lidar)
    active_modalities='[imu,lidar]'
    enable_vision=0
    lidar_weight=1.0
    gnss_weight=0.0
    flow_weight=0.0
    vision_weight=0.0
    ;;
  gnss)
    active_modalities='[imu,gnss]'
    enable_vision=0
    lidar_weight=0.0
    gnss_weight=1.0
    flow_weight=0.0
    vision_weight=0.0
    ;;
  optical_flow)
    active_modalities='[imu,optical_flow]'
    enable_vision=0
    lidar_weight=0.0
    gnss_weight=0.0
    flow_weight=1.0
    vision_weight=0.0
    ;;
  vision)
    active_modalities='[imu,vision]'
    enable_vision=1
    lidar_weight=0.0
    gnss_weight=0.0
    flow_weight=0.0
    vision_weight=1.0
    export VALIDATION_VISUAL_FACTOR_MODE=${VALIDATION_VISUAL_FACTOR_MODE:-rgbd_direct}
    ;;
  *)
    printf 'Usage: %s {lidar|gnss|optical_flow|vision}\n' "$0" >&2
    exit 2
    ;;
esac

export LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/single_aiding_${AIDING_MODALITY}_$(date +%Y%m%d_%H%M%S)"}
export ACTIVE_MODALITIES="$active_modalities"
export VALIDATION_FACTOR_PROFILE="$AIDING_MODALITY"
export VALIDATION_RELIABILITY_MODE=fixed
export VALIDATION_ENABLE_EXTERNALNAV_EKF3=${VALIDATION_ENABLE_EXTERNALNAV_EKF3:-0}
export VALIDATION_ROUTE_FEEDBACK_SOURCE=${VALIDATION_ROUTE_FEEDBACK_SOURCE:-fcu_local}
export VALIDATION_LOCALIZATION_SAFETY_ENABLED=${VALIDATION_LOCALIZATION_SAFETY_ENABLED:-false}
export VALIDATION_REQUIRE_FASTLIO_DRIFT=${VALIDATION_REQUIRE_FASTLIO_DRIFT:-false}
export VALIDATION_ENABLE_VISION="$enable_vision"
export VALIDATION_REQUIRE_VISUAL_FACTORS="$enable_vision"
export FIXED_IMU_WEIGHT=1.0
export FIXED_LIDAR_WEIGHT="$lidar_weight"
export FIXED_GNSS_WEIGHT="$gnss_weight"
export FIXED_OPTICAL_FLOW_WEIGHT="$flow_weight"
export FIXED_VISION_WEIGHT="$vision_weight"
export VALIDATION_RANGE_FACET_ENABLED=false
export VALIDATION_AXIS_INFORMATION_HANDOFF_ENABLED=false
export VALIDATION_GNSS_Z_REANCHOR_ENABLED=false
export VALIDATION_BAROMETER_FALLBACK_ENABLED=false
export VALIDATION_RECORD_REPLAY_BAG=${VALIDATION_RECORD_REPLAY_BAG:-false}
export VALIDATION_RECORD_FASTLIO_ACCURACY=${VALIDATION_RECORD_FASTLIO_ACCURACY:-false}
export VALIDATION_RECORD_RELIABILITY_TIMELINE=${VALIDATION_RECORD_RELIABILITY_TIMELINE:-false}
export VALIDATION_RECORD_SLAM_DRIFT=${VALIDATION_RECORD_SLAM_DRIFT:-false}
export VALIDATION_RESOURCE_INTERVAL_S=${VALIDATION_RESOURCE_INTERVAL_S:-2.0}

printf 'Single-aiding estimator validation: modality=%s log=%s\n' \
  "$AIDING_MODALITY" "$LOG_DIR"
printf 'FCU route feedback is isolated from unified ExternalNav; FAST-LIO remains the native LiDAR frontend.\n'

if [[ -n "$SINGLE_AIDING_SCENE_PROFILE" ]]; then
  export LARGE_SCENE_PROFILE="$SINGLE_AIDING_SCENE_PROFILE"
  exec bash "$REPO_ROOT/tools/run_large_scene_validation.sh"
fi

exec bash "$REPO_ROOT/tools/run_unified_rectangle_validation.sh"
