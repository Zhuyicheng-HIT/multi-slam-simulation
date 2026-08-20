#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
profile=${LARGE_SCENE_PROFILE:-city_static}
run_id=${LARGE_SCENE_RUN_ID:-$(date +%Y%m%d_%H%M%S)}

export VALIDATION_ROUTE=s_curve
export VALIDATION_RECORD_REPLAY_BAG=${VALIDATION_RECORD_REPLAY_BAG:-true}
export VALIDATION_RECORD_RAW_LIDAR=${VALIDATION_RECORD_RAW_LIDAR:-false}
export VALIDATION_STOP_OBSERVERS_ON_LANDING=${VALIDATION_STOP_OBSERVERS_ON_LANDING:-true}
export VALIDATION_STOP_AFTER_LANDING=${VALIDATION_STOP_AFTER_LANDING:-true}
export VALIDATION_LANDING_GRACE_S=${VALIDATION_LANDING_GRACE_S:-5}
export ENABLE_RELIABILITY_RECORD=${ENABLE_RELIABILITY_RECORD:-1}
export S_CURVE_PASSES=1

case "$profile" in
  city_static|city_dynamic|city_dynamic_relocalization)
    export VALIDATION_WORLD_NAME=apm_city_rgbd_mid360
    export VALIDATION_WORLD_PATH="$REPO_ROOT/src/multi_slam_uav_sim/worlds/apm_city_rgbd_mid360.sdf"
    export VALIDATION_GAZEBO_WORLD_NAME=city_apm_rgbd
    export VALIDATION_DYNAMIC_AGENTS_CONFIG="$REPO_ROOT/src/multi_slam_uav_sim/config/city_apm_motion_params.yaml"
    export VALIDATION_TAKEOFF_ALT=${VALIDATION_TAKEOFF_ALT:-3.0}
    export S_CURVE_SPAN=${S_CURVE_SPAN:-20.0}
    export S_CURVE_AMPLITUDE=${S_CURVE_AMPLITUDE:-2.0}
    export S_CURVE_VERTICAL_AMPLITUDE=${S_CURVE_VERTICAL_AMPLITUDE:-1.2}
    export FIGURE8_ROTATION_DEG=${FIGURE8_ROTATION_DEG:-0.0}
    export S_CURVE_SPEED=${S_CURVE_SPEED:-0.60}
    export VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M=${VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M:-50.0}
    export VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS=${VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS:-20}
    export METRICS_DURATION=${METRICS_DURATION:-360}
    export DRIFT_DURATION=${DRIFT_DURATION:-350}
    ;;
  tunnel_static|tunnel_dynamic|tunnel_dynamic_relocalization)
    export VALIDATION_WORLD_NAME=large_indoor_tunnel_apm_rgbd_mid360
    export VALIDATION_WORLD_PATH="$REPO_ROOT/src/multi_slam_uav_sim/worlds/large_indoor_tunnel_apm_rgbd_mid360.sdf"
    export VALIDATION_GAZEBO_WORLD_NAME=large_indoor_tunnel
    export VALIDATION_DYNAMIC_AGENTS_CONFIG="$REPO_ROOT/src/multi_slam_uav_sim/config/large_tunnel_motion_params.yaml"
    export VALIDATION_TAKEOFF_ALT=${VALIDATION_TAKEOFF_ALT:-2.2}
    export S_CURVE_SPAN=${S_CURVE_SPAN:-70.0}
    export S_CURVE_AMPLITUDE=${S_CURVE_AMPLITUDE:-1.0}
    export S_CURVE_VERTICAL_AMPLITUDE=${S_CURVE_VERTICAL_AMPLITUDE:-0.35}
    export FIGURE8_ROTATION_DEG=${FIGURE8_ROTATION_DEG:-90.0}
    export S_CURVE_SPEED=${S_CURVE_SPEED:-0.80}
    export VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M=${VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M:-140.0}
    export VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS=${VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS:-45}
    export METRICS_DURATION=${METRICS_DURATION:-480}
    export DRIFT_DURATION=${DRIFT_DURATION:-470}
    ;;
  *)
    printf 'Unknown LARGE_SCENE_PROFILE=%s\n' "$profile" >&2
    printf 'Known profiles: city_static, city_dynamic, city_dynamic_relocalization, tunnel_static, tunnel_dynamic, tunnel_dynamic_relocalization\n' >&2
    exit 2
    ;;
esac

case "$profile" in
  *_dynamic|*_dynamic_relocalization)
    export VALIDATION_DYNAMIC_AGENTS_ENABLED=true
    ;;
  *)
    export VALIDATION_DYNAMIC_AGENTS_ENABLED=false
    ;;
esac

case "$profile" in
  *_relocalization)
    export VALIDATION_RELOCALIZATION_CHECKPOINTS=${VALIDATION_RELOCALIZATION_CHECKPOINTS:-8,16}
    export VALIDATION_RELOCALIZATION_MOTION_PROFILE=${VALIDATION_RELOCALIZATION_MOTION_PROFILE:-yaw_scan}
    export VALIDATION_RELOCALIZATION_VELOCITY_POLICY=${VALIDATION_RELOCALIZATION_VELOCITY_POLICY:-rotate}
    export VALIDATION_RELOCALIZATION_BIAS_POLICY=${VALIDATION_RELOCALIZATION_BIAS_POLICY:-preserve}
    export VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S=${VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S:-20.0}
    ;;
  *)
    export VALIDATION_RELOCALIZATION_CHECKPOINTS=
    ;;
esac

export LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/large_scene_${profile}_${run_id}"}
mkdir -p "$LOG_DIR"
{
  printf 'profile=%s\n' "$profile"
  printf 'git_commit=%s\n' "$(git -C "$REPO_ROOT" rev-parse HEAD)"
  printf 'world_profile=%s\n' "$VALIDATION_WORLD_NAME"
  printf 'gazebo_world=%s\n' "$VALIDATION_GAZEBO_WORLD_NAME"
  printf 'dynamic_agents=%s\n' "$VALIDATION_DYNAMIC_AGENTS_ENABLED"
  printf 'route_span_m=%s\n' "$S_CURVE_SPAN"
  printf 'route_amplitude_m=%s\n' "$S_CURVE_AMPLITUDE"
  printf 'route_vertical_amplitude_m=%s\n' "$S_CURVE_VERTICAL_AMPLITUDE"
  printf 'route_speed_mps=%s\n' "$S_CURVE_SPEED"
  printf 'relocalization_checkpoints=%s\n' "$VALIDATION_RELOCALIZATION_CHECKPOINTS"
  printf 'body_envelope_m=0.50,0.50,0.10\n'
} >"$LOG_DIR/campaign_profile.env"

printf 'Large-scene profile: %s\n' "$profile"
printf 'Evidence directory: %s\n' "$LOG_DIR"
exec bash "$REPO_ROOT/tools/run_frozen_low_figure8_validation.sh" "$@"
