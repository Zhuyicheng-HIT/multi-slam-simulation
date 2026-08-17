#!/usr/bin/env bash
set -Ee -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/unified_backend_$(date +%Y%m%d_%H%M%S)"}
LIDAR_WS=${LIDAR_WS:-"$HOME/multi-slam-deps/mid360_ws"}
ENABLE_VISION=${ENABLE_VISION:-false}
case "${ENABLE_VISION,,}" in
  1|true|yes|on)
    ENABLE_VISION_ARG=true
    VISUAL_FACTOR_MODE=${VISUAL_FACTOR_MODE:-paper_reprojection}
    ;;
  0|false|no|off)
    ENABLE_VISION_ARG=false
    VISUAL_FACTOR_MODE=${VISUAL_FACTOR_MODE:-disabled}
    ;;
  *)
    printf 'ENABLE_VISION must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
case "$VISUAL_FACTOR_MODE" in
  disabled|paper_reprojection) ;;
  *)
    printf 'VISUAL_FACTOR_MODE must be disabled or paper_reprojection.\n' >&2
    exit 2
    ;;
esac
RGBD_MINIMUM_DEPTH_M=${RGBD_MINIMUM_DEPTH_M:-0.30}
RGBD_MAXIMUM_DEPTH_M=${RGBD_MAXIMUM_DEPTH_M:-6.0}
RGBD_DEPTH_FACTOR_ENABLED=${RGBD_DEPTH_FACTOR_ENABLED:-true}
RGBD_DEPTH_HEALTHY_LIDAR_STRIDE=${RGBD_DEPTH_HEALTHY_LIDAR_STRIDE:-1}
PRESERVE_LIO_ANCHOR=${PRESERVE_LIO_ANCHOR:-false}
BACKEND_NUMERIC_THREADS=${BACKEND_NUMERIC_THREADS:-1}
PERFORMANCE_PROFILING_ENABLED=${PERFORMANCE_PROFILING_ENABLED:-false}
RELIABILITY_MODE=${RELIABILITY_MODE:-dynamic}
FIXED_LIDAR_WEIGHT=${FIXED_LIDAR_WEIGHT:-1.0}
FIXED_GNSS_WEIGHT=${FIXED_GNSS_WEIGHT:-1.0}
FIXED_IMU_WEIGHT=${FIXED_IMU_WEIGHT:-1.0}
FIXED_OPTICAL_FLOW_WEIGHT=${FIXED_OPTICAL_FLOW_WEIGHT:-1.0}
FIXED_VISION_WEIGHT=${FIXED_VISION_WEIGHT:-1.0}
FRONTEND_MAP_COMMIT_DELAY_STATES=${FRONTEND_MAP_COMMIT_DELAY_STATES:-7}
CALIBRATION_APPLY_LOCKED_TIME_OFFSET=${CALIBRATION_APPLY_LOCKED_TIME_OFFSET:-false}
CALIBRATION_APPLY_LOCKED_ROTATION=${CALIBRATION_APPLY_LOCKED_ROTATION:-false}
VISUAL_TIME_CALIBRATION_APPLY_LOCKED=${VISUAL_TIME_CALIBRATION_APPLY_LOCKED:-false}
Z_GAUGE_ENABLED=${Z_GAUGE_ENABLED:-false}
Z_GAUGE_GLOBAL_FRAME=${Z_GAUGE_GLOBAL_FRAME:-fusion_map}
Z_GAUGE_TARGET_HISTORY_SIZE=${Z_GAUGE_TARGET_HISTORY_SIZE:-1}
Z_GAUGE_UPDATE_TIME_CONSTANT_S=${Z_GAUGE_UPDATE_TIME_CONSTANT_S:-0.60}
Z_GAUGE_MAXIMUM_CORRECTION_RATE_MPS=${Z_GAUGE_MAXIMUM_CORRECTION_RATE_MPS:-1.0}
USE_SIM_TIME=${USE_SIM_TIME:-true}
OPTICAL_FLOW_INPUT_TOPIC=${OPTICAL_FLOW_INPUT_TOPIC:-/sim/optical_flow/rad}
if [[ -z "${BAROMETER_TOPIC+x}" ]]; then
  case "${USE_SIM_TIME,,}" in
    1|true|yes|on) BAROMETER_TOPIC=/sim/barometer/pressure ;;
    *) BAROMETER_TOPIC=/mavros/imu/static_pressure ;;
  esac
fi
FRONTEND_STATE_SEED_ENABLED=${FRONTEND_STATE_SEED_ENABLED:-false}
# The unified backend owns the trajectory by default.  Keep the legacy
# FAST-LIO-local trajectory available only as an explicit compatibility mode.
# Keep the proven FAST-LIO-local deskew/matching predictor as the stable
# default. The backend-owned trajectory handshake remains an explicit A/B
# mode until it can sustain long runs without a request/factor deadlock.
FRONTEND_SCAN_PREDICTION_ENABLED=${FRONTEND_SCAN_PREDICTION_ENABLED:-false}
EXTERNAL_NAV_OUTPUT_TOPIC=${EXTERNAL_NAV_OUTPUT_TOPIC:-/mavros/odometry/out}
RELOCALIZATION_SEARCH_TIMEOUT_S=${RELOCALIZATION_SEARCH_TIMEOUT_S:-6.0}
if [[ -z "${PUBLISH_MAVROS_FRAME_TRANSFORMS+x}" ]]; then
  if [[ "$EXTERNAL_NAV_OUTPUT_TOPIC" == "/mavros/odometry/out" ]]; then
    PUBLISH_MAVROS_FRAME_TRANSFORMS=true
  else
    PUBLISH_MAVROS_FRAME_TRANSFORMS=false
  fi
fi

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
if [[ ! -f "$LIDAR_WS/install/setup.bash" ]]; then
  printf 'Patched FAST-LIO overlay is required: %s/install/setup.bash\n' "$LIDAR_WS" >&2
  exit 2
fi
source "$LIDAR_WS/install/setup.bash"
if ! ros2 interface show fast_lio/msg/NativeLidarFactor >/dev/null 2>&1; then
  printf 'Patched FAST-LIO NativeLidarFactor interface is unavailable.\n' >&2
  exit 2
fi
case "${FRONTEND_STATE_SEED_ENABLED,,}" in
  1|true|yes|on) FRONTEND_STATE_SEED_ENABLED_ARG=true ;;
  0|false|no|off) FRONTEND_STATE_SEED_ENABLED_ARG=false ;;
  *)
    printf 'FRONTEND_STATE_SEED_ENABLED must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
case "${FRONTEND_SCAN_PREDICTION_ENABLED,,}" in
  1|true|yes|on) FRONTEND_SCAN_PREDICTION_ENABLED_ARG=true ;;
  0|false|no|off) FRONTEND_SCAN_PREDICTION_ENABLED_ARG=false ;;
  *)
    printf 'FRONTEND_SCAN_PREDICTION_ENABLED must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
case "${Z_GAUGE_ENABLED,,}" in
  1|true|yes|on) Z_GAUGE_ENABLED_ARG=true ;;
  0|false|no|off) Z_GAUGE_ENABLED_ARG=false ;;
  *)
    printf 'Z_GAUGE_ENABLED must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
if [[ "$FRONTEND_STATE_SEED_ENABLED_ARG" == "true" ]] &&
   ! ros2 interface show fast_lio/msg/BackendStateSeed >/dev/null 2>&1; then
  printf 'Patched FAST-LIO BackendStateSeed interface is unavailable.\n' >&2
  exit 2
fi
if [[ "$FRONTEND_SCAN_PREDICTION_ENABLED_ARG" == "true" ]]; then
  if ! ros2 interface show fast_lio/msg/FrontendScanRequest >/dev/null 2>&1 ||
     ! ros2 interface show fast_lio/msg/BackendDeskewTrajectory >/dev/null 2>&1; then
    printf 'Patched FAST-LIO scan prediction interfaces are unavailable.\n' >&2
    exit 2
  fi
fi
mkdir -p "$LOG_DIR"

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

setsid ros2 launch uf_sensor_pipeline sensor_pipeline.launch.py \
  use_sim_time:="$USE_SIM_TIME" \
  enable_vision:="$ENABLE_VISION_ARG" \
  optical_flow_input_topic:="$OPTICAL_FLOW_INPUT_TOPIC" \
  >"$LOG_DIR/sensor_pipeline.log" 2>&1 &
pids+=("$!")

setsid ros2 launch uf_lio_adapter lio_adapter.launch.py \
  use_sim_time:="$USE_SIM_TIME" \
  >"$LOG_DIR/lio_adapter.log" 2>&1 &
pids+=("$!")

setsid env \
  OMP_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  OPENBLAS_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  MKL_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  NUMEXPR_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  ros2 launch uf_backend_fusion online_backend.launch.py \
  use_sim_time:="$USE_SIM_TIME" \
  enable_vision:="$ENABLE_VISION_ARG" \
  visual_factor_mode:="$VISUAL_FACTOR_MODE" \
  rgbd_minimum_depth_m:="$RGBD_MINIMUM_DEPTH_M" \
  rgbd_maximum_depth_m:="$RGBD_MAXIMUM_DEPTH_M" \
  rgbd_depth_factor_enabled:="$RGBD_DEPTH_FACTOR_ENABLED" \
  rgbd_depth_healthy_lidar_stride:="$RGBD_DEPTH_HEALTHY_LIDAR_STRIDE" \
  preserve_lio_anchor:="$PRESERVE_LIO_ANCHOR" \
  frontend_state_seed_enabled:="$FRONTEND_STATE_SEED_ENABLED_ARG" \
  frontend_scan_prediction_enabled:="$FRONTEND_SCAN_PREDICTION_ENABLED_ARG" \
  performance_profiling_enabled:="$PERFORMANCE_PROFILING_ENABLED" \
  reliability_mode:="$RELIABILITY_MODE" \
  fixed_lidar_weight:="$FIXED_LIDAR_WEIGHT" \
  fixed_gnss_weight:="$FIXED_GNSS_WEIGHT" \
  fixed_imu_weight:="$FIXED_IMU_WEIGHT" \
  fixed_optical_flow_weight:="$FIXED_OPTICAL_FLOW_WEIGHT" \
  fixed_vision_weight:="$FIXED_VISION_WEIGHT" \
  frontend_map_commit_delay_states:="$FRONTEND_MAP_COMMIT_DELAY_STATES" \
  calibration_apply_locked_time_offset:="$CALIBRATION_APPLY_LOCKED_TIME_OFFSET" \
  calibration_apply_locked_rotation:="$CALIBRATION_APPLY_LOCKED_ROTATION" \
  visual_time_calibration_apply_locked:="$VISUAL_TIME_CALIBRATION_APPLY_LOCKED" \
  z_gauge_enabled:="$Z_GAUGE_ENABLED_ARG" \
  z_gauge_global_frame:="$Z_GAUGE_GLOBAL_FRAME" \
  z_gauge_target_history_size:="$Z_GAUGE_TARGET_HISTORY_SIZE" \
  z_gauge_update_time_constant_s:="$Z_GAUGE_UPDATE_TIME_CONSTANT_S" \
  z_gauge_maximum_correction_rate_mps:="$Z_GAUGE_MAXIMUM_CORRECTION_RATE_MPS" \
  barometer_topic:="$BAROMETER_TOPIC" \
  external_nav_output_topic:="$EXTERNAL_NAV_OUTPUT_TOPIC" \
  publish_mavros_frame_transforms:="$PUBLISH_MAVROS_FRAME_TRANSFORMS" \
  relocalization_search_timeout_s:="$RELOCALIZATION_SEARCH_TIMEOUT_S" \
  >"$LOG_DIR/online_backend.log" 2>&1 &
pids+=("$!")

printf 'Unified backend stack started. Logs: %s\n' "$LOG_DIR"
# All three launch processes are intended to remain alive together. Stop the
# stack as soon as any one exits so callers cannot mistake stale DDS discovery
# or a surviving sidecar for a healthy unified backend.
if wait -n "${pids[@]}"; then
  status=0
else
  status=$?
fi
printf 'Unified backend stack child exited unexpectedly (status=%s).\n' \
  "$status" >&2
exit "$status"
