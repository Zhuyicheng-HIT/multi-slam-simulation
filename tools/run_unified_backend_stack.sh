#!/usr/bin/env bash
set -Ee -o pipefail

# Keep every ROS process on the known-good middleware in the simulation.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/unified_backend_$(date +%Y%m%d_%H%M%S)"}
LIDAR_WS=${LIDAR_WS:-"$HOME/multi-slam-deps/mid360_ws"}
ENABLE_VISION=${ENABLE_VISION:-false}
RUNTIME_PROFILE=${RUNTIME_PROFILE:-four_source}
ENABLE_FAULT_INJECTION=${ENABLE_FAULT_INJECTION:-false}
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
  disabled|paper_reprojection|rgbd_direct) ;;
  *)
    printf 'VISUAL_FACTOR_MODE must be disabled, paper_reprojection, or rgbd_direct.\n' >&2
    exit 2
    ;;
esac
RGBD_MINIMUM_DEPTH_M=${RGBD_MINIMUM_DEPTH_M:-0.30}
RGBD_MAXIMUM_DEPTH_M=${RGBD_MAXIMUM_DEPTH_M:-6.0}
RGBD_DEPTH_FACTOR_ENABLED=${RGBD_DEPTH_FACTOR_ENABLED:-false}
RGBD_DEPTH_HEALTHY_LIDAR_STRIDE=${RGBD_DEPTH_HEALTHY_LIDAR_STRIDE:-1}
RANGE_FACET_ENABLED=${RANGE_FACET_ENABLED:-false}
PRESERVE_LIO_ANCHOR=${PRESERVE_LIO_ANCHOR:-false}
BACKEND_NUMERIC_THREADS=${BACKEND_NUMERIC_THREADS:-1}
PERFORMANCE_PROFILING_ENABLED=${PERFORMANCE_PROFILING_ENABLED:-false}
AXIS_INFORMATION_HANDOFF_ENABLED=${AXIS_INFORMATION_HANDOFF_ENABLED:-false}
GNSS_Z_REANCHOR_ENABLED=${GNSS_Z_REANCHOR_ENABLED:-false}
GNSS_Z_RECOVERY_INFORMATION_SCALE=${GNSS_Z_RECOVERY_INFORMATION_SCALE:-0.50}
BAROMETER_FALLBACK_ENABLED=${BAROMETER_FALLBACK_ENABLED:-false}
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
VISUAL_ROTATION_BODY_CAMERA=${VISUAL_ROTATION_BODY_CAMERA:-'[0.0, 0.0, 1.0, -1.0, 0.0, 0.0, 0.0, -1.0, 0.0]'}
VISUAL_TRANSLATION_BODY_CAMERA_M=${VISUAL_TRANSLATION_BODY_CAMERA_M:-'[0.20, 0.0, 0.02]'}
USE_SIM_TIME=${USE_SIM_TIME:-true}
OPTICAL_FLOW_INPUT_TOPIC=${OPTICAL_FLOW_INPUT_TOPIC:-/sim/optical_flow/rad}
D435_COLOR_INPUT_TOPIC=${D435_COLOR_INPUT_TOPIC:-/front/d435i/color/image_raw}
D435_DEPTH_INPUT_TOPIC=${D435_DEPTH_INPUT_TOPIC:-/front/d435i/aligned_depth_to_color/image_raw}
SENSOR_PIPELINE_CONFIG=${SENSOR_PIPELINE_CONFIG:-$REPO_ROOT/src/ultra_fusion_nav/uf_sensor_pipeline/config/sim_sensor_config.yaml}
GNSS_INPUT_TOPIC=${GNSS_INPUT_TOPIC:-/mavros/global_position/raw/fix}
GNSS_RAW_INPUT_TOPIC=${GNSS_RAW_INPUT_TOPIC:-/mavros/gpsstatus/gps1/raw}
GNSS_ALGORITHM_RATE_HZ=${GNSS_ALGORITHM_RATE_HZ:-5.0}
ACTIVE_MODALITIES=${ACTIVE_MODALITIES:-[lidar,imu,gnss,optical_flow]}
if [[ -z "${SENSOR_ACTIVE_MODALITIES+x}" ]]; then
  case "$RUNTIME_PROFILE" in
    minimal_lidar_imu) SENSOR_ACTIVE_MODALITIES='[lidar,imu]'; ACTIVE_MODALITIES='[lidar,imu]' ;;
    four_source) SENSOR_ACTIVE_MODALITIES='[lidar,imu,gnss,optical_flow]'; ACTIVE_MODALITIES='[lidar,imu,gnss,optical_flow]' ;;
    five_source) SENSOR_ACTIVE_MODALITIES='[lidar,imu,gnss,optical_flow,depth,color]'; ACTIVE_MODALITIES='[lidar,imu,gnss,optical_flow,vision]'; ENABLE_VISION=true ;;
    robustness|test) SENSOR_ACTIVE_MODALITIES='[lidar,imu,gnss,optical_flow,depth,color]'; ACTIVE_MODALITIES='[lidar,imu,gnss,optical_flow,vision]'; ENABLE_FAULT_INJECTION=true ;;
    *) printf 'RUNTIME_PROFILE must be minimal_lidar_imu, four_source, five_source, or robustness/test.\n' >&2; exit 2 ;;
  esac
fi
if [[ "$SENSOR_ACTIVE_MODALITIES" == *gnss* ]]; then ENABLE_GNSS_ARG=true; else ENABLE_GNSS_ARG=false; fi
case "$RUNTIME_PROFILE" in
  five_source|robustness|test)
    ENABLE_VISION=true
    [[ "$VISUAL_FACTOR_MODE" == disabled ]] && VISUAL_FACTOR_MODE=paper_reprojection
    ENABLE_VISION_ARG=true
    ;;
esac
if [[ -z "${BAROMETER_TOPIC+x}" ]]; then
  case "${USE_SIM_TIME,,}" in
    1|true|yes|on) BAROMETER_TOPIC=/sim/barometer/pressure ;;
    *) BAROMETER_TOPIC=/mavros/imu/static_pressure ;;
  esac
fi
for toggle_name in \
  AXIS_INFORMATION_HANDOFF_ENABLED \
  GNSS_Z_REANCHOR_ENABLED \
  BAROMETER_FALLBACK_ENABLED \
  RANGE_FACET_ENABLED
do
  toggle_value=${!toggle_name}
  case "${toggle_value,,}" in
    1|true|yes|on) printf -v "${toggle_name}_ARG" '%s' true ;;
    0|false|no|off) printf -v "${toggle_name}_ARG" '%s' false ;;
    *)
      printf '%s must be true/false or 1/0.\n' "$toggle_name" >&2
      exit 2
      ;;
  esac
done
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
  config:="$SENSOR_PIPELINE_CONFIG" \
  use_sim_time:="$USE_SIM_TIME" \
  enable_vision:="$ENABLE_VISION_ARG" \
  active_modalities:="$SENSOR_ACTIVE_MODALITIES" \
  enable_fault_injection:="$ENABLE_FAULT_INJECTION" \
  enable_lidar:="true" \
  enable_gnss:="$ENABLE_GNSS_ARG" \
  optical_flow_input_topic:="$OPTICAL_FLOW_INPUT_TOPIC" \
  d435_color_input_topic:="$D435_COLOR_INPUT_TOPIC" \
  d435_depth_input_topic:="$D435_DEPTH_INPUT_TOPIC" \
  gnss_input_topic:="$GNSS_INPUT_TOPIC" \
  gnss_raw_input_topic:="$GNSS_RAW_INPUT_TOPIC" \
  gnss_algorithm_rate_hz:="$GNSS_ALGORITHM_RATE_HZ" \
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
  active_modalities:="$ACTIVE_MODALITIES" \
  visual_factor_mode:="$VISUAL_FACTOR_MODE" \
  rgbd_minimum_depth_m:="$RGBD_MINIMUM_DEPTH_M" \
  rgbd_maximum_depth_m:="$RGBD_MAXIMUM_DEPTH_M" \
  rgbd_depth_factor_enabled:="$RGBD_DEPTH_FACTOR_ENABLED" \
  rgbd_depth_healthy_lidar_stride:="$RGBD_DEPTH_HEALTHY_LIDAR_STRIDE" \
  range_facet_enabled:="$RANGE_FACET_ENABLED_ARG" \
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
  visual_rotation_body_camera:="$VISUAL_ROTATION_BODY_CAMERA" \
  visual_translation_body_camera_m:="$VISUAL_TRANSLATION_BODY_CAMERA_M" \
  barometer_topic:="$BAROMETER_TOPIC" \
  axis_information_handoff_enabled:="$AXIS_INFORMATION_HANDOFF_ENABLED_ARG" \
  gnss_z_reanchor_enabled:="$GNSS_Z_REANCHOR_ENABLED_ARG" \
  gnss_z_recovery_information_scale:="$GNSS_Z_RECOVERY_INFORMATION_SCALE" \
  barometer_fallback_enabled:="$BAROMETER_FALLBACK_ENABLED_ARG" \
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
