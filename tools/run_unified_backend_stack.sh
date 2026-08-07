#!/usr/bin/env bash
set -Ee -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/unified_backend_$(date +%Y%m%d_%H%M%S)"}
LIDAR_WS=${LIDAR_WS:-"$HOME/multi-slam-deps/mid360_ws"}
ENABLE_VISION=${ENABLE_VISION:-false}
PRESERVE_LIO_ANCHOR=${PRESERVE_LIO_ANCHOR:-false}
BACKEND_NUMERIC_THREADS=${BACKEND_NUMERIC_THREADS:-1}
USE_SIM_TIME=${USE_SIM_TIME:-true}
FRONTEND_STATE_SEED_ENABLED=${FRONTEND_STATE_SEED_ENABLED:-false}
# The unified backend owns the trajectory by default.  Keep the legacy
# FAST-LIO-local trajectory available only as an explicit compatibility mode.
# Keep the proven FAST-LIO-local deskew/matching predictor as the stable
# default. The backend-owned trajectory handshake remains an explicit A/B
# mode until it can sustain long runs without a request/factor deadlock.
FRONTEND_SCAN_PREDICTION_ENABLED=${FRONTEND_SCAN_PREDICTION_ENABLED:-false}
EXTERNAL_NAV_OUTPUT_TOPIC=${EXTERNAL_NAV_OUTPUT_TOPIC:-/mavros/odometry/out}

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
  use_sim_time:="$USE_SIM_TIME" \
  enable_vision:="$ENABLE_VISION" \
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
  preserve_lio_anchor:="$PRESERVE_LIO_ANCHOR" \
  frontend_state_seed_enabled:="$FRONTEND_STATE_SEED_ENABLED_ARG" \
  frontend_scan_prediction_enabled:="$FRONTEND_SCAN_PREDICTION_ENABLED_ARG" \
  external_nav_output_topic:="$EXTERNAL_NAV_OUTPUT_TOPIC" \
  >"$LOG_DIR/online_backend.log" 2>&1 &
pids+=("$!")

printf 'Unified backend stack started. Logs: %s\n' "$LOG_DIR"
wait "${pids[@]}"
