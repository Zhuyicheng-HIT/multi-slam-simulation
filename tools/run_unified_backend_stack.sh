#!/usr/bin/env bash
set -Ee -o pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/unified_backend_$(date +%Y%m%d_%H%M%S)"}
LIDAR_WS=${LIDAR_WS:-"$HOME/multi-slam-deps/mid360_ws"}
ENABLE_VISION=${ENABLE_VISION:-false}
PRESERVE_LIO_ANCHOR=${PRESERVE_LIO_ANCHOR:-true}
BACKEND_NUMERIC_THREADS=${BACKEND_NUMERIC_THREADS:-1}

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
  enable_vision:="$ENABLE_VISION" \
  >"$LOG_DIR/sensor_pipeline.log" 2>&1 &
pids+=("$!")

setsid ros2 launch uf_lio_adapter lio_adapter.launch.py \
  >"$LOG_DIR/lio_adapter.log" 2>&1 &
pids+=("$!")

setsid env \
  OMP_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  OPENBLAS_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  MKL_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  NUMEXPR_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  ros2 launch uf_backend_fusion online_backend.launch.py \
  preserve_lio_anchor:="$PRESERVE_LIO_ANCHOR" \
  >"$LOG_DIR/online_backend.log" 2>&1 &
pids+=("$!")

printf 'Unified backend stack started. Logs: %s\n' "$LOG_DIR"
wait "${pids[@]}"
