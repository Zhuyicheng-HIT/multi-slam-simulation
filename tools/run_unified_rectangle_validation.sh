#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/unified_rectangle_$(date +%Y%m%d_%H%M%S)"}
LIDAR_WS=${LIDAR_WS:-"$HOME/multi-slam-deps/mid360_ws"}
VALIDATION_ROUTE=${VALIDATION_ROUTE:-rectangle}
METRICS_DURATION=${METRICS_DURATION:-135}
DRIFT_DURATION=${DRIFT_DURATION:-125}
VALIDATION_ENABLE_EXTERNALNAV_EKF3=${VALIDATION_ENABLE_EXTERNALNAV_EKF3:-0}
VALIDATION_MID360_SIM_BRIDGE_MODE=${VALIDATION_MID360_SIM_BRIDGE_MODE:-direct_livox}
FASTLIO_BACKEND_TRAJECTORY_FRONTEND=${FASTLIO_BACKEND_TRAJECTORY_FRONTEND:-0}
case "$FASTLIO_BACKEND_TRAJECTORY_FRONTEND" in
  1|true|TRUE|yes|YES) frontend_scan_prediction_enabled=true ;;
  0|false|FALSE|no|NO) frontend_scan_prediction_enabled=false ;;
  *)
    printf 'FASTLIO_BACKEND_TRAJECTORY_FRONTEND must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
mkdir -p "$LOG_DIR"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
if [[ ! -f "$LIDAR_WS/install/setup.bash" ]]; then
  printf 'missing LiDAR workspace overlay: %s\n' "$LIDAR_WS/install/setup.bash" >&2
  exit 2
fi
source "$LIDAR_WS/install/setup.bash"

wait_rate() {
  local topic=$1
  local minimum_hz=$2
  local timeout_s=$3
  python3 "$REPO_ROOT/tools/topic_rate_probe.py" \
    --topic "$topic" --minimum-hz "$minimum_hz" --timeout "$timeout_s"
}

pids=()
cleanup_started=0
cleanup() {
  if [[ "$cleanup_started" == "1" ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  sitl_pid=""
  if [[ -f "$LOG_DIR/sim/arducopter.pid" ]]; then
    sitl_pid=$(cat "$LOG_DIR/sim/arducopter.pid" 2>/dev/null || true)
    if [[ "$sitl_pid" =~ ^[0-9]+$ ]]; then
      kill -INT "$sitl_pid" 2>/dev/null || true
    fi
  fi
  for pid in "${pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  if [[ "$sitl_pid" =~ ^[0-9]+$ ]]; then
    kill -TERM "$sitl_pid" 2>/dev/null || true
  fi
  for pid in "${pids[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 2
  if [[ "$sitl_pid" =~ ^[0-9]+$ ]]; then
    kill -KILL "$sitl_pid" 2>/dev/null || true
  fi
  for pid in "${pids[@]:-}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    kill -KILL "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

setsid env HEADLESS=1 REQUIRE_GAZEBO_GPU=1 ENABLE_D435_POINTCLOUD=false \
  USE_SIM_TIME=true \
  ENABLE_EXTERNALNAV_EKF3="$VALIDATION_ENABLE_EXTERNALNAV_EKF3" \
  ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV=0 \
  MID360_SIM_BRIDGE_MODE="$VALIDATION_MID360_SIM_BRIDGE_MODE" \
  LOG_DIR="$LOG_DIR/sim" bash "$REPO_ROOT/tools/run_sim_with_unified_externalnav.sh" \
  >"$LOG_DIR/sim_launcher.log" 2>&1 &
pids+=("$!")
case "$VALIDATION_MID360_SIM_BRIDGE_MODE" in
  direct_livox)
    fastlio_pointcloud_bridge=0
    wait_rate /livox/lidar 2.0 90
    ;;
  pointcloud_python)
    fastlio_pointcloud_bridge=1
    wait_rate /sim/mid360/points_raw 2.0 90
    ;;
  *)
    printf 'Unsupported validation MID360 bridge mode: %s\n' \
      "$VALIDATION_MID360_SIM_BRIDGE_MODE" >&2
    exit 2
    ;;
esac
wait_rate /mavros/imu/data_raw 20.0 40

setsid env LIDAR_WS="$LIDAR_WS" RVIZ=0 FASTLIO_INPUT_MODE=livox \
  USE_SIM_TIME=true \
  FASTLIO_NATIVE_FACTOR_EXPORT=1 \
  FASTLIO_DOWNSTREAM_BACKEND=1 \
  FASTLIO_DIAGNOSTIC_ODOMETRY=1 \
  FASTLIO_DIAGNOSTIC_PATH=0 \
  FASTLIO_DIAGNOSTIC_TF=0 \
  FASTLIO_MAP_INSERTION_MODE="${FASTLIO_MAP_INSERTION_MODE:-backend_confirmed}" \
  FASTLIO_BACKEND_STATE_TOPIC=/fusion/unified/odom \
  START_LIVOX_POINTCLOUD_BRIDGE="$fastlio_pointcloud_bridge" \
  LOG_DIR="$LOG_DIR/fastlio" bash "$REPO_ROOT/tools/run_fastlio_mapping.sh" \
  >"$LOG_DIR/fastlio_launcher.log" 2>&1 &
pids+=("$!")

# Start the backend immediately after the frontend. In backend-confirmed map
# mode the frontend intentionally stops advancing after its bounded startup
# queue until the backend acknowledges optimized states.
setsid env ENABLE_VISION=false USE_SIM_TIME=true \
  FRONTEND_SCAN_PREDICTION_ENABLED="$frontend_scan_prediction_enabled" \
  LOG_DIR="$LOG_DIR/unified" \
  bash "$REPO_ROOT/tools/run_unified_backend_stack.sh" \
  >"$LOG_DIR/unified_launcher.log" 2>&1 &
pids+=("$!")

case "${FASTLIO_BACKEND_TRAJECTORY_FRONTEND:-0}" in
  1|true|TRUE|yes|YES)
    printf 'Backend-owned trajectory mode: FAST-LIO diagnostic topics are not backend startup dependencies.\n'
    ;;
  *)
    # The native factor packet, not FAST-LIO's diagnostic pose, is the backend
    # keyframe clock. Requiring /Odometry here can deadlock backend-confirmed
    # map insertion before the unified backend has started.
    wait_rate /fast_lio/native_lidar_factor 1.0 60
    if ! wait_rate /Odometry 1.0 10; then
      printf 'FAST-LIO diagnostic /Odometry is not continuously ready yet; continuing on native factors.\n'
    fi
    ;;
esac
wait_rate /fusion/unified/odom 2.0 60
case "$VALIDATION_ENABLE_EXTERNALNAV_EKF3" in
  # The backend first accumulates a stationary IMU bias window and initializes
  # its first native LiDAR state. Preserve the 10 Hz continuity requirement,
  # but do not spend its entire readiness budget on cold-start initialization.
  1|true|TRUE|yes|YES) wait_rate /mavros/odometry/out 10.0 75 ;;
  *) printf 'ExternalNav FCU consumption disabled; output continuity is metrics-only.\n' ;;
esac

setsid ros2 run multi_slam_uav_sim external_nav_accuracy --ros-args \
  -p use_sim_time:=true \
  -p odom_topic:=/Odometry \
  -p output_path:="$LOG_DIR/fastlio_accuracy.json" \
  >"$LOG_DIR/fastlio_accuracy.log" 2>&1 &
pids+=("$!")

setsid ros2 run multi_slam_uav_sim external_nav_accuracy --ros-args \
  -p use_sim_time:=true \
  -p odom_topic:=/fusion/unified/odom \
  -p output_path:="$LOG_DIR/unified_accuracy.json" \
  >"$LOG_DIR/unified_accuracy.log" 2>&1 &
pids+=("$!")

python3 "$REPO_ROOT/tools/unified_runtime_metrics.py" --duration "$METRICS_DURATION" \
  --output "$LOG_DIR/unified_runtime_metrics.json" \
  --ros-args -p use_sim_time:=true \
  >"$LOG_DIR/unified_runtime_metrics.log" 2>&1 &
metrics_pid=$!
pids+=("$metrics_pid")

reliability_pid=""
case "${ENABLE_RELIABILITY_RECORD:-0}" in
  1|true|TRUE|yes|YES)
    setsid bash "$REPO_ROOT/tools/run_reliability_score_recorder.sh" \
      --duration "$METRICS_DURATION" \
      --output "$LOG_DIR/reliability_scores.csv" \
      --allow-missing \
      >"$LOG_DIR/reliability_scores.log" 2>&1 &
    reliability_pid=$!
    pids+=("$reliability_pid")
    ;;
esac

case "$VALIDATION_ROUTE" in
  rectangle)
    route_script="$REPO_ROOT/tools/run_rectangle_state_machine.sh"
    route_log="$LOG_DIR/rectangle.log"
    ;;
  s_curve)
    route_script="$REPO_ROOT/tools/run_s_curve_state_machine.sh"
    route_log="$LOG_DIR/s_curve.log"
    ;;
  *)
    printf 'Unknown VALIDATION_ROUTE=%s (expected rectangle or s_curve)\n' \
      "$VALIDATION_ROUTE" >&2
    exit 2
    ;;
esac
bash "$route_script" >"$route_log" 2>&1 &
route_pid=$!
pids+=("$route_pid")

python3 "$REPO_ROOT/tools/analyze_slam_drift.py" --duration "$DRIFT_DURATION" \
  --output "$LOG_DIR/slam_drift.json" \
  --ros-args -p use_sim_time:=true \
  >"$LOG_DIR/slam_drift.log" 2>&1 || true
wait "$metrics_pid" || true
if [[ -n "$reliability_pid" ]]; then
  wait "$reliability_pid" || true
fi
route_status=0
wait "$route_pid" || route_status=$?
if (( route_status != 0 )); then
  printf 'route_failed: %s exited with status %d\n' \
    "$VALIDATION_ROUTE" "$route_status" >&2
  exit "$route_status"
fi
printf 'validation_complete: %s\n' "$LOG_DIR"
