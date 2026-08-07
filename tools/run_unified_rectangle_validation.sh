#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/unified_rectangle_$(date +%Y%m%d_%H%M%S)"}
LIDAR_WS=${LIDAR_WS:-"$HOME/multi-slam-deps/mid360_ws"}
VALIDATION_ROUTE=${VALIDATION_ROUTE:-rectangle}
VALIDATION_ROS_DOMAIN_ID=${VALIDATION_ROS_DOMAIN_ID:-41}
if [[ ! "$VALIDATION_ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] ||
  (( VALIDATION_ROS_DOMAIN_ID < 0 || VALIDATION_ROS_DOMAIN_ID > 232 ))
then
  printf 'VALIDATION_ROS_DOMAIN_ID must be an integer in [0, 232].\n' >&2
  exit 2
fi
export ROS_DOMAIN_ID="$VALIDATION_ROS_DOMAIN_ID"
if [[ "$VALIDATION_ROUTE" == "s_curve" ]]; then
  METRICS_DURATION=${METRICS_DURATION:-280}
  DRIFT_DURATION=${DRIFT_DURATION:-270}
  VALIDATION_RELOCALIZATION_WALL_TIMEOUT=${VALIDATION_RELOCALIZATION_WALL_TIMEOUT:-600}
else
  METRICS_DURATION=${METRICS_DURATION:-135}
  DRIFT_DURATION=${DRIFT_DURATION:-125}
  VALIDATION_RELOCALIZATION_WALL_TIMEOUT=${VALIDATION_RELOCALIZATION_WALL_TIMEOUT:-300}
fi
VALIDATION_ENABLE_EXTERNALNAV_EKF3=${VALIDATION_ENABLE_EXTERNALNAV_EKF3:-0}
case "$VALIDATION_ENABLE_EXTERNALNAV_EKF3" in
  1|true|TRUE|yes|YES)
    VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC=${VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC:-/mavros/odometry/out}
    ;;
  *)
    # Estimator-only validation must not feed MAVROS' ODOMETRY plugin. The
    # plugin validates its own NED/FRD TF contract even when EKF3 ignores the
    # measurement, which can otherwise flood logs and delay FCU services.
    VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC=${VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC:-/fusion/externalnav/odom}
    ;;
esac
VALIDATION_MID360_SIM_BRIDGE_MODE=${VALIDATION_MID360_SIM_BRIDGE_MODE:-direct_livox}
VALIDATION_PRESERVE_LIO_ANCHOR=${VALIDATION_PRESERVE_LIO_ANCHOR:-false}
VALIDATION_LOCALIZATION_SAFETY_ENABLED=${VALIDATION_LOCALIZATION_SAFETY_ENABLED:-true}
VALIDATION_RELOCALIZATION_TRIGGER_SIM_S=${VALIDATION_RELOCALIZATION_TRIGGER_SIM_S:-}
VALIDATION_RELOCALIZATION_TRIGGER_PHASE=${VALIDATION_RELOCALIZATION_TRIGGER_PHASE:-}
VALIDATION_RELOCALIZATION_CHECKPOINTS=${VALIDATION_RELOCALIZATION_CHECKPOINTS:-}
relocalization_trigger_modes=0
[[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_SIM_S" ]] &&
  ((relocalization_trigger_modes += 1))
[[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" ]] &&
  ((relocalization_trigger_modes += 1))
[[ -n "$VALIDATION_RELOCALIZATION_CHECKPOINTS" ]] &&
  ((relocalization_trigger_modes += 1))
if (( relocalization_trigger_modes > 1 ))
then
  printf 'Choose one relocalization trigger: simulation time, mission phase, or checkpoints.\n' >&2
  exit 2
fi
if [[ -n "$VALIDATION_RELOCALIZATION_CHECKPOINTS" &&
  "$VALIDATION_ROUTE" != "s_curve" ]]
then
  printf 'Checkpoint relocalization requires VALIDATION_ROUTE=s_curve.\n' >&2
  exit 2
fi
if [[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" ]]; then
  VALIDATION_POST_TAKEOFF_HOLD_TIME=${VALIDATION_POST_TAKEOFF_HOLD_TIME:-10.0}
else
  VALIDATION_POST_TAKEOFF_HOLD_TIME=${VALIDATION_POST_TAKEOFF_HOLD_TIME:-3.0}
fi
if [[ "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" == "final_loop_hold" ]]; then
  # The rectangle mission skips this phase when FINAL_HOLD_TIME is zero. Keep
  # the vehicle stationary long enough for reciprocal registration and the
  # backend epoch acknowledgement to complete before landing.
  VALIDATION_FINAL_HOLD_TIME=${VALIDATION_FINAL_HOLD_TIME:-12.0}
else
  VALIDATION_FINAL_HOLD_TIME=${VALIDATION_FINAL_HOLD_TIME:-0.0}
fi
# The stable path keeps FAST-LIO's internal prediction for deskew/matching but
# exports native factors and gives the unified backend final state/map
# ownership. Set this to 1 only for the experimental backend-trajectory A/B;
# the 2026-08-07 urban run exposed a request/factor deadlock in that mode.
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
printf 'Validation ROS domain: %s\n' "$ROS_DOMAIN_ID"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
if [[ ! -f "$LIDAR_WS/install/setup.bash" ]]; then
  printf 'missing LiDAR workspace overlay: %s\n' "$LIDAR_WS/install/setup.bash" >&2
  exit 2
fi
source "$LIDAR_WS/install/setup.bash"

# This launcher owns the complete validation graph. Duplicate estimator or
# scheduler nodes mix clock domains and can publish competing factor/recovery
# decisions while still looking like a valid ROS topic. Refuse to start until
# the previous graph has been shut down explicitly.
existing_validation_nodes=$(
  timeout 6 ros2 node list --no-daemon --spin-time 2.0 2>/dev/null |
    grep -E '^/(fastlio_mapping|gazebo_clock_bridge|guided_rectangle_waypoints|guided_s_curve_waypoints|reliability_monitor|reliability_scheduler|relocalization_node|unified_backend_fusion|unified_external_nav_gate)$' || true
)
if [[ -n "$existing_validation_nodes" ]]; then
  printf 'A previous validation graph is still active:\n%s\n' \
    "$existing_validation_nodes" >&2
  printf 'Stop that graph before starting an isolated quantitative run.\n' >&2
  exit 2
fi

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
    # Gazebo + ArduPilot cold starts can spend over 80 s before the direct
    # adapter is created. Keep the rate gate strict but leave enough time to
    # collect multiple source stamps after DDS discovery.
    wait_rate /livox/lidar 2.0 150
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
  FASTLIO_BACKEND_TRAJECTORY_FRONTEND="$FASTLIO_BACKEND_TRAJECTORY_FRONTEND" \
  FASTLIO_NATIVE_FACTOR_EXPORT=1 \
  FASTLIO_DOWNSTREAM_BACKEND=1 \
  FASTLIO_DIAGNOSTIC_ODOMETRY=1 \
  FASTLIO_DIAGNOSTIC_PATH=0 \
  FASTLIO_DIAGNOSTIC_TF=0 \
  FASTLIO_MAP_INSERTION_MODE="${FASTLIO_MAP_INSERTION_MODE:-backend_confirmed}" \
  FASTLIO_BACKEND_STATE_TOPIC=/fusion/unified/map_pose \
  FASTLIO_BACKEND_ACTIVATION_STATE_TOPIC=/fusion/unified/odom \
  START_LIVOX_POINTCLOUD_BRIDGE="$fastlio_pointcloud_bridge" \
  LOG_DIR="$LOG_DIR/fastlio" bash "$REPO_ROOT/tools/run_fastlio_mapping.sh" \
  >"$LOG_DIR/fastlio_launcher.log" 2>&1 &
pids+=("$!")

# Start the backend immediately after the frontend. In backend-confirmed map
# mode the frontend intentionally stops advancing after its bounded startup
# queue until the backend acknowledges optimized states.
setsid env ENABLE_VISION=false USE_SIM_TIME=true \
  PRESERVE_LIO_ANCHOR="$VALIDATION_PRESERVE_LIO_ANCHOR" \
  FRONTEND_SCAN_PREDICTION_ENABLED="$frontend_scan_prediction_enabled" \
  EXTERNAL_NAV_OUTPUT_TOPIC="$VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC" \
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
  -p external_nav_topic:="$VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC" \
  >"$LOG_DIR/unified_runtime_metrics.log" 2>&1 &
metrics_pid=$!
pids+=("$metrics_pid")

relocalization_trigger_pid=""
if [[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_SIM_S" ||
  -n "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" ||
  -n "$VALIDATION_RELOCALIZATION_CHECKPOINTS" ]]
then
  if [[ -n "$VALIDATION_RELOCALIZATION_CHECKPOINTS" ]]; then
    python3 "$REPO_ROOT/tools/trigger_relocalization_checkpoints.py" \
      --indices "$VALIDATION_RELOCALIZATION_CHECKPOINTS" \
      --wall-timeout "$VALIDATION_RELOCALIZATION_WALL_TIMEOUT" \
      --output "$LOG_DIR/relocalization_trigger.json" \
      --ros-args -p use_sim_time:=true \
      >"$LOG_DIR/relocalization_trigger.log" 2>&1 &
  else
    relocalization_trigger_args=()
    if [[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" ]]; then
      relocalization_trigger_args+=(
        --wait-for-phase "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE")
    else
      relocalization_trigger_args+=(
        --after "$VALIDATION_RELOCALIZATION_TRIGGER_SIM_S")
    fi
    python3 "$REPO_ROOT/tools/trigger_relocalization_once.py" \
      "${relocalization_trigger_args[@]}" \
      --wall-timeout "$VALIDATION_RELOCALIZATION_WALL_TIMEOUT" \
      --output "$LOG_DIR/relocalization_trigger.json" \
      --ros-args -p use_sim_time:=true \
      >"$LOG_DIR/relocalization_trigger.log" 2>&1 &
  fi
  relocalization_trigger_pid=$!
  pids+=("$relocalization_trigger_pid")
fi

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
env LOCALIZATION_SAFETY_ENABLED="$VALIDATION_LOCALIZATION_SAFETY_ENABLED" \
  POST_TAKEOFF_HOLD_TIME="$VALIDATION_POST_TAKEOFF_HOLD_TIME" \
  FINAL_HOLD_TIME="$VALIDATION_FINAL_HOLD_TIME" \
  bash "$route_script" >"$route_log" 2>&1 &
route_pid=$!
pids+=("$route_pid")

drift_status=0
python3 "$REPO_ROOT/tools/analyze_slam_drift.py" --duration "$DRIFT_DURATION" \
  --output "$LOG_DIR/slam_drift.json" \
  --ros-args -p use_sim_time:=true \
  >"$LOG_DIR/slam_drift.log" 2>&1 || drift_status=$?
metrics_status=0
wait "$metrics_pid" || metrics_status=$?
if [[ -n "$relocalization_trigger_pid" ]]; then
  relocalization_status=0
  wait "$relocalization_trigger_pid" || relocalization_status=$?
  if (( relocalization_status != 0 )); then
    printf 'relocalization_transaction_failed: status %d\n' \
      "$relocalization_status" >&2
    exit "$relocalization_status"
  fi
fi
if [[ -n "$reliability_pid" ]]; then
  reliability_status=0
  wait "$reliability_pid" || reliability_status=$?
fi
route_status=0
wait "$route_pid" || route_status=$?
if (( route_status != 0 )); then
  printf 'route_failed: %s exited with status %d\n' \
    "$VALIDATION_ROUTE" "$route_status" >&2
  exit "$route_status"
fi
if (( drift_status != 0 || metrics_status != 0 )); then
  printf 'metric_collection_failed: drift=%d runtime=%d\n' \
    "$drift_status" "$metrics_status" >&2
  exit 3
fi
if [[ -n "$reliability_pid" ]] && (( reliability_status != 0 )); then
  printf 'reliability_collection_failed: status %d\n' \
    "$reliability_status" >&2
  exit "$reliability_status"
fi
printf 'validation_complete: %s\n' "$LOG_DIR"
