#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/unified_rectangle_$(date +%Y%m%d_%H%M%S)"}
LIDAR_WS=${LIDAR_WS:-"$HOME/multi-slam-deps/mid360_ws"}
mkdir -p "$LOG_DIR"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"

wait_rate() {
  local topic=$1
  local minimum_hz=$2
  local timeout_s=$3
  local output
  local started=$SECONDS
  while (( SECONDS - started < timeout_s )); do
    output=$(ros2 topic hz "$topic" --window 5 2>/dev/null | grep -m1 'average rate' || true)
    if [[ -n "$output" ]]; then
      rate=$(awk '{print $3}' <<<"$output")
      if awk -v rate="$rate" -v minimum="$minimum_hz" 'BEGIN { exit !(rate >= minimum) }'; then
        printf 'ready: %s %.3f Hz\n' "$topic" "$rate"
        return 0
      fi
    fi
    sleep 1
  done
  printf 'timeout: %s did not reach %.2f Hz\n' "$topic" "$minimum_hz" >&2
  return 1
}

pids=()
cleanup() {
  for pid in "${pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
}
trap cleanup EXIT INT TERM

setsid env HEADLESS=1 REQUIRE_GAZEBO_GPU=1 ENABLE_D435_POINTCLOUD=false \
  ENABLE_EXTERNALNAV_EKF3=1 ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV=0 \
  LOG_DIR="$LOG_DIR/sim" bash "$REPO_ROOT/tools/run_sim_with_unified_externalnav.sh" \
  >"$LOG_DIR/sim_launcher.log" 2>&1 &
pids+=("$!")
wait_rate /sim/mid360/points_raw 2.0 90
wait_rate /mavros/imu/data_raw 20.0 40

setsid env LIDAR_WS="$LIDAR_WS" RVIZ=0 FASTLIO_INPUT_MODE=livox \
  LOG_DIR="$LOG_DIR/fastlio" bash "$REPO_ROOT/tools/run_fastlio_mapping.sh" \
  >"$LOG_DIR/fastlio_launcher.log" 2>&1 &
pids+=("$!")
wait_rate /Odometry 1.0 60

setsid env ENABLE_VISION=false LOG_DIR="$LOG_DIR/unified" \
  bash "$REPO_ROOT/tools/run_unified_backend_stack.sh" \
  >"$LOG_DIR/unified_launcher.log" 2>&1 &
pids+=("$!")
wait_rate /fusion/unified/odom 2.0 60
wait_rate /mavros/odometry/out 10.0 30

python3 "$REPO_ROOT/tools/unified_runtime_metrics.py" --duration 135 \
  --output "$LOG_DIR/unified_runtime_metrics.json" \
  >"$LOG_DIR/unified_runtime_metrics.log" 2>&1 &
metrics_pid=$!
pids+=("$metrics_pid")

bash "$REPO_ROOT/tools/run_rectangle_state_machine.sh" \
  >"$LOG_DIR/rectangle.log" 2>&1 &
rectangle_pid=$!
pids+=("$rectangle_pid")

python3 "$REPO_ROOT/tools/analyze_slam_drift.py" --duration 125 \
  --output "$LOG_DIR/slam_drift.json" \
  >"$LOG_DIR/slam_drift.log" 2>&1 || true
wait "$metrics_pid" || true
wait "$rectangle_pid" || true
printf 'validation_complete: %s\n' "$LOG_DIR"
