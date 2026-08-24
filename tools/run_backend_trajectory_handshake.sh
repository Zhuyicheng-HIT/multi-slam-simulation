#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
LOG_DIR=${LOG_DIR:-/tmp/uf_backend_trajectory_handshake}
HANDSHAKE_DURATION_S=${HANDSHAKE_DURATION_S:-20}

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
  source "$LIDAR_WS/install/local_setup.bash"

mkdir -p "$LOG_DIR"
wait_rate() {
  python3 "$REPO_ROOT/tools/topic_rate_probe.py" \
    --topic "$1" --minimum-hz "$2" --timeout "$3"
}
pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 3
  for pid in "${pids[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

setsid env HEADLESS=1 REQUIRE_GAZEBO_GPU=1 \
  ENABLE_D435_POINTCLOUD=false USE_SIM_TIME=true \
  ENABLE_EXTERNALNAV_EKF3=0 ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV=0 \
  MID360_SIM_BRIDGE_MODE=direct_livox LOG_DIR="$LOG_DIR/sim" \
  bash "$REPO_ROOT/tools/run_sim_with_unified_externalnav.sh" \
  >"$LOG_DIR/sim_launcher.log" 2>&1 &
pids+=("$!")
wait_rate /livox/lidar 2.0 90
wait_rate /mavros/imu/data_raw 20.0 40

setsid env LIDAR_WS="$LIDAR_WS" RVIZ=0 FASTLIO_INPUT_MODE=livox \
  USE_SIM_TIME=true FASTLIO_NATIVE_FACTOR_EXPORT=1 \
  FASTLIO_DOWNSTREAM_BACKEND=1 FASTLIO_DIAGNOSTIC_ODOMETRY=1 \
  FASTLIO_MAP_INSERTION_MODE=backend_confirmed \
  FASTLIO_BACKEND_STATE_TOPIC=/fusion/unified/odom \
  FASTLIO_BACKEND_TRAJECTORY_FRONTEND=1 \
  START_LIVOX_POINTCLOUD_BRIDGE=0 LOG_DIR="$LOG_DIR/fastlio" \
  bash "$REPO_ROOT/tools/run_fastlio_mapping.sh" \
  >"$LOG_DIR/fastlio_launcher.log" 2>&1 &
pids+=("$!")

setsid env ENABLE_VISION=false USE_SIM_TIME=true \
  FRONTEND_SCAN_PREDICTION_ENABLED=true LOG_DIR="$LOG_DIR/unified" \
  bash "$REPO_ROOT/tools/run_unified_backend_stack.sh" \
  >"$LOG_DIR/unified_launcher.log" 2>&1 &
pids+=("$!")

wait_rate /fusion/unified/odom 2.0 60
python3 "$REPO_ROOT/tools/unified_runtime_metrics.py" \
  --duration "$HANDSHAKE_DURATION_S" \
  --output "$LOG_DIR/handshake_runtime_metrics.json" \
  --ros-args -p use_sim_time:=true \
  >"$LOG_DIR/handshake_runtime_metrics.log" 2>&1

for topic in \
  /fast_lio/frontend_scan_request \
  /fusion/unified/backend_deskew_trajectory \
  /fast_lio/native_lidar_factor \
  /fusion/unified/odom; do
  name=${topic#/}
  name=${name//\//_}
  ros2 topic info "$topic" -v >"$LOG_DIR/${name}_info.txt" 2>&1 || true
  timeout 8s ros2 topic hz "$topic" >"$LOG_DIR/${name}_hz.txt" 2>&1 || true
done

cleanup
sleep 2

grep 'Backend trajectory frontend summary:' \
  "$LOG_DIR/fastlio/fast_lio.log" | tail -n 1 \
  >"$LOG_DIR/fastlio_handshake_summary.txt" || true
grep 'Unified backend final summary:' \
  "$LOG_DIR/unified/online_backend.log" | tail -n 1 \
  >"$LOG_DIR/backend_handshake_summary.txt" || true

cat "$LOG_DIR/fastlio_handshake_summary.txt"
cat "$LOG_DIR/backend_handshake_summary.txt"

printf 'backend_trajectory_handshake_complete: %s\n' "$LOG_DIR"
