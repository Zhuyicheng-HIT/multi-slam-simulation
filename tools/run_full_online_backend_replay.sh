#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=${WORKSPACE_ROOT:-$REPO_ROOT}
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
BAG_DIR=${BAG_DIR:?Set BAG_DIR to the frozen full-online rosbag directory}
OUTPUT_DIR=${OUTPUT_DIR:-$REPO_ROOT/logs/tmp/full_online_replay_$(date +%Y%m%d_%H%M%S)}
REPLAY_RATE=${REPLAY_RATE:-1.0}
ENABLE_CYCLE_TRACE=${ENABLE_CYCLE_TRACE:-1}
BACKEND_CPUSET=${BACKEND_CPUSET:-}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-67}
export ROS_DOMAIN_ID

source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"
source "$LIDAR_WS/install/setup.bash"
mkdir -p "$OUTPUT_DIR"

pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${pids[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

backend_command=(
  ros2 run uf_backend_fusion online_backend_fusion
  --ros-args
  --params-file "$WORKSPACE_ROOT/install/uf_backend_fusion/share/uf_backend_fusion/config/online_backend.yaml"
  -p use_sim_time:=true
  -p visual_factor_mode:=paper_reprojection
  -p visual_pending_enabled:=true
  -p native_lidar_factor_enabled:=true
  -p input_trigger_mode:=native_factor
  -p frontend_scan_prediction_enabled:=true
  -p allow_lio_pose_fallback:=false
  -p imu_factor_enabled:=true
)
if [[ "$ENABLE_CYCLE_TRACE" == 1 ]]; then
  backend_command+=(
    -p performance_profiling_enabled:=true
    -p performance_trace_path:="$OUTPUT_DIR/backend_cycle_trace.jsonl"
  )
fi
if [[ -n "$BACKEND_CPUSET" ]]; then
  backend_command=(taskset --cpu-list "$BACKEND_CPUSET" "${backend_command[@]}")
fi

setsid env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  NUMEXPR_NUM_THREADS=1 "${backend_command[@]}" \
  >"$OUTPUT_DIR/backend.log" 2>&1 &
pids+=("$!")

setsid python3 "$REPO_ROOT/tools/record_backend_replay_metrics.py" \
  --output "$OUTPUT_DIR/replay_metrics.json" --wall-timeout 1800 \
  >"$OUTPUT_DIR/metrics_recorder.log" 2>&1 &
recorder_pid=$!
pids+=("$recorder_pid")
sleep 4

set +e
timeout 1800s ros2 bag play "$BAG_DIR" --rate "$REPLAY_RATE" \
  >"$OUTPUT_DIR/rosbag_play.log" 2>&1
play_status=$?
set -e
sleep 5
kill -INT "$recorder_pid" 2>/dev/null || true
for _ in {1..30}; do
  [[ -s "$OUTPUT_DIR/replay_metrics.json" ]] && break
  sleep 0.2
done
cleanup
trap - EXIT INT TERM

printf 'play_status=%s\nworkspace=%s\nbag=%s\nrate=%s\ncpuset=%s\n' \
  "$play_status" "$WORKSPACE_ROOT" "$BAG_DIR" "$REPLAY_RATE" \
  "${BACKEND_CPUSET:-normal}" >"$OUTPUT_DIR/replay_result.env"
if [[ ! -s "$OUTPUT_DIR/replay_metrics.json" ]]; then
  printf 'Replay metrics were not written.\n' >&2
  exit 3
fi
cat "$OUTPUT_DIR/replay_metrics.json"
exit "$play_status"
