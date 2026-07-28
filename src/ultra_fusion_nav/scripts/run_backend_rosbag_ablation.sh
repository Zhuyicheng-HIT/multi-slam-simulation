#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
BAG_DIR=${1:?usage: run_backend_rosbag_ablation.sh BAG_DIR [OUTPUT_DIR]}
OUTPUT_DIR=${2:-$REPO_ROOT/logs/backend_ablation_$(date +%Y%m%d_%H%M%S)}
PLAYBACK_RATE=${PLAYBACK_RATE:-1.0}
POST_ROLL_S=${POST_ROLL_S:-3}
ABLATION_MODES=${ABLATION_MODES:-"fixed dynamic"}
RESYNTHESIZE_RELIABILITY=${RESYNTHESIZE_RELIABILITY:-1}
REPLAY_OPTICAL_FLOW_SCALE=${REPLAY_OPTICAL_FLOW_SCALE:-1.0}
REPLAY_FLOW_FAULT_START_SOURCE_S=${REPLAY_FLOW_FAULT_START_SOURCE_S:-0.0}
REPLAY_FLOW_FAULT_DURATION_SOURCE_S=${REPLAY_FLOW_FAULT_DURATION_SOURCE_S:-0.0}

if [[ ! -f "$BAG_DIR/metadata.yaml" ]]; then
  printf 'rosbag2 metadata not found: %s/metadata.yaml\n' "$BAG_DIR" >&2
  exit 2
fi
if awk "BEGIN {exit !($REPLAY_OPTICAL_FLOW_SCALE != 1.0)}" \
    && [[ "$RESYNTHESIZE_RELIABILITY" != "1" ]]; then
  printf '%s\n' \
    'replay optical-flow faults require RESYNTHESIZE_RELIABILITY=1' >&2
  exit 2
fi

mkdir -p "$OUTPUT_DIR"
export OPENBLAS_NUM_THREADS=${NUMPY_NUM_THREADS:-1}
export MKL_NUM_THREADS=${NUMPY_NUM_THREADS:-1}
export NUMEXPR_NUM_THREADS=${NUMPY_NUM_THREADS:-1}
set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
if [[ -f "$HOME/multi-slam-deps/mid360_ws/install/setup.bash" ]]; then
  source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
fi
set -u

bag_duration_s=$(ros2 bag info "$BAG_DIR" | awk '/Duration:/ {gsub(/s/, "", $2); print $2; exit}')
bag_info=$(ros2 bag info "$BAG_DIR")
if [[ -z "$bag_duration_s" ]]; then
  printf 'could not read rosbag2 duration: %s\n' "$BAG_DIR" >&2
  exit 2
fi
record_duration_s=$(awk -v duration="$bag_duration_s" -v rate="$PLAYBACK_RATE" \
  -v post="$POST_ROLL_S" 'BEGIN {printf "%.3f", duration / rate + post + 2.0}')

playback_topics=(
  /fast_lio/native_lidar_factor
  /lio/odom
  /sensors/gnss/fix
  /sensors/imu
  /sensors/optical_flow/rad
  /sim/mid360/ground_truth_odom
)
if grep -Fq 'Topic: /lio/diagnostics ' <<<"$bag_info"; then
  playback_topics+=(/lio/diagnostics)
elif grep -Fq 'Topic: /reliability/lidar_score ' <<<"$bag_info"; then
  playback_topics+=(/reliability/lidar_score)
else
  printf '%s\n' \
    'bag has neither /lio/diagnostics nor /reliability/lidar_score' >&2
  exit 2
fi
for optional_topic in /mavros/gpsstatus/gps1/raw /fault/state; do
  if grep -Fq "Topic: $optional_topic " <<<"$bag_info"; then
    playback_topics+=("$optional_topic")
  fi
done

pids=()
cleanup() {
  set +e
  for pid in "${pids[@]:-}"; do
    kill -INT "$pid" 2>/dev/null
    kill -INT -- "-$pid" 2>/dev/null
  done
}
trap cleanup EXIT INT TERM

run_mode() {
  local mode=$1
  local mode_dir="$OUTPUT_DIR/$mode"
  mkdir -p "$mode_dir/trajectory"

  local monitor_pid=""
  local scheduler_pid=""
  local replay_flow_fault_pid=""
  local replay_flow_trigger_pid=""
  if [[ "$RESYNTHESIZE_RELIABILITY" == "1" ]]; then
    setsid ros2 run uf_reliability reliability_monitor --ros-args \
      --params-file "$REPO_ROOT/install/uf_reliability/share/uf_reliability/config/reliability.yaml" \
      >"$mode_dir/reliability_monitor.stdout.log" \
      2>"$mode_dir/reliability_monitor.stderr.log" &
    monitor_pid=$!
    pids+=("$monitor_pid")
    setsid ros2 run uf_reliability reliability_scheduler --ros-args \
      --params-file "$REPO_ROOT/install/uf_reliability/share/uf_reliability/config/scheduler_config.yaml" \
      >"$mode_dir/reliability_scheduler.stdout.log" \
      2>"$mode_dir/reliability_scheduler.stderr.log" &
    scheduler_pid=$!
    pids+=("$scheduler_pid")
  fi

  if awk "BEGIN {exit !($REPLAY_OPTICAL_FLOW_SCALE != 1.0)}"; then
    setsid ros2 run uf_sensor_pipeline fault_injector --ros-args \
      -r __node:=replay_flow_fault_injector \
      -p modality:=optical_flow \
      -p input_topic:=/replay/optical_flow/rad \
      -p output_topic:=/sensors/optical_flow/rad \
      -p fault_type:=none \
      -p magnitude:="$REPLAY_OPTICAL_FLOW_SCALE" \
      >"$mode_dir/replay_flow_fault.stdout.log" \
      2>"$mode_dir/replay_flow_fault.stderr.log" &
    replay_flow_fault_pid=$!
    pids+=("$replay_flow_fault_pid")
  fi

  setsid ros2 run uf_backend_fusion online_backend_fusion --ros-args \
    --params-file "$REPO_ROOT/install/uf_backend_fusion/share/uf_backend_fusion/config/online_backend.yaml" \
    -p reliability_mode:="$mode" \
    >"$mode_dir/backend.stdout.log" 2>"$mode_dir/backend.stderr.log" &
  local backend_pid=$!
  pids+=("$backend_pid")

  python3 "$SCRIPT_DIR/record_lio_trajectory.py" \
    --duration "$record_duration_s" \
    --output-dir "$mode_dir/trajectory" \
    --estimate-topic /fusion/unified/odom \
    >"$mode_dir/recorder.stdout.log" 2>"$mode_dir/recorder.stderr.log" &
  local recorder_pid=$!
  pids+=("$recorder_pid")

  python3 "$SCRIPT_DIR/record_reliability_timeline.py" \
    --duration "$record_duration_s" \
    --output "$mode_dir/reliability_timeline.json" \
    >"$mode_dir/timeline.stdout.log" 2>"$mode_dir/timeline.stderr.log" &
  local timeline_pid=$!
  pids+=("$timeline_pid")

  sleep 1
  if [[ -n "$replay_flow_fault_pid" ]]; then
    local fault_start_wall_s
    local fault_duration_wall_s
    fault_start_wall_s=$(awk \
      -v source="$REPLAY_FLOW_FAULT_START_SOURCE_S" -v rate="$PLAYBACK_RATE" \
      'BEGIN {printf "%.6f", source / rate}')
    fault_duration_wall_s=$(awk \
      -v source="$REPLAY_FLOW_FAULT_DURATION_SOURCE_S" -v rate="$PLAYBACK_RATE" \
      'BEGIN {printf "%.6f", source / rate}')
    (
      sleep "$fault_start_wall_s"
      ros2 param set /replay_flow_fault_injector fault_type scale
      if awk "BEGIN {exit !($fault_duration_wall_s > 0.0)}"; then
        sleep "$fault_duration_wall_s"
        ros2 param set /replay_flow_fault_injector fault_type none
      fi
    ) >"$mode_dir/replay_flow_trigger.log" 2>&1 &
    replay_flow_trigger_pid=$!
    pids+=("$replay_flow_trigger_pid")
  fi
  if [[ "$RESYNTHESIZE_RELIABILITY" == "1" ]]; then
    if [[ -n "$replay_flow_fault_pid" ]]; then
      ros2 bag play "$BAG_DIR" --rate "$PLAYBACK_RATE" \
        --topics "${playback_topics[@]}" \
        --remap /sensors/optical_flow/rad:=/replay/optical_flow/rad \
        >"$mode_dir/play.stdout.log" 2>"$mode_dir/play.stderr.log"
    else
      ros2 bag play "$BAG_DIR" --rate "$PLAYBACK_RATE" \
        --topics "${playback_topics[@]}" \
        >"$mode_dir/play.stdout.log" 2>"$mode_dir/play.stderr.log"
    fi
  else
    ros2 bag play "$BAG_DIR" --rate "$PLAYBACK_RATE" \
      >"$mode_dir/play.stdout.log" 2>"$mode_dir/play.stderr.log"
  fi
  sleep "$POST_ROLL_S"
  kill -INT "$backend_pid" 2>/dev/null || true
  kill -INT -- "-$backend_pid" 2>/dev/null || true
  if [[ -n "$monitor_pid" ]]; then
    kill -INT "$monitor_pid" 2>/dev/null || true
    kill -INT -- "-$monitor_pid" 2>/dev/null || true
  fi
  if [[ -n "$scheduler_pid" ]]; then
    kill -INT "$scheduler_pid" 2>/dev/null || true
    kill -INT -- "-$scheduler_pid" 2>/dev/null || true
  fi
  if [[ -n "$replay_flow_fault_pid" ]]; then
    kill -INT "$replay_flow_fault_pid" 2>/dev/null || true
    kill -INT -- "-$replay_flow_fault_pid" 2>/dev/null || true
  fi
  if [[ -n "$replay_flow_trigger_pid" ]]; then
    kill "$replay_flow_trigger_pid" 2>/dev/null || true
  fi
  wait "$recorder_pid"
  wait "$timeline_pid"
  wait "$backend_pid" 2>/dev/null || true
  if [[ -n "$monitor_pid" ]]; then
    wait "$monitor_pid" 2>/dev/null || true
  fi
  if [[ -n "$scheduler_pid" ]]; then
    wait "$scheduler_pid" 2>/dev/null || true
  fi
  if [[ -n "$replay_flow_fault_pid" ]]; then
    wait "$replay_flow_fault_pid" 2>/dev/null || true
  fi
  if [[ -n "$replay_flow_trigger_pid" ]]; then
    wait "$replay_flow_trigger_pid" 2>/dev/null || true
  fi

  python3 "$SCRIPT_DIR/evaluate_lio_trajectory.py" \
    --estimate "$mode_dir/trajectory/estimate.tum" \
    --truth "$mode_dir/trajectory/ground_truth.tum" \
    --output "$mode_dir/trajectory_metrics.json"
}

for mode in $ABLATION_MODES; do
  if [[ "$mode" != "fixed" && "$mode" != "dynamic" ]]; then
    printf 'unsupported ablation mode: %s\n' "$mode" >&2
    exit 2
  fi
  run_mode "$mode"
done

if [[ -f "$OUTPUT_DIR/fixed/trajectory_metrics.json" \
      && -f "$OUTPUT_DIR/dynamic/trajectory_metrics.json" ]]; then
  python3 "$SCRIPT_DIR/summarize_backend_ablation.py" \
    --fixed "$OUTPUT_DIR/fixed/trajectory_metrics.json" \
    --dynamic "$OUTPUT_DIR/dynamic/trajectory_metrics.json" \
    --output "$OUTPUT_DIR/ablation_table.csv"
fi

printf 'Backend rosbag2 ablation: %s\n' "$OUTPUT_DIR"
