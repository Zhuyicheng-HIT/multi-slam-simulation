#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
RUN_ID=${RUN_ID:-$(date +%Y%m%d_%H%M%S)}
OUTPUT_DIR=${OUTPUT_DIR:-$REPO_ROOT/logs/uf_stage2_${RUN_ID}}
ANALYSIS_DURATION_S=${ANALYSIS_DURATION_S:-125}
ENABLE_LIO_ADAPTER=${ENABLE_LIO_ADAPTER:-1}
ENABLE_RELIABILITY=${ENABLE_RELIABILITY:-0}

mkdir -p "$OUTPUT_DIR"
set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

pids=()
cleanup() {
  set +e
  for pid in "${pids[@]:-}"; do
    kill -TERM "$pid" 2>/dev/null
    kill -TERM -- "-$pid" 2>/dev/null
  done
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic=$1
  local timeout_s=$2
  local started=$SECONDS
  until ros2 topic list 2>/dev/null | grep -Fxq "$topic"; do
    if (( SECONDS - started >= timeout_s )); then
      printf 'Timed out waiting for %s\n' "$topic" >&2
      return 1
    fi
    sleep 1
  done
}

wait_for_message() {
  local topic=$1
  local timeout_s=$2
  local started=$SECONDS
  until timeout 5s ros2 topic echo "$topic" --once >/dev/null 2>&1; do
    if (( SECONDS - started >= timeout_s )); then
      printf 'Timed out waiting for a message on %s\n' "$topic" >&2
      return 1
    fi
  done
}

printf 'Stage 2 output: %s\n' "$OUTPUT_DIR"

setsid env SHOW_FLOW_WINDOW=0 FLOW_DEBUG=false LOG_DIR="$OUTPUT_DIR/sim" \
  bash "$REPO_ROOT/tools/run_sim_with_flow.sh" \
  >"$OUTPUT_DIR/sim.stdout.log" 2>"$OUTPUT_DIR/sim.stderr.log" &
pids+=("$!")
wait_for_message /mavros/state 90
wait_for_message /mavros/imu/data_raw 90
wait_for_message /sim/mid360/points_raw 90

setsid ros2 launch uf_sensor_pipeline sensor_pipeline.launch.py \
  >"$OUTPUT_DIR/sensor_pipeline.stdout.log" 2>"$OUTPUT_DIR/sensor_pipeline.stderr.log" &
pids+=("$!")

setsid env RVIZ=0 LOG_DIR="$OUTPUT_DIR/lio" \
  bash "$REPO_ROOT/tools/run_fastlio_mapping.sh" \
  >"$OUTPUT_DIR/fastlio.stdout.log" 2>"$OUTPUT_DIR/fastlio.stderr.log" &
pids+=("$!")
wait_for_message /Odometry 90
wait_for_message /sensors/imu 30

estimate_topic=/Odometry
if [[ "$ENABLE_LIO_ADAPTER" == "1" ]]; then
  setsid ros2 launch uf_lio_adapter lio_adapter.launch.py \
    >"$OUTPUT_DIR/lio_adapter.stdout.log" 2>"$OUTPUT_DIR/lio_adapter.stderr.log" &
  pids+=("$!")
  wait_for_message /lio/diagnostics 45
  estimate_topic=/lio/odom
fi

score_recorder_pid=""
if [[ "$ENABLE_RELIABILITY" == "1" ]]; then
  setsid ros2 launch uf_reliability reliability.launch.py \
    >"$OUTPUT_DIR/reliability.stdout.log" 2>"$OUTPUT_DIR/reliability.stderr.log" &
  pids+=("$!")
  wait_for_message /reliability/imu_score 30
  python3 "$SCRIPT_DIR/record_reliability_scores.py" \
    --duration "$ANALYSIS_DURATION_S" \
    --output "$OUTPUT_DIR/reliability_scores.csv" \
    >"$OUTPUT_DIR/reliability_recorder.stdout.log" \
    2>"$OUTPUT_DIR/reliability_recorder.stderr.log" &
  score_recorder_pid=$!
  pids+=("$score_recorder_pid")
fi

python3 "$REPO_ROOT/tools/analyze_slam_drift.py" \
  --duration "$ANALYSIS_DURATION_S" --output "$OUTPUT_DIR/report.json" \
  >"$OUTPUT_DIR/analyzer.stdout.log" 2>"$OUTPUT_DIR/analyzer.stderr.log" &
analyzer_pid=$!
pids+=("$analyzer_pid")

python3 "$SCRIPT_DIR/record_lio_trajectory.py" \
  --duration "$ANALYSIS_DURATION_S" --output-dir "$OUTPUT_DIR/trajectory" \
  --estimate-topic "$estimate_topic" \
  >"$OUTPUT_DIR/trajectory_recorder.stdout.log" 2>"$OUTPUT_DIR/trajectory_recorder.stderr.log" &
recorder_pid=$!
pids+=("$recorder_pid")

set +e
env LOG_DIR="$OUTPUT_DIR/rectangle" ACCURACY_DURATION_S="$ANALYSIS_DURATION_S" \
  bash "$REPO_ROOT/tools/run_rectangle_state_machine.sh" \
  >"$OUTPUT_DIR/rectangle.stdout.log" 2>"$OUTPUT_DIR/rectangle.stderr.log"
rectangle_status=$?
wait "$analyzer_pid"
analyzer_status=$?
wait "$recorder_pid"
recorder_status=$?
score_status=0
if [[ -n "$score_recorder_pid" ]]; then
  wait "$score_recorder_pid"
  score_status=$?
fi
set -e

python3 "$SCRIPT_DIR/evaluate_lio_trajectory.py" \
  --estimate "$OUTPUT_DIR/trajectory/estimate.tum" \
  --truth "$OUTPUT_DIR/trajectory/ground_truth.tum" \
  --output "$OUTPUT_DIR/trajectory_metrics.json"

if [[ -n "$score_recorder_pid" ]]; then
  python3 "$SCRIPT_DIR/plot_reliability_timeline.py" \
    --input "$OUTPUT_DIR/reliability_scores.csv" \
    --output "$OUTPUT_DIR/reliability_timeline.png"
fi

printf 'rectangle_status=%s analyzer_status=%s recorder_status=%s score_status=%s\n' \
  "$rectangle_status" "$analyzer_status" "$recorder_status" "$score_status"
printf 'Stage 2 output: %s\n' "$OUTPUT_DIR"
(( rectangle_status == 0 && analyzer_status == 0 && recorder_status == 0 && score_status == 0 ))
