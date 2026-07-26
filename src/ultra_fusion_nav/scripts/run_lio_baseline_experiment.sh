#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
RUN_ID=${RUN_ID:-$(date +%Y%m%d_%H%M%S)}
OUTPUT_DIR=${OUTPUT_DIR:-$REPO_ROOT/logs/uf_stage2_${RUN_ID}}
ANALYSIS_DURATION_S=${ANALYSIS_DURATION_S:-125}
ENABLE_LIO_ADAPTER=${ENABLE_LIO_ADAPTER:-1}
ENABLE_UNIFIED_BACKEND=${ENABLE_UNIFIED_BACKEND:-0}
PRESERVE_LIO_ANCHOR=${PRESERVE_LIO_ANCHOR:-true}
ENABLE_RELIABILITY=${ENABLE_RELIABILITY:-0}
ENABLE_FLOW_CALIBRATION=${ENABLE_FLOW_CALIBRATION:-0}
FLOW_CALIBRATION_REQUIRE_PASS=${FLOW_CALIBRATION_REQUIRE_PASS:-0}
ENABLE_PERFORMANCE_MONITOR=${ENABLE_PERFORMANCE_MONITOR:-1}
ENABLE_RELIABILITY_TIMELINE=${ENABLE_RELIABILITY_TIMELINE:-0}
SIM_WORLD_NAME=${SIM_WORLD_NAME:-simple_apm_rgbd_mid360}
FASTLIO_INPUT_MODE=${FASTLIO_INPUT_MODE:-filtered_pointcloud}
ALLOW_MISSING_RELIABILITY=${ALLOW_MISSING_RELIABILITY:-0}
FAULT_MODALITY=${FAULT_MODALITY:-}
FAULT_TYPE=${FAULT_TYPE:-none}
FAULT_TRIGGER_DELAY_S=${FAULT_TRIGGER_DELAY_S:-0}
FAULT_DURATION_S=${FAULT_DURATION_S:-0}
FAULT_MAGNITUDE=${FAULT_MAGNITUDE:-0}
FAULT_SECONDARY_MAGNITUDE=${FAULT_SECONDARY_MAGNITUDE:-0}
FAULT_DELIVERY_MODE=${FAULT_DELIVERY_MODE:-runtime}

if [[ "$FAULT_DELIVERY_MODE" != "runtime" && "$FAULT_DELIVERY_MODE" != "startup" ]]; then
  printf 'FAULT_DELIVERY_MODE must be runtime or startup, got %s\n' \
    "$FAULT_DELIVERY_MODE" >&2
  exit 2
fi
if [[ "$PRESERVE_LIO_ANCHOR" != "true" && "$PRESERVE_LIO_ANCHOR" != "false" ]]; then
  printf 'PRESERVE_LIO_ANCHOR must be true or false, got %s\n' \
    "$PRESERVE_LIO_ANCHOR" >&2
  exit 2
fi

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

setsid env SHOW_FLOW_WINDOW=0 FLOW_DEBUG="${FLOW_DEBUG:-false}" LOG_DIR="$OUTPUT_DIR/sim" \
  bash "$REPO_ROOT/tools/run_sim_with_flow.sh" \
  >"$OUTPUT_DIR/sim.stdout.log" 2>"$OUTPUT_DIR/sim.stderr.log" &
pids+=("$!")
wait_for_message /mavros/state 90
wait_for_message /mavros/imu/data_raw 90
wait_for_message /sim/mid360/points_raw 90

fault_launch_env=()
if [[ -n "$FAULT_MODALITY" && "$FAULT_TYPE" != "none" \
      && "$FAULT_DELIVERY_MODE" == "startup" ]]; then
  fault_launch_env=(
    "UF_FAULT_MODALITY=$FAULT_MODALITY"
    "UF_FAULT_TYPE=$FAULT_TYPE"
    "UF_FAULT_START_S=$FAULT_TRIGGER_DELAY_S"
    "UF_FAULT_DURATION_S=$FAULT_DURATION_S"
    "UF_FAULT_MAGNITUDE=$FAULT_MAGNITUDE"
    "UF_FAULT_SECONDARY_MAGNITUDE=$FAULT_SECONDARY_MAGNITUDE"
  )
fi
setsid env "${fault_launch_env[@]}" ros2 launch uf_sensor_pipeline sensor_pipeline.launch.py \
  >"$OUTPUT_DIR/sensor_pipeline.stdout.log" 2>"$OUTPUT_DIR/sensor_pipeline.stderr.log" &
pids+=("$!")
if [[ "$FASTLIO_INPUT_MODE" == "filtered_pointcloud" ]]; then
  wait_for_message /sensors/lidar/points 30
fi

setsid env RVIZ=0 LOG_DIR="$OUTPUT_DIR/lio" FASTLIO_INPUT_MODE="$FASTLIO_INPUT_MODE" \
  bash "$REPO_ROOT/tools/run_fastlio_mapping.sh" \
  >"$OUTPUT_DIR/fastlio.stdout.log" 2>"$OUTPUT_DIR/fastlio.stderr.log" &
pids+=("$!")
wait_for_message /Odometry 90
wait_for_message /sensors/imu 30

performance_monitor_pid=""
if [[ "$ENABLE_PERFORMANCE_MONITOR" == "1" ]]; then
  ros2 run multi_slam_uav_sim simulation_performance_monitor --ros-args \
    -p world_name:="$SIM_WORLD_NAME" \
    -p output_path:="$OUTPUT_DIR/simulation_performance.json" \
    -p flow_truth_assistance:=false \
    -p minimum_external_nav_rate_hz:=0.0 \
    >"$OUTPUT_DIR/simulation_performance.stdout.log" \
    2>"$OUTPUT_DIR/simulation_performance.stderr.log" &
  performance_monitor_pid=$!
  pids+=("$performance_monitor_pid")
fi

estimate_topic=/Odometry
if [[ "$ENABLE_LIO_ADAPTER" == "1" ]]; then
  setsid ros2 launch uf_lio_adapter lio_adapter.launch.py \
    >"$OUTPUT_DIR/lio_adapter.stdout.log" 2>"$OUTPUT_DIR/lio_adapter.stderr.log" &
  pids+=("$!")
  wait_for_message /lio/diagnostics 45
  estimate_topic=/lio/odom
fi

unified_backend_pid=""
if [[ "$ENABLE_UNIFIED_BACKEND" == "1" ]]; then
  setsid ros2 launch uf_backend_fusion online_backend.launch.py \
    preserve_lio_anchor:="$PRESERVE_LIO_ANCHOR" \
    >"$OUTPUT_DIR/unified_backend.stdout.log" 2>"$OUTPUT_DIR/unified_backend.stderr.log" &
  unified_backend_pid=$!
  pids+=("$unified_backend_pid")
  wait_for_message /fusion/unified/odom 45
  estimate_topic=/fusion/unified/odom
fi

timeline_pid=""
if [[ "$ENABLE_RELIABILITY_TIMELINE" == "1" ]]; then
  timeline_args=(
    --duration "$ANALYSIS_DURATION_S"
    --output "$OUTPUT_DIR/reliability_timeline.json"
  )
  if [[ -n "$FAULT_MODALITY" && "$FAULT_TYPE" != "none" ]]; then
    timeline_args+=(
      --expect-fault-modality "$FAULT_MODALITY"
      --expect-fault-type "$FAULT_TYPE"
    )
  fi
  python3 "$SCRIPT_DIR/record_reliability_timeline.py" "${timeline_args[@]}" \
    >"$OUTPUT_DIR/reliability_timeline.stdout.log" \
    2>"$OUTPUT_DIR/reliability_timeline.stderr.log" &
  timeline_pid=$!
  pids+=("$timeline_pid")
fi

fault_trigger_pid=""
if [[ -n "$FAULT_MODALITY" && "$FAULT_TYPE" != "none" \
      && "$FAULT_DELIVERY_MODE" == "runtime" ]]; then
  case "$FAULT_MODALITY" in
    lidar|imu|gnss|optical_flow|depth|color) ;;
    *)
      printf 'Unsupported FAULT_MODALITY=%s\n' "$FAULT_MODALITY" >&2
      exit 2
      ;;
  esac
  fault_node="/fault_injector_${FAULT_MODALITY}"
  fault_magnitude_value=$(printf '%.9f' "$FAULT_MAGNITUDE")
  fault_secondary_value=$(printf '%.9f' "$FAULT_SECONDARY_MAGNITUDE")
  (
    printf 'fault_trigger_start modality=%s type=%s delay_s=%s duration_s=%s\n' \
      "$FAULT_MODALITY" "$FAULT_TYPE" "$FAULT_TRIGGER_DELAY_S" "$FAULT_DURATION_S"
    sleep "$FAULT_TRIGGER_DELAY_S"
    if awk "BEGIN {exit !($FAULT_MAGNITUDE != 0.0)}"; then
      timeout 60s ros2 param set "$fault_node" magnitude "$fault_magnitude_value"
    fi
    if awk "BEGIN {exit !($FAULT_SECONDARY_MAGNITUDE != 0.0)}"; then
      timeout 60s ros2 param set "$fault_node" secondary_magnitude "$fault_secondary_value"
    fi
    timeout 60s ros2 param set "$fault_node" fault_type "$FAULT_TYPE"
    printf 'fault_trigger_active node=%s\n' "$fault_node"
    if awk "BEGIN {exit !($FAULT_DURATION_S > 0.0)}"; then
      sleep "$FAULT_DURATION_S"
      timeout 60s ros2 param set "$fault_node" fault_type none
      printf 'fault_trigger_cleared node=%s\n' "$fault_node"
    fi
  ) >"$OUTPUT_DIR/fault_trigger.log" 2>&1 &
  fault_trigger_pid=$!
  pids+=("$fault_trigger_pid")
elif [[ -n "$FAULT_MODALITY" && "$FAULT_TYPE" != "none" ]]; then
  printf 'fault_trigger_scheduled modality=%s type=%s start_from_node_s=%s duration_s=%s\n' \
    "$FAULT_MODALITY" "$FAULT_TYPE" "$FAULT_TRIGGER_DELAY_S" "$FAULT_DURATION_S" \
    >"$OUTPUT_DIR/fault_trigger.log"
fi

score_recorder_pid=""
if [[ "$ENABLE_RELIABILITY" == "1" ]]; then
  if [[ "$ENABLE_UNIFIED_BACKEND" != "1" ]]; then
    setsid ros2 launch uf_reliability reliability.launch.py \
      >"$OUTPUT_DIR/reliability.stdout.log" 2>"$OUTPUT_DIR/reliability.stderr.log" &
    pids+=("$!")
  fi
  wait_for_message /reliability/imu_score 30
  score_args=(
    --duration "$ANALYSIS_DURATION_S"
    --output "$OUTPUT_DIR/reliability_scores.csv"
  )
  if [[ "$ALLOW_MISSING_RELIABILITY" == "1" ]]; then
    score_args+=(--allow-missing)
  fi
  python3 "$SCRIPT_DIR/record_reliability_scores.py" "${score_args[@]}" \
    >"$OUTPUT_DIR/reliability_recorder.stdout.log" \
    2>"$OUTPUT_DIR/reliability_recorder.stderr.log" &
  score_recorder_pid=$!
  pids+=("$score_recorder_pid")
fi

flow_calibration_pid=""
if [[ "$ENABLE_FLOW_CALIBRATION" == "1" ]]; then
  flow_args=(
    --duration "$ANALYSIS_DURATION_S"
    --output "$OUTPUT_DIR/optical_flow_lio_calibration.json"
    --csv "$OUTPUT_DIR/optical_flow_lio_pairs.csv"
  )
  python3 "$SCRIPT_DIR/calibrate_optical_flow_lio.py" "${flow_args[@]}" \
    >"$OUTPUT_DIR/optical_flow_lio_calibration.stdout.log" \
    2>"$OUTPUT_DIR/optical_flow_lio_calibration.stderr.log" &
  flow_calibration_pid=$!
  pids+=("$flow_calibration_pid")
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
flow_calibration_status=0
if [[ -n "$flow_calibration_pid" ]]; then
  wait "$flow_calibration_pid"
  flow_calibration_status=$?
fi
fault_status=0
if [[ -n "$fault_trigger_pid" ]]; then
  wait "$fault_trigger_pid"
  fault_status=$?
fi
timeline_status=0
if [[ -n "$timeline_pid" ]]; then
  wait "$timeline_pid"
  timeline_status=$?
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

flow_gate_status=0
if [[ -n "$flow_calibration_pid" ]]; then
  flow_gate_args=(
    --calibration "$OUTPUT_DIR/optical_flow_lio_calibration.json"
    --lio-report "$OUTPUT_DIR/report.json"
    --gazebo-log "$OUTPUT_DIR/rectangle/flow_gazebo_accuracy.log"
    --output "$OUTPUT_DIR/optical_flow_gate.json"
  )
  if [[ "$FLOW_CALIBRATION_REQUIRE_PASS" == "1" ]]; then
    flow_gate_args+=(--require-pass)
  fi
  set +e
  python3 "$SCRIPT_DIR/evaluate_optical_flow_gate.py" "${flow_gate_args[@]}"
  flow_gate_status=$?
  set -e
fi

printf 'rectangle_status=%s analyzer_status=%s recorder_status=%s score_status=%s flow_calibration_status=%s flow_gate_status=%s fault_status=%s timeline_status=%s\n' \
  "$rectangle_status" "$analyzer_status" "$recorder_status" "$score_status" \
  "$flow_calibration_status" "$flow_gate_status" "$fault_status" "$timeline_status"
printf 'Stage 2 output: %s\n' "$OUTPUT_DIR"
(( rectangle_status == 0 && analyzer_status == 0 && recorder_status == 0 \
   && score_status == 0 && flow_calibration_status == 0 && flow_gate_status == 0 \
   && fault_status == 0 \
   && timeline_status == 0 ))
