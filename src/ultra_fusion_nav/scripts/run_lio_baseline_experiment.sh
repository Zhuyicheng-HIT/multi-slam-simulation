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
ENABLE_NATIVE_FACTOR_VALIDATOR=${ENABLE_NATIVE_FACTOR_VALIDATOR:-0}
ENABLE_ROSBAG_RECORDING=${ENABLE_ROSBAG_RECORDING:-0}
NUMPY_NUM_THREADS=${NUMPY_NUM_THREADS:-1}
SIM_WORLD_NAME=${SIM_WORLD_NAME:-simple_apm_rgbd_mid360}
FASTLIO_INPUT_MODE=${FASTLIO_INPUT_MODE:-filtered_pointcloud}
ENABLE_VISION_PIPELINE=${ENABLE_VISION_PIPELINE:-${ENABLE_D435_BRIDGE:-1}}
case "$ENABLE_VISION_PIPELINE" in
  1|true) ENABLE_VISION_PIPELINE=true ;;
  0|false) ENABLE_VISION_PIPELINE=false ;;
  *)
    printf 'ENABLE_VISION_PIPELINE must be true/false or 1/0, got %s\n' \
      "$ENABLE_VISION_PIPELINE" >&2
    exit 2
    ;;
esac
if [[ -z "${ALLOW_MISSING_RELIABILITY+x}" ]]; then
  if [[ "$ENABLE_VISION_PIPELINE" == "false" ]]; then
    ALLOW_MISSING_RELIABILITY=1
  else
    ALLOW_MISSING_RELIABILITY=0
  fi
fi
FAULT_MODALITY=${FAULT_MODALITY:-}
FAULT_TYPE=${FAULT_TYPE:-none}
FAULT_TRIGGER_DELAY_S=${FAULT_TRIGGER_DELAY_S:-0}
FAULT_DURATION_S=${FAULT_DURATION_S:-0}
FAULT_MAGNITUDE=${FAULT_MAGNITUDE:-0}
FAULT_SECONDARY_MAGNITUDE=${FAULT_SECONDARY_MAGNITUDE:-0}
FAULT_DELIVERY_MODE=${FAULT_DELIVERY_MODE:-runtime}
FLOW_USE_PHYSICS=${FLOW_USE_PHYSICS:-false}
FLOW_RESTAMP_OUTPUT=${FLOW_RESTAMP_OUTPUT:-true}

if [[ "$FLOW_USE_PHYSICS" != "false" && "$FLOW_USE_PHYSICS" != "0" ]]; then
  printf '%s\n' \
    'run_lio_baseline_experiment.sh rejects Gazebo-pose synthesized flow.' \
    'Set FLOW_USE_PHYSICS=false for algorithm-quality evaluation.' >&2
  exit 2
fi
if [[ "$FLOW_RESTAMP_OUTPUT" != "true" && "$FLOW_RESTAMP_OUTPUT" != "1" ]]; then
  printf '%s\n' \
    'The current non-use_sim_time stack requires FLOW_RESTAMP_OUTPUT=true.' \
    'integration_time_us still preserves the source exposure interval.' >&2
  exit 2
fi

if [[ "$ENABLE_UNIFIED_BACKEND" == "1" \
      && -z "${FASTLIO_NATIVE_FACTOR_EXPORT:-}" ]]; then
  # A unified run must exercise the native factor path by default. Set this to
  # 0 explicitly only for the documented pose-fallback ablation.
  FASTLIO_NATIVE_FACTOR_EXPORT=1
  export FASTLIO_NATIVE_FACTOR_EXPORT
fi

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
if [[ "$ENABLE_VISION_PIPELINE" == "false" \
      && "$FAULT_TYPE" != "none" \
      && ("$FAULT_MODALITY" == "depth" || "$FAULT_MODALITY" == "color") ]]; then
  printf 'Vision fault injection requires ENABLE_VISION_PIPELINE=true\n' >&2
  exit 2
fi

mkdir -p "$OUTPUT_DIR"
profile_name=full_sensor
if [[ "$ENABLE_VISION_PIPELINE" == "false" ]]; then
  profile_name=four_source
fi
{
  printf 'profile=%s\n' "$profile_name"
  printf 'active_modalities=lidar,imu,gnss,optical_flow\n'
  printf 'vision_pipeline=%s\n' "$ENABLE_VISION_PIPELINE"
  printf 'fault_modality=%s\n' "${FAULT_MODALITY:-none}"
  printf 'fault_type=%s\n' "$FAULT_TYPE"
  printf 'flow_use_physics=%s\n' "$FLOW_USE_PHYSICS"
  printf 'flow_restamp_output=%s\n' "$FLOW_RESTAMP_OUTPUT"
} >"$OUTPUT_DIR/experiment_profile.txt"
export OPENBLAS_NUM_THREADS="$NUMPY_NUM_THREADS"
export MKL_NUM_THREADS="$NUMPY_NUM_THREADS"
export NUMEXPR_NUM_THREADS="$NUMPY_NUM_THREADS"
set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
if [[ -f "$HOME/multi-slam-deps/mid360_ws/install/setup.bash" ]]; then
  # Keep the patched FAST-LIO message overlay on top so the backend can import
  # NativeLidarFactor at runtime while still seeing this workspace's packages.
  source "$HOME/multi-slam-deps/mid360_ws/install/setup.bash"
fi
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

setsid env SHOW_FLOW_WINDOW=0 FLOW_DEBUG="${FLOW_DEBUG:-false}" \
  FLOW_USE_PHYSICS=false FLOW_RESTAMP_OUTPUT="$FLOW_RESTAMP_OUTPUT" \
  LOG_DIR="$OUTPUT_DIR/sim" \
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
  enable_vision:="$ENABLE_VISION_PIPELINE" \
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

native_factor_validator_pid=""
if [[ "$ENABLE_NATIVE_FACTOR_VALIDATOR" == "1" ]]; then
  if [[ "${FASTLIO_NATIVE_FACTOR_EXPORT:-0}" != "1" \
        && "${FASTLIO_NATIVE_FACTOR_EXPORT:-0}" != "true" ]]; then
    printf 'ENABLE_NATIVE_FACTOR_VALIDATOR=1 requires FASTLIO_NATIVE_FACTOR_EXPORT=1\n' >&2
    exit 2
  fi
  setsid ros2 run uf_lio_adapter native_factor_validator --ros-args \
    --params-file "$REPO_ROOT/install/uf_lio_adapter/share/uf_lio_adapter/config/native_factor_validator.yaml" \
    -p output_path:="$OUTPUT_DIR/native_factor_metrics.jsonl" \
    -p summary_path:="$OUTPUT_DIR/native_factor_summary.json" \
    >"$OUTPUT_DIR/native_factor_validator.stdout.log" \
    2>"$OUTPUT_DIR/native_factor_validator.stderr.log" &
  native_factor_validator_pid=$!
  pids+=("$native_factor_validator_pid")
  wait_for_message /fast_lio/native_lidar_factor 30
fi

performance_monitor_pid=""
if [[ "$ENABLE_PERFORMANCE_MONITOR" == "1" ]]; then
  performance_fusion_topic=/fusion/gps_flow/odom
  performance_fusion_diagnostic_topic=/fusion/gps_flow/diagnostics
  if [[ "$ENABLE_UNIFIED_BACKEND" == "1" ]]; then
    performance_fusion_topic=/fusion/unified/odom
    performance_fusion_diagnostic_topic=/fusion/unified/diagnostics
  fi
  ros2 run multi_slam_uav_sim simulation_performance_monitor --ros-args \
    -p world_name:="$SIM_WORLD_NAME" \
    -p output_path:="$OUTPUT_DIR/simulation_performance.json" \
    -p fusion_topic:="$performance_fusion_topic" \
    -p fusion_diagnostic_topic:="$performance_fusion_diagnostic_topic" \
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

rosbag_pid=""
if [[ "$ENABLE_ROSBAG_RECORDING" == "1" ]]; then
  rosbag_dir="$OUTPUT_DIR/rosbag_inputs"
  setsid ros2 bag record --storage sqlite3 --output "$rosbag_dir" \
    /lio/odom \
    /lio/diagnostics \
    /fast_lio/native_lidar_factor \
    /sensors/gnss/fix \
    /mavros/gpsstatus/gps1/raw \
    /sensors/imu \
    /sensors/optical_flow/rad \
    /fault/state \
    /reliability/scheduler_state \
    /reliability/lidar_score \
    /reliability/gnss_score \
    /reliability/imu_score \
    /reliability/optical_flow_score \
    /sim/mid360/ground_truth_odom \
    >"$OUTPUT_DIR/rosbag_record.stdout.log" \
    2>"$OUTPUT_DIR/rosbag_record.stderr.log" &
  rosbag_pid=$!
  pids+=("$rosbag_pid")
  sleep 2
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

rosbag_status=0
if [[ -n "$rosbag_pid" ]]; then
  kill -INT "$rosbag_pid" 2>/dev/null || true
  kill -INT -- "-$rosbag_pid" 2>/dev/null || true
  set +e
  timeout 20s tail --pid="$rosbag_pid" -f /dev/null
  rosbag_wait_status=$?
  set -e
  if [[ "$rosbag_wait_status" != "0" \
        || ! -f "$OUTPUT_DIR/rosbag_inputs/metadata.yaml" ]]; then
    printf 'rosbag2 did not finalize cleanly in %s\n' \
      "$OUTPUT_DIR/rosbag_inputs" >&2
    rosbag_status=1
  fi
fi

if [[ -n "$native_factor_validator_pid" ]]; then
  kill -INT "$native_factor_validator_pid" 2>/dev/null || true
  kill -INT -- "-$native_factor_validator_pid" 2>/dev/null || true
  set +e
  timeout 15s tail --pid="$native_factor_validator_pid" -f /dev/null
  set -e
fi

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

printf 'rectangle_status=%s analyzer_status=%s recorder_status=%s score_status=%s flow_calibration_status=%s flow_gate_status=%s fault_status=%s timeline_status=%s rosbag_status=%s\n' \
  "$rectangle_status" "$analyzer_status" "$recorder_status" "$score_status" \
  "$flow_calibration_status" "$flow_gate_status" "$fault_status" "$timeline_status" \
  "$rosbag_status"
printf 'Stage 2 output: %s\n' "$OUTPUT_DIR"
(( rectangle_status == 0 && analyzer_status == 0 && recorder_status == 0 \
   && score_status == 0 && flow_calibration_status == 0 && flow_gate_status == 0 \
   && fault_status == 0 \
   && timeline_status == 0 && rosbag_status == 0 ))
