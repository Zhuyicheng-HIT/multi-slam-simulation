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
# The stable simulator owns the protocol-compatible /livox streams directly.
# Keep filtered PointCloud2 as an explicit debug mode; making it the default
# deadlocked startup while waiting for the retired /sim/.../points_raw topic.
FASTLIO_INPUT_MODE=${FASTLIO_INPUT_MODE:-livox}
ENABLE_VISION_PIPELINE=${ENABLE_VISION_PIPELINE:-${ENABLE_D435_BRIDGE:-1}}
case "$ENABLE_VISION_PIPELINE" in
  1|true) ENABLE_VISION_PIPELINE=true; ENABLE_D435_BRIDGE_VALUE=1 ;;
  0|false) ENABLE_VISION_PIPELINE=false; ENABLE_D435_BRIDGE_VALUE=0 ;;
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
FLOW_RESTAMP_OUTPUT=${FLOW_RESTAMP_OUTPUT:-false}
FLOW_TRANSPORT=${FLOW_TRANSPORT:-direct}
ENABLE_FLOW_ROUTE_VALIDATION=${ENABLE_FLOW_ROUTE_VALIDATION:-1}
FLOW_ROUTE_REQUIRE_PASS=${FLOW_ROUTE_REQUIRE_PASS:-1}

case "$FLOW_TRANSPORT" in
  direct)
    enable_fcu_flow_router=0
    enable_fcu_observation_bridge=false
    optical_flow_input_topic=/sim/optical_flow/rad
    ;;
  fcu_router)
    enable_fcu_flow_router=1
    enable_fcu_observation_bridge=true
    optical_flow_input_topic=/fcu/optical_flow/rad
    ;;
  *)
    printf 'FLOW_TRANSPORT must be direct or fcu_router, got %s\n' \
      "$FLOW_TRANSPORT" >&2
    exit 2
    ;;
esac

if [[ "$FLOW_USE_PHYSICS" != "false" && "$FLOW_USE_PHYSICS" != "0" ]]; then
  printf '%s\n' \
    'run_lio_baseline_experiment.sh rejects Gazebo-pose synthesized flow.' \
    'Set FLOW_USE_PHYSICS=false for algorithm-quality evaluation.' >&2
  exit 2
fi
if [[ "$FLOW_RESTAMP_OUTPUT" != "false" && "$FLOW_RESTAMP_OUTPUT" != "0" ]]; then
  printf '%s\n' \
    'ROS simulation time requires FLOW_RESTAMP_OUTPUT=false.' \
    'Keep source acquisition stamps and /clock in one time domain.' >&2
  exit 2
fi

if [[ "$ENABLE_UNIFIED_BACKEND" == "1" \
      && -z "${FASTLIO_NATIVE_FACTOR_EXPORT:-}" ]]; then
  # A unified run must exercise the native factor path by default. Set this to
  # 0 explicitly only for the documented pose-fallback ablation.
  FASTLIO_NATIVE_FACTOR_EXPORT=1
  export FASTLIO_NATIVE_FACTOR_EXPORT
fi
if [[ "$ENABLE_UNIFIED_BACKEND" == "1" \
      && ("${FASTLIO_NATIVE_FACTOR_EXPORT:-0}" == "1" \
          || "${FASTLIO_NATIVE_FACTOR_EXPORT:-0}" == "true") ]]; then
  # Native-factor Stage3 is a two-sided contract. After the bootstrap factor,
  # the backend owns scan prediction and FAST-LIO inserts only states confirmed
  # by that backend. Enabling only factor export yields exactly one state and
  # then a permanent scan-prediction cache miss.
  FASTLIO_DOWNSTREAM_BACKEND=${FASTLIO_DOWNSTREAM_BACKEND:-1}
  FASTLIO_MAP_INSERTION_MODE=${FASTLIO_MAP_INSERTION_MODE:-backend_confirmed}
  FASTLIO_BACKEND_TRAJECTORY_FRONTEND=${FASTLIO_BACKEND_TRAJECTORY_FRONTEND:-1}
  export FASTLIO_DOWNSTREAM_BACKEND
  export FASTLIO_MAP_INSERTION_MODE
  export FASTLIO_BACKEND_TRAJECTORY_FRONTEND
  # Diagnostic /Odometry is intentionally off when the unified backend owns
  # the trajectory. A pose adapter would reintroduce the forbidden proxy path.
  ENABLE_LIO_ADAPTER=0
  NATIVE_UNIFIED_MODE=1
else
  NATIVE_UNIFIED_MODE=0
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
  printf 'flow_transport=%s\n' "$FLOW_TRANSPORT"
  if [[ "$FLOW_TRANSPORT" == "fcu_router" ]]; then
    printf 'flow_wire_protocol=MAVLink1\n'
    printf 'flow_message_ids=OPTICAL_FLOW:100,DISTANCE_SENSOR:132\n'
    printf 'flow_route=SERIAL1->ArduPilot->SERIAL0->MAVROS_raw_source\n'
  fi
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

wait_for_message_from() {
  local topic=$1 timeout_s=$2 producer_pid=$3
  local started=$SECONDS
  until timeout 5s ros2 topic echo "$topic" --once >/dev/null 2>&1; do
    if ! kill -0 "$producer_pid" 2>/dev/null; then
      printf 'Producer pid %s exited while waiting for %s\n' \
        "$producer_pid" "$topic" >&2
      return 1
    fi
    if (( SECONDS - started >= timeout_s )); then
      printf 'Timed out waiting for a message on %s\n' "$topic" >&2
      return 1
    fi
  done
}

printf 'Stage 2 output: %s\n' "$OUTPUT_DIR"

setsid env SHOW_FLOW_WINDOW=0 FLOW_DEBUG="${FLOW_DEBUG:-false}" \
  USE_SIM_TIME=true MTF_RESTAMP_OUTPUT=false \
  ENABLE_D435_BRIDGE="$ENABLE_D435_BRIDGE_VALUE" \
  FLOW_USE_PHYSICS=false FLOW_RESTAMP_OUTPUT="$FLOW_RESTAMP_OUTPUT" \
  ENABLE_FCU_FLOW_ROUTER="$enable_fcu_flow_router" \
  LOG_DIR="$OUTPUT_DIR/sim" \
  bash "$REPO_ROOT/tools/run_sim_with_flow.sh" \
  >"$OUTPUT_DIR/sim.stdout.log" 2>"$OUTPUT_DIR/sim.stderr.log" &
pids+=("$!")
wait_for_message /mavros/state 90
wait_for_message /livox/imu 90
case "$FASTLIO_INPUT_MODE" in
  livox)
    wait_for_message /livox/lidar 90
    ;;
  pointcloud)
    wait_for_message /sim/mid360/points_raw 90
    ;;
  filtered_pointcloud)
    # The sensor pipeline below owns body filtering. Its source must exist
    # before the filtered output can become ready.
    wait_for_message /sim/mid360/points_raw 90
    ;;
  *)
    printf 'Unsupported FASTLIO_INPUT_MODE=%s\n' "$FASTLIO_INPUT_MODE" >&2
    exit 2
    ;;
esac
if [[ "$FLOW_TRANSPORT" == "fcu_router" ]]; then
  wait_for_message /fcu/mavlink/optical_flow 45
  wait_for_message /fcu/mavlink/optical_flow_rad 45
  wait_for_message /fcu/mavlink/range 45
fi

fault_launch_env=()
if [[ -n "$FAULT_MODALITY" && "$FAULT_TYPE" != "none" ]]; then
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
  use_sim_time:=true \
  enable_vision:="$ENABLE_VISION_PIPELINE" \
  enable_fcu_observation_bridge:="$enable_fcu_observation_bridge" \
  optical_flow_input_topic:="$optical_flow_input_topic" \
  fcu_flow_input_topic:=/fcu/mavlink/optical_flow \
  fcu_flow_rad_input_topic:=/fcu/mavlink/optical_flow_rad \
  fcu_range_input_topic:=/fcu/mavlink/range \
  >"$OUTPUT_DIR/sensor_pipeline.stdout.log" 2>"$OUTPUT_DIR/sensor_pipeline.stderr.log" &
pids+=("$!")
wait_for_message /sensors/optical_flow/rad 30
if [[ "$FASTLIO_INPUT_MODE" == "filtered_pointcloud" ]]; then
  wait_for_message /sensors/lidar/points 30
fi

flow_route_validation_pid=""
if [[ "$FLOW_TRANSPORT" == "fcu_router" \
      && "$ENABLE_FLOW_ROUTE_VALIDATION" == "1" ]]; then
  route_args=(
    --duration "$ANALYSIS_DURATION_S"
    --output "$OUTPUT_DIR/fcu_flow_roundtrip.json"
  )
  if [[ "$FLOW_ROUTE_REQUIRE_PASS" == "1" ]]; then
    route_args+=(--require-pass)
  fi
  python3 "$SCRIPT_DIR/evaluate_fcu_flow_roundtrip.py" "${route_args[@]}" \
    >"$OUTPUT_DIR/fcu_flow_roundtrip.stdout.log" \
    2>"$OUTPUT_DIR/fcu_flow_roundtrip.stderr.log" &
  flow_route_validation_pid=$!
  pids+=("$flow_route_validation_pid")
fi

fastlio_bridge_mode=${START_LIVOX_POINTCLOUD_BRIDGE:-auto}
if [[ "$FASTLIO_INPUT_MODE" == "livox" \
      && -z "${START_LIVOX_POINTCLOUD_BRIDGE+x}" ]]; then
  # The simulator already owns /livox/lidar and /livox/imu. Discovery-based
  # auto mode races DDS startup and can create a duplicate bridge.
  fastlio_bridge_mode=0
fi
setsid env RVIZ=0 USE_SIM_TIME=true LOG_DIR="$OUTPUT_DIR/lio" FASTLIO_INPUT_MODE="$FASTLIO_INPUT_MODE" \
  START_LIVOX_POINTCLOUD_BRIDGE="$fastlio_bridge_mode" \
  bash "$REPO_ROOT/tools/run_fastlio_mapping.sh" \
  >"$OUTPUT_DIR/fastlio.stdout.log" 2>"$OUTPUT_DIR/fastlio.stderr.log" &
fastlio_pid=$!
pids+=("$fastlio_pid")
if [[ "$NATIVE_UNIFIED_MODE" == "1" ]]; then
  wait_for_message_from /fast_lio/native_lidar_factor 90 "$fastlio_pid"
else
  wait_for_message_from /Odometry 90 "$fastlio_pid"
fi
wait_for_message /sensors/imu 30

native_factor_validator_pid=""
if [[ "$ENABLE_NATIVE_FACTOR_VALIDATOR" == "1" ]]; then
  if [[ "${FASTLIO_NATIVE_FACTOR_EXPORT:-0}" != "1" \
        && "${FASTLIO_NATIVE_FACTOR_EXPORT:-0}" != "true" ]]; then
    printf 'ENABLE_NATIVE_FACTOR_VALIDATOR=1 requires FASTLIO_NATIVE_FACTOR_EXPORT=1\n' >&2
    exit 2
  fi
  setsid ros2 run uf_lio_adapter native_factor_validator --ros-args \
    -p use_sim_time:=true \
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
    -p use_sim_time:=true \
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
    use_sim_time:=true \
    >"$OUTPUT_DIR/lio_adapter.stdout.log" 2>"$OUTPUT_DIR/lio_adapter.stderr.log" &
  pids+=("$!")
  wait_for_message /lio/diagnostics 45
  estimate_topic=/lio/odom
fi

unified_backend_pid=""
if [[ "$ENABLE_UNIFIED_BACKEND" == "1" ]]; then
  setsid ros2 launch uf_backend_fusion online_backend.launch.py \
    use_sim_time:=true \
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
if [[ -n "$FAULT_MODALITY" && "$FAULT_TYPE" != "none" ]]; then
  case "$FAULT_MODALITY" in
    lidar|imu|gnss|optical_flow|depth|color) ;;
    *)
      printf 'Unsupported FAULT_MODALITY=%s\n' "$FAULT_MODALITY" >&2
      exit 2
      ;;
  esac
  printf 'fault_trigger_source_time modality=%s type=%s start_from_first_sample_s=%s duration_s=%s\n' \
    "$FAULT_MODALITY" "$FAULT_TYPE" "$FAULT_TRIGGER_DELAY_S" "$FAULT_DURATION_S" \
    >"$OUTPUT_DIR/fault_trigger.log"
fi

score_recorder_pid=""
if [[ "$ENABLE_RELIABILITY" == "1" ]]; then
  if [[ "$ENABLE_UNIFIED_BACKEND" != "1" ]]; then
    setsid ros2 launch uf_reliability reliability.launch.py \
      use_sim_time:=true \
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
  --ros-args -p use_sim_time:=true \
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
  MAVLINK_TAKEOFF_URL=tcp:127.0.0.1:5763 \
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
flow_route_validation_status=0
if [[ -n "$flow_route_validation_pid" ]]; then
  wait "$flow_route_validation_pid"
  flow_route_validation_status=$?
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

printf 'rectangle_status=%s analyzer_status=%s recorder_status=%s score_status=%s flow_calibration_status=%s flow_gate_status=%s flow_route_validation_status=%s fault_status=%s timeline_status=%s rosbag_status=%s\n' \
  "$rectangle_status" "$analyzer_status" "$recorder_status" "$score_status" \
  "$flow_calibration_status" "$flow_gate_status" "$flow_route_validation_status" \
  "$fault_status" "$timeline_status" \
  "$rosbag_status"
printf 'Stage 2 output: %s\n' "$OUTPUT_DIR"
(( rectangle_status == 0 && analyzer_status == 0 && recorder_status == 0 \
   && score_status == 0 && flow_calibration_status == 0 && flow_gate_status == 0 \
   && flow_route_validation_status == 0 \
   && fault_status == 0 \
   && timeline_status == 0 && rosbag_status == 0 ))
