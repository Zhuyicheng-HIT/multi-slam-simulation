#!/usr/bin/env bash
set -Eeo pipefail

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=${WORKSPACE_ROOT:-$REPO_ROOT}
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
BAG_DIR=${BAG_DIR:?Set BAG_DIR to the frozen full-online rosbag directory}
OUTPUT_DIR=${OUTPUT_DIR:-$REPO_ROOT/logs/tmp/full_online_replay_$(date +%Y%m%d_%H%M%S)}
REPLAY_RATE=${REPLAY_RATE:-0.5}
REPLAY_START_OFFSET=${REPLAY_START_OFFSET:-0.0}
REPLAY_READ_AHEAD_QUEUE_SIZE=${REPLAY_READ_AHEAD_QUEUE_SIZE:-5000}
REPLAY_DISCOVERY_DELAY_S=${REPLAY_DISCOVERY_DELAY_S:-3.0}
REPLAY_ACK_TIMEOUT_MS=${REPLAY_ACK_TIMEOUT_MS:-10000}
REPLAY_WALL_TIMEOUT_S=${REPLAY_WALL_TIMEOUT_S:-1800}
POST_REPLAY_DRAIN_WALL_S=${POST_REPLAY_DRAIN_WALL_S:-15}
ENABLE_CYCLE_TRACE=${ENABLE_CYCLE_TRACE:-1}
BACKEND_CPUSET=${BACKEND_CPUSET:-}
BACKEND_NUMERIC_THREADS=${BACKEND_NUMERIC_THREADS:-1}
BACKEND_EXECUTOR_THREADS=${BACKEND_EXECUTOR_THREADS:-2}
NONLINEAR_MAX_ITERATIONS=${NONLINEAR_MAX_ITERATIONS:-2}
NONLINEAR_INITIALIZATION_MAX_ITERATIONS=${NONLINEAR_INITIALIZATION_MAX_ITERATIONS:-4}
NONLINEAR_RECOVERY_MAX_ITERATIONS=${NONLINEAR_RECOVERY_MAX_ITERATIONS:-4}
NONLINEAR_REINTEGRATION_MAX_ITERATIONS=${NONLINEAR_REINTEGRATION_MAX_ITERATIONS:-1}
NATIVE_LIDAR_QOS_DEPTH=${NATIVE_LIDAR_QOS_DEPTH:-auto}
NATIVE_WORKER_QUEUE_SIZE=${NATIVE_WORKER_QUEUE_SIZE:-auto}
FRONTEND_SCAN_PREDICTION_ENABLED=${FRONTEND_SCAN_PREDICTION_ENABLED:-auto}
CPP_MATH_CORE_ENABLED=${CPP_MATH_CORE_ENABLED:-true}
VISUAL_FACTOR_MODE=${VISUAL_FACTOR_MODE:-paper_reprojection}
VISUAL_PENDING_ENABLED=${VISUAL_PENDING_ENABLED:-true}
VISUAL_REQUIRE_TIME_LOCK=${VISUAL_REQUIRE_TIME_LOCK:-false}
RGBD_DEPTH_HEALTHY_LIDAR_STRIDE=${RGBD_DEPTH_HEALTHY_LIDAR_STRIDE:-1}
AXIS_INFORMATION_HANDOFF_ENABLED=${AXIS_INFORMATION_HANDOFF_ENABLED:-false}
Z_GAUGE_ENABLED=${Z_GAUGE_ENABLED:-false}
Z_GAUGE_TARGET_HISTORY_SIZE=${Z_GAUGE_TARGET_HISTORY_SIZE:-1}
Z_GAUGE_UPDATE_TIME_CONSTANT_S=${Z_GAUGE_UPDATE_TIME_CONSTANT_S:-0.60}
Z_GAUGE_MAXIMUM_CORRECTION_RATE_MPS=${Z_GAUGE_MAXIMUM_CORRECTION_RATE_MPS:-1.0}
Z_GAUGE_MAXIMUM_CORRECTION_STEP_M=${Z_GAUGE_MAXIMUM_CORRECTION_STEP_M:-0.30}
BACKEND_RELIABILITY_MODE=${BACKEND_RELIABILITY_MODE:-dynamic}
FIXED_LIDAR_WEIGHT=${FIXED_LIDAR_WEIGHT:-1.0}
FIXED_GNSS_WEIGHT=${FIXED_GNSS_WEIGHT:-1.0}
FIXED_IMU_WEIGHT=${FIXED_IMU_WEIGHT:-1.0}
FIXED_OPTICAL_FLOW_WEIGHT=${FIXED_OPTICAL_FLOW_WEIGHT:-1.0}
FIXED_VISION_WEIGHT=${FIXED_VISION_WEIGHT:-1.0}
LIDAR_PREDICTION_GATE_MAX_POSITION_M=${LIDAR_PREDICTION_GATE_MAX_POSITION_M:-1.0}
LIDAR_PREDICTION_GATE_MAX_YAW_RAD=${LIDAR_PREDICTION_GATE_MAX_YAW_RAD:-0.50}
LIDAR_PREDICTION_GATE_RECOVERY_AFTER=${LIDAR_PREDICTION_GATE_RECOVERY_AFTER:-3}
LIDAR_PREDICTION_RECOVERY_WEIGHT=${LIDAR_PREDICTION_RECOVERY_WEIGHT:-0.20}
LIDAR_PREDICTION_RECOVERY_INFLATION=${LIDAR_PREDICTION_RECOVERY_INFLATION:-5.0}
CALIBRATION_APPLY_LOCKED_TIME_OFFSET=${CALIBRATION_APPLY_LOCKED_TIME_OFFSET:-false}
CALIBRATION_APPLY_LOCKED_ROTATION=${CALIBRATION_APPLY_LOCKED_ROTATION:-false}
ACCURACY_ENABLED=${ACCURACY_ENABLED:-1}
REPLAY_EXTERNAL_NAV_GATE_ENABLED=${REPLAY_EXTERNAL_NAV_GATE_ENABLED:-false}
REPLAY_EXTERNAL_NAV_METRICS_DURATION_S=${REPLAY_EXTERNAL_NAV_METRICS_DURATION_S:-120}
MISSING_VISION_FACTOR_SCORE_POLICY=${MISSING_VISION_FACTOR_SCORE_POLICY:-error}
REGENERATE_VISION_FACTOR_SCORE=${REGENERATE_VISION_FACTOR_SCORE:-auto}
REPLAY_VISION_FACTOR_SCORE_TOPIC=${REPLAY_VISION_FACTOR_SCORE_TOPIC:-/replay/reliability/vision_factor_score}
REPLAY_RGBD_MAX_DEPTH_M=${REPLAY_RGBD_MAX_DEPTH_M:-10.0}
REPLAY_REQUIRE_RGBD_GEOMETRY=${REPLAY_REQUIRE_RGBD_GEOMETRY:-auto}
STRICT_REPLAY_ACCEPTANCE=${STRICT_REPLAY_ACCEPTANCE:-1}
REPLAY_REQUIRE_TIME_CALIBRATION_LOCK=${REPLAY_REQUIRE_TIME_CALIBRATION_LOCK:-false}
REPLAY_REQUIRE_TIME_CALIBRATION_APPLIED=${REPLAY_REQUIRE_TIME_CALIBRATION_APPLIED:-false}
REPLAY_ALLOW_AUXILIARY_KEYFRAMES=${REPLAY_ALLOW_AUXILIARY_KEYFRAMES:-false}
REPLAY_EXPECTED_MINIMUM_COMMITTED_COUNT=${REPLAY_EXPECTED_MINIMUM_COMMITTED_COUNT:-auto}
REPLAY_MAXIMUM_UNCOMMITTED_NATIVE_COUNT=${REPLAY_MAXIMUM_UNCOMMITTED_NATIVE_COUNT:-auto}
REPLAY_ACCURACY_POLICY=${REPLAY_ACCURACY_POLICY:-strict}
REPLAY_QOS_OVERRIDES=${REPLAY_QOS_OVERRIDES:-$REPO_ROOT/tools/config/full_online_replay_qos.yaml}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-67}
export ROS_DOMAIN_ID

if [[ "$CPP_MATH_CORE_ENABLED" != true && "$CPP_MATH_CORE_ENABLED" != false ]]; then
  printf 'CPP_MATH_CORE_ENABLED must be true or false.\n' >&2
  exit 2
fi
if [[ "$AXIS_INFORMATION_HANDOFF_ENABLED" != true && \
      "$AXIS_INFORMATION_HANDOFF_ENABLED" != false ]]; then
  printf 'AXIS_INFORMATION_HANDOFF_ENABLED must be true or false.\n' >&2
  exit 2
fi
if [[ "$Z_GAUGE_ENABLED" != true && "$Z_GAUGE_ENABLED" != false ]]; then
  printf 'Z_GAUGE_ENABLED must be true or false.\n' >&2
  exit 2
fi
if ! [[ "$Z_GAUGE_TARGET_HISTORY_SIZE" =~ ^[1-9][0-9]*$ ]]; then
  printf 'Z_GAUGE_TARGET_HISTORY_SIZE must be a positive integer.\n' >&2
  exit 2
fi
for value in \
  "$Z_GAUGE_UPDATE_TIME_CONSTANT_S" \
  "$Z_GAUGE_MAXIMUM_CORRECTION_RATE_MPS" \
  "$Z_GAUGE_MAXIMUM_CORRECTION_STEP_M"; do
  if ! python3 -c 'import sys; raise SystemExit(not float(sys.argv[1]) > 0.0)' \
      "$value"; then
    printf 'Z gauge tuning values must be positive.\n' >&2
    exit 2
  fi
done
if [[ "$VISUAL_REQUIRE_TIME_LOCK" != true && \
      "$VISUAL_REQUIRE_TIME_LOCK" != false ]]; then
  printf 'VISUAL_REQUIRE_TIME_LOCK must be true or false.\n' >&2
  exit 2
fi
if [[ "$BACKEND_RELIABILITY_MODE" != dynamic && \
      "$BACKEND_RELIABILITY_MODE" != fixed ]]; then
  printf 'BACKEND_RELIABILITY_MODE must be dynamic or fixed.\n' >&2
  exit 2
fi
for value in \
  "$FIXED_LIDAR_WEIGHT" "$FIXED_GNSS_WEIGHT" "$FIXED_IMU_WEIGHT" \
  "$FIXED_OPTICAL_FLOW_WEIGHT" "$FIXED_VISION_WEIGHT"; do
  if ! python3 -c 'import sys; value=float(sys.argv[1]); raise SystemExit(not 0.0 <= value <= 1.0)' \
      "$value"; then
    printf 'Fixed factor weights must be finite values in [0, 1].\n' >&2
    exit 2
  fi
done
if ! python3 -c \
    'import math,sys; values=[float(v) for v in sys.argv[1:]]; raise SystemExit(not all(math.isfinite(v) and v>0 for v in values))' \
    "$LIDAR_PREDICTION_GATE_MAX_POSITION_M" \
    "$LIDAR_PREDICTION_GATE_MAX_YAW_RAD" \
    "$LIDAR_PREDICTION_RECOVERY_WEIGHT" \
    "$LIDAR_PREDICTION_RECOVERY_INFLATION"; then
  printf 'LiDAR prediction gate and recovery values must be positive and finite.\n' >&2
  exit 2
fi
if ! [[ "$LIDAR_PREDICTION_GATE_RECOVERY_AFTER" =~ ^[1-9][0-9]*$ ]]; then
  printf 'LIDAR_PREDICTION_GATE_RECOVERY_AFTER must be a positive integer.\n' >&2
  exit 2
fi
if ! [[ "$RGBD_DEPTH_HEALTHY_LIDAR_STRIDE" =~ ^[1-9][0-9]*$ ]]; then
  printf 'RGBD_DEPTH_HEALTHY_LIDAR_STRIDE must be a positive integer.\n' >&2
  exit 2
fi
for value in \
  "$CALIBRATION_APPLY_LOCKED_TIME_OFFSET" \
  "$CALIBRATION_APPLY_LOCKED_ROTATION" \
  "$REPLAY_REQUIRE_TIME_CALIBRATION_LOCK" \
  "$REPLAY_REQUIRE_TIME_CALIBRATION_APPLIED" \
  "$REPLAY_ALLOW_AUXILIARY_KEYFRAMES" \
  "$REPLAY_EXTERNAL_NAV_GATE_ENABLED"; do
  if [[ "$value" != true && "$value" != false ]]; then
    printf 'Calibration apply switches must be true or false.\n' >&2
    exit 2
  fi
done
case "$REPLAY_ACCURACY_POLICY" in
  strict|rmse) ;;
  *)
    printf 'REPLAY_ACCURACY_POLICY must be strict or rmse.\n' >&2
    exit 2
    ;;
esac
case "$MISSING_VISION_FACTOR_SCORE_POLICY" in
  error|disable_visual) ;;
  *)
    printf 'MISSING_VISION_FACTOR_SCORE_POLICY must be error or disable_visual.\n' >&2
    exit 2
    ;;
esac
case "$REGENERATE_VISION_FACTOR_SCORE" in
  auto|0|1) ;;
  *) printf 'REGENERATE_VISION_FACTOR_SCORE must be auto, 0, or 1.\n' >&2; exit 2 ;;
esac
case "$REPLAY_REQUIRE_RGBD_GEOMETRY" in
  auto|true|false) ;;
  *)
    printf 'REPLAY_REQUIRE_RGBD_GEOMETRY must be auto, true, or false.\n' >&2
    exit 2
    ;;
esac
case "$STRICT_REPLAY_ACCEPTANCE" in
  0|1) ;;
  *) printf 'STRICT_REPLAY_ACCEPTANCE must be 0 or 1.\n' >&2; exit 2 ;;
esac
case "$FRONTEND_SCAN_PREDICTION_ENABLED" in
  auto|true|false) ;;
  *)
    printf 'FRONTEND_SCAN_PREDICTION_ENABLED must be auto, true, or false.\n' >&2
    exit 2
    ;;
esac
if [[ ! -f "$REPLAY_QOS_OVERRIDES" ]]; then
  printf 'Missing rosbag QoS override file: %s\n' "$REPLAY_QOS_OVERRIDES" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"
source "$LIDAR_WS/install/setup.bash"
mkdir -p "$OUTPUT_DIR"

requested_visual_factor_mode=$VISUAL_FACTOR_MODE
contract_args=(
  python3 "$REPO_ROOT/tools/verify_estimator_input_bag.py"
  --bag "$BAG_DIR"
  --output "$OUTPUT_DIR/bag_contract.json"
)
if [[ "$VISUAL_FACTOR_MODE" == paper_reprojection ]]; then
  contract_args+=(--require-visual)
fi
require_rgbd_geometry=$REPLAY_REQUIRE_RGBD_GEOMETRY
if [[ "$require_rgbd_geometry" == auto ]]; then
  if [[ "$VISUAL_FACTOR_MODE" == paper_reprojection ]]; then
    require_rgbd_geometry=true
  else
    require_rgbd_geometry=false
  fi
fi
if [[ "$require_rgbd_geometry" == true ]]; then
  contract_args+=(--require-rgbd-geometry)
fi
set +e
"${contract_args[@]}" >"$OUTPUT_DIR/bag_contract.log" 2>&1
bag_contract_status=$?
set -e
if [[ "$bag_contract_status" != 0 ]]; then
  cat "$OUTPUT_DIR/bag_contract.log" >&2
  printf 'Frozen bag does not satisfy the selected replay contract.\n' >&2
  exit 2
fi
mapfile -t bag_counts < <(python3 - "$OUTPUT_DIR/bag_contract.json" <<'PY'
import json
import sys

report = json.load(open(sys.argv[1], encoding="utf-8"))
print(int(report.get("native_factor_count", 0)))
print(int(report.get("visual_factor_score_count", 0)))
print(int(report.get("frontend_scan_request_count", 0)))
print(int(report.get("reference_unified_odom_count", 0)))
PY
)
expected_native_factor_count=${bag_counts[0]:-0}
visual_factor_score_count=${bag_counts[1]:-0}
expected_scan_request_count=${bag_counts[2]:-0}
expected_committed_count=${bag_counts[3]:-0}
if [[ "$REPLAY_EXPECTED_MINIMUM_COMMITTED_COUNT" != auto ]]; then
  expected_committed_count=$REPLAY_EXPECTED_MINIMUM_COMMITTED_COUNT
fi
if [[ "$FRONTEND_SCAN_PREDICTION_ENABLED" == auto ]]; then
  if (( expected_scan_request_count > 0 )); then
    FRONTEND_SCAN_PREDICTION_ENABLED=true
  else
    FRONTEND_SCAN_PREDICTION_ENABLED=false
  fi
fi
if (( expected_native_factor_count <= 0 )); then
  printf 'Frozen bag has no native LiDAR factors.\n' >&2
  exit 2
fi
if (( expected_committed_count <= 0 )); then
  printf 'Frozen bag has no reference unified odometry.\n' >&2
  exit 2
fi
visual_factor_score_status=not_required
regenerate_visual_factor_score=0
if [[ "$VISUAL_FACTOR_MODE" == paper_reprojection ]]; then
  if [[ "$REGENERATE_VISION_FACTOR_SCORE" == 1 ]] || \
     [[ "$REGENERATE_VISION_FACTOR_SCORE" == auto && \
        "$visual_factor_score_count" == 0 ]]; then
    regenerate_visual_factor_score=1
    visual_factor_score_status=regenerated_from_tracks
  fi
  if (( visual_factor_score_count <= 0 )); then
    if (( regenerate_visual_factor_score == 1 )); then
      :
    elif [[ "$MISSING_VISION_FACTOR_SCORE_POLICY" == error ]]; then
      printf '%s\n' \
        'Bag lacks /reliability/vision_factor_score; set MISSING_VISION_FACTOR_SCORE_POLICY=disable_visual for an explicitly non-visual replay.' >&2
      exit 2
    else
      VISUAL_FACTOR_MODE=disabled
      VISUAL_PENDING_ENABLED=false
      visual_factor_score_status=disabled_missing_factor_score
      printf '%s\n' \
        'WARNING: visual factor replay is explicitly disabled because the bag lacks /reliability/vision_factor_score.' >&2
    fi
  else
    if (( regenerate_visual_factor_score == 0 )); then
      visual_factor_score_status=available
    fi
  fi
fi
backend_visual_factor_score_topic=/reliability/vision_factor_score
if (( regenerate_visual_factor_score == 1 )); then
  backend_visual_factor_score_topic=$REPLAY_VISION_FACTOR_SCORE_TOPIC
fi
if [[ "$NATIVE_LIDAR_QOS_DEPTH" == auto ]]; then
  NATIVE_LIDAR_QOS_DEPTH=$((expected_native_factor_count + 16))
fi
if [[ "$NATIVE_WORKER_QUEUE_SIZE" == auto ]]; then
  NATIVE_WORKER_QUEUE_SIZE=$((expected_native_factor_count + 16))
fi
if ! [[ "$NATIVE_LIDAR_QOS_DEPTH" =~ ^[1-9][0-9]*$ ]] || \
   ! [[ "$NATIVE_WORKER_QUEUE_SIZE" =~ ^[1-9][0-9]*$ ]]; then
  printf 'Replay native QoS and worker queue depths must be positive integers or auto.\n' >&2
  exit 2
fi
if [[ "$STRICT_REPLAY_ACCEPTANCE" == 1 ]] && \
   ! python3 -c 'import sys; raise SystemExit(abs(float(sys.argv[1])) > 1e-12)' \
      "$REPLAY_START_OFFSET"; then
  printf 'Strict replay acceptance requires REPLAY_START_OFFSET=0.\n' >&2
  exit 2
fi

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
  -p cpp_math_core_enabled:="$CPP_MATH_CORE_ENABLED"
  -p cpp_math_core_required:="$CPP_MATH_CORE_ENABLED"
  -p visual_factor_mode:="$VISUAL_FACTOR_MODE"
  -p visual_pending_enabled:="$VISUAL_PENDING_ENABLED"
  -p visual_initialization_require_time_lock:="$VISUAL_REQUIRE_TIME_LOCK"
  -p visual_factor_score_topic:="$backend_visual_factor_score_topic"
  -p rgbd_depth_healthy_lidar_stride:="$RGBD_DEPTH_HEALTHY_LIDAR_STRIDE"
  -p axis_information_handoff_enabled:="$AXIS_INFORMATION_HANDOFF_ENABLED"
  -p z_gauge_enabled:="$Z_GAUGE_ENABLED"
  -p z_gauge_target_history_size:="$Z_GAUGE_TARGET_HISTORY_SIZE"
  -p z_gauge_update_time_constant_s:="$Z_GAUGE_UPDATE_TIME_CONSTANT_S"
  -p z_gauge_maximum_correction_rate_mps:="$Z_GAUGE_MAXIMUM_CORRECTION_RATE_MPS"
  -p z_gauge_maximum_correction_step_m:="$Z_GAUGE_MAXIMUM_CORRECTION_STEP_M"
  -p reliability_mode:="$BACKEND_RELIABILITY_MODE"
  -p fixed_lidar_weight:="$FIXED_LIDAR_WEIGHT"
  -p fixed_gnss_weight:="$FIXED_GNSS_WEIGHT"
  -p fixed_imu_weight:="$FIXED_IMU_WEIGHT"
  -p fixed_optical_flow_weight:="$FIXED_OPTICAL_FLOW_WEIGHT"
  -p fixed_vision_weight:="$FIXED_VISION_WEIGHT"
  -p lidar_prediction_gate_max_position_m:="$LIDAR_PREDICTION_GATE_MAX_POSITION_M"
  -p lidar_prediction_gate_max_yaw_rad:="$LIDAR_PREDICTION_GATE_MAX_YAW_RAD"
  -p lidar_prediction_gate_recovery_after_rejections:="$LIDAR_PREDICTION_GATE_RECOVERY_AFTER"
  -p lidar_prediction_recovery_weight:="$LIDAR_PREDICTION_RECOVERY_WEIGHT"
  -p lidar_prediction_recovery_inflation:="$LIDAR_PREDICTION_RECOVERY_INFLATION"
  -p calibration_apply_locked_time_offset:="$CALIBRATION_APPLY_LOCKED_TIME_OFFSET"
  -p calibration_apply_locked_rotation:="$CALIBRATION_APPLY_LOCKED_ROTATION"
  -p native_lidar_factor_enabled:=true
  -p input_trigger_mode:=native_factor
  -p frontend_scan_prediction_enabled:="$FRONTEND_SCAN_PREDICTION_ENABLED"
  -p allow_lio_pose_fallback:=false
  -p imu_factor_enabled:=true
  -p executor_threads:="$BACKEND_EXECUTOR_THREADS"
  -p nonlinear_max_iterations:="$NONLINEAR_MAX_ITERATIONS"
  -p nonlinear_initialization_max_iterations:="$NONLINEAR_INITIALIZATION_MAX_ITERATIONS"
  -p nonlinear_recovery_max_iterations:="$NONLINEAR_RECOVERY_MAX_ITERATIONS"
  -p nonlinear_reintegration_max_iterations:="$NONLINEAR_REINTEGRATION_MAX_ITERATIONS"
  -p native_lidar_qos_depth:="$NATIVE_LIDAR_QOS_DEPTH"
  -p native_worker_queue_size:="$NATIVE_WORKER_QUEUE_SIZE"
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

setsid env OMP_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  OPENBLAS_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  MKL_NUM_THREADS="$BACKEND_NUMERIC_THREADS" \
  NUMEXPR_NUM_THREADS="$BACKEND_NUMERIC_THREADS" "${backend_command[@]}" \
  >"$OUTPUT_DIR/backend.log" 2>&1 &
pids+=("$!")

if [[ "$REPLAY_EXTERNAL_NAV_GATE_ENABLED" == true ]]; then
  setsid ros2 run uf_sensor_pipeline external_nav_gate --ros-args \
    -p use_sim_time:=true \
    -p input_topic:=/fusion/unified/odom \
    -p output_topic:=/fusion/replay_external_nav \
    -p expected_map_frame:=camera_init \
    -p expected_body_frame:=body \
    -p maximum_input_age_s:=0.65 \
    -p maximum_propagation_age_s:=0.65 \
    -p output_rate_hz:=20.0 \
    -p require_scheduler_health:=false \
    -p require_capability_support:=false \
    -p inflate_covariance_from_estimator_support:=false \
    -p stop_on_excessive_covariance:=false \
    >"$OUTPUT_DIR/external_nav_gate.log" 2>&1 &
  pids+=("$!")
  setsid ros2 run multi_slam_uav_sim external_nav_accuracy --ros-args \
    -p use_sim_time:=true \
    -p odom_topic:=/fusion/replay_external_nav \
    -p truth_odom_topic:=/sim/mid360/ground_truth_odom \
    -p output_path:="$OUTPUT_DIR/external_nav_gate_accuracy.json" \
    -p initial_alignment_duration_s:=10.0 \
    >"$OUTPUT_DIR/external_nav_gate_accuracy.log" 2>&1 &
  pids+=("$!")
  setsid python3 "$REPO_ROOT/tools/unified_runtime_metrics.py" \
    --duration "$REPLAY_EXTERNAL_NAV_METRICS_DURATION_S" \
    --output "$OUTPUT_DIR/external_nav_gate_metrics.json" \
    --wall-timeout "$REPLAY_WALL_TIMEOUT_S" \
    --ros-args -p use_sim_time:=true \
    -p external_nav_topic:=/fusion/replay_external_nav \
    >"$OUTPUT_DIR/external_nav_gate_metrics.log" 2>&1 &
  pids+=("$!")
fi

if (( regenerate_visual_factor_score == 1 )); then
  setsid ros2 run uf_reliability reliability_monitor --ros-args \
    --params-file "$WORKSPACE_ROOT/install/uf_reliability/share/uf_reliability/config/reliability.yaml" \
    -p use_sim_time:=true \
    -p vision.factor_score_topic:="$REPLAY_VISION_FACTOR_SCORE_TOPIC" \
    -p vision.maximum_depth_m:="$REPLAY_RGBD_MAX_DEPTH_M" \
    -r /reliability/lidar_score:=/replay/reliability/lidar_score \
    -r /reliability/lidar_map_score:=/replay/reliability/lidar_map_score \
    -r /reliability/gnss_score:=/replay/reliability/gnss_score \
    -r /reliability/imu_score:=/replay/reliability/imu_score \
    -r /reliability/optical_flow_score:=/replay/reliability/optical_flow_score \
    -r /reliability/vision_score:=/replay/reliability/vision_score \
    -r /reliability/gnss_integrity:=/replay/reliability/gnss_integrity \
    >"$OUTPUT_DIR/vision_factor_score_regenerator.log" 2>&1 &
  pids+=("$!")
fi

setsid python3 "$REPO_ROOT/tools/record_backend_replay_metrics.py" \
  --output "$OUTPUT_DIR/replay_metrics.json" --wall-timeout 1800 \
  >"$OUTPUT_DIR/metrics_recorder.log" 2>&1 &
recorder_pid=$!
pids+=("$recorder_pid")

if [[ "$ACCURACY_ENABLED" == 1 ]]; then
  setsid ros2 run multi_slam_uav_sim external_nav_accuracy --ros-args \
    -p use_sim_time:=true \
    -p odom_topic:=/fusion/unified/odom \
    -p truth_odom_topic:=/sim/mid360/ground_truth_odom \
    -p output_path:="$OUTPUT_DIR/external_nav_accuracy.json" \
    -p initial_alignment_duration_s:=10.0 \
    >"$OUTPUT_DIR/external_nav_accuracy.log" 2>&1 &
  pids+=("$!")
fi
sleep 4

play_command=(
  ros2 bag play "$BAG_DIR"
  --rate "$REPLAY_RATE"
  --start-offset "$REPLAY_START_OFFSET"
  --read-ahead-queue-size "$REPLAY_READ_AHEAD_QUEUE_SIZE"
  --delay "$REPLAY_DISCOVERY_DELAY_S"
  --wait-for-all-acked "$REPLAY_ACK_TIMEOUT_MS"
  --disable-keyboard-controls
  --qos-profile-overrides-path "$REPLAY_QOS_OVERRIDES"
  --topics
  /clock
  /fast_lio/frontend_scan_request
  /fast_lio/native_lidar_factor
  /sensors/imu
  /sensors/gnss/fix
  /sensors/gnss/raw
  /mavros/imu/static_pressure
  /sensors/optical_flow/rad
  /vision/feature_tracks
  /vision/rgbd_geometry_tracks
  /reliability/scheduler_state
  /reliability/lidar_score
  /reliability/imu_score
  /reliability/gnss_score
  /reliability/optical_flow_score
  /reliability/vision_score
  /reliability/vision_factor_score
  /calibration/lidar_relative_motion
  /sim/mid360/ground_truth_odom
  /mission/phase
  /mission/checkpoint
)
set +e
timeout "${REPLAY_WALL_TIMEOUT_S}s" "${play_command[@]}" \
  </dev/null >"$OUTPUT_DIR/rosbag_play.log" 2>&1
play_status=$?
set -e
sleep "$POST_REPLAY_DRAIN_WALL_S"
kill -INT "$recorder_pid" 2>/dev/null || true
for _ in {1..30}; do
  [[ -s "$OUTPUT_DIR/replay_metrics.json" ]] && break
  sleep 0.2
done
cleanup
trap - EXIT INT TERM

acceptance_status=0
if [[ "$STRICT_REPLAY_ACCEPTANCE" == 1 ]]; then
  acceptance_args=(
    python3 "$REPO_ROOT/tools/check_backend_replay_result.py"
    --backend-log "$OUTPUT_DIR/backend.log"
    --expected-native-count "$expected_native_factor_count"
    --expected-scan-request-count "$expected_scan_request_count"
    --expected-committed-count "$expected_committed_count"
    --accuracy-policy "$REPLAY_ACCURACY_POLICY"
    --metrics-json "$OUTPUT_DIR/replay_metrics.json"
    --output "$OUTPUT_DIR/replay_acceptance.json"
  )
  if [[ "$REPLAY_ALLOW_AUXILIARY_KEYFRAMES" == true ]]; then
    acceptance_args+=(--allow-auxiliary-keyframes)
  fi
  if [[ "$REPLAY_MAXIMUM_UNCOMMITTED_NATIVE_COUNT" != auto ]]; then
    acceptance_args+=(
      --maximum-uncommitted-native-count
      "$REPLAY_MAXIMUM_UNCOMMITTED_NATIVE_COUNT"
    )
  fi
  if [[ "$ACCURACY_ENABLED" == 1 ]]; then
    acceptance_args+=(--accuracy-json "$OUTPUT_DIR/external_nav_accuracy.json")
  fi
  if [[ "$REPLAY_REQUIRE_TIME_CALIBRATION_LOCK" == true ]]; then
    acceptance_args+=(--require-time-calibration-lock)
  fi
  if [[ "$REPLAY_REQUIRE_TIME_CALIBRATION_APPLIED" == true ]]; then
    acceptance_args+=(--require-time-calibration-applied)
  fi
  set +e
  "${acceptance_args[@]}" \
    >"$OUTPUT_DIR/replay_acceptance.log" 2>&1
  acceptance_status=$?
  set -e
fi

printf 'play_status=%s\nworkspace=%s\nbag=%s\nrate=%s\nstart_offset=%s\ncpuset=%s\n' \
  "$play_status" "$WORKSPACE_ROOT" "$BAG_DIR" "$REPLAY_RATE" \
  "$REPLAY_START_OFFSET" "${BACKEND_CPUSET:-normal}" \
  >"$OUTPUT_DIR/replay_result.env"
printf 'numeric_threads=%s\ncpp_math_core_enabled=%s\nvisual_factor_mode_requested=%s\nvisual_factor_mode=%s\nvisual_pending=%s\nvisual_require_time_lock=%s\nreliability_mode=%s\n' \
  "$BACKEND_NUMERIC_THREADS" "$CPP_MATH_CORE_ENABLED" \
  "$requested_visual_factor_mode" "$VISUAL_FACTOR_MODE" \
  "$VISUAL_PENDING_ENABLED" \
  "$VISUAL_REQUIRE_TIME_LOCK" "$BACKEND_RELIABILITY_MODE" \
  >>"$OUTPUT_DIR/replay_result.env"
printf 'executor_threads=%s\nnative_lidar_qos_depth=%s\nnative_worker_queue_size=%s\n' \
  "$BACKEND_EXECUTOR_THREADS" "$NATIVE_LIDAR_QOS_DEPTH" \
  "$NATIVE_WORKER_QUEUE_SIZE" >>"$OUTPUT_DIR/replay_result.env"
printf 'qos_overrides=%s\n' "$REPLAY_QOS_OVERRIDES" \
  >>"$OUTPUT_DIR/replay_result.env"
printf 'read_ahead_queue_size=%s\ndiscovery_delay_s=%s\nack_timeout_ms=%s\npost_replay_drain_wall_s=%s\n' \
  "$REPLAY_READ_AHEAD_QUEUE_SIZE" "$REPLAY_DISCOVERY_DELAY_S" \
  "$REPLAY_ACK_TIMEOUT_MS" "$POST_REPLAY_DRAIN_WALL_S" \
  >>"$OUTPUT_DIR/replay_result.env"
printf 'expected_native_factor_count=%s\nstrict_replay_acceptance=%s\nacceptance_status=%s\n' \
  "$expected_native_factor_count" "$STRICT_REPLAY_ACCEPTANCE" \
  "$acceptance_status" >>"$OUTPUT_DIR/replay_result.env"
printf 'expected_frontend_scan_request_count=%s\n' \
  "$expected_scan_request_count" >>"$OUTPUT_DIR/replay_result.env"
printf 'frontend_scan_prediction_enabled=%s\n' \
  "$FRONTEND_SCAN_PREDICTION_ENABLED" >>"$OUTPUT_DIR/replay_result.env"
printf 'fixed_lidar_weight=%s\nfixed_gnss_weight=%s\nfixed_imu_weight=%s\nfixed_optical_flow_weight=%s\nfixed_vision_weight=%s\n' \
  "$FIXED_LIDAR_WEIGHT" "$FIXED_GNSS_WEIGHT" "$FIXED_IMU_WEIGHT" \
  "$FIXED_OPTICAL_FLOW_WEIGHT" "$FIXED_VISION_WEIGHT" \
  >>"$OUTPUT_DIR/replay_result.env"
printf 'rgbd_depth_healthy_lidar_stride=%s\n' \
  "$RGBD_DEPTH_HEALTHY_LIDAR_STRIDE" >>"$OUTPUT_DIR/replay_result.env"
printf 'axis_information_handoff_enabled=%s\n' \
  "$AXIS_INFORMATION_HANDOFF_ENABLED" >>"$OUTPUT_DIR/replay_result.env"
printf 'require_rgbd_geometry=%s\n' "$require_rgbd_geometry" \
  >>"$OUTPUT_DIR/replay_result.env"
printf 'calibration_apply_locked_time_offset=%s\ncalibration_apply_locked_rotation=%s\n' \
  "$CALIBRATION_APPLY_LOCKED_TIME_OFFSET" \
  "$CALIBRATION_APPLY_LOCKED_ROTATION" >>"$OUTPUT_DIR/replay_result.env"
printf 'require_time_calibration_lock=%s\nrequire_time_calibration_applied=%s\n' \
  "$REPLAY_REQUIRE_TIME_CALIBRATION_LOCK" \
  "$REPLAY_REQUIRE_TIME_CALIBRATION_APPLIED" >>"$OUTPUT_DIR/replay_result.env"
printf 'missing_vision_factor_score_policy=%s\nvisual_factor_score_count=%s\nvisual_factor_score_status=%s\n' \
  "$MISSING_VISION_FACTOR_SCORE_POLICY" "$visual_factor_score_count" \
  "$visual_factor_score_status" >>"$OUTPUT_DIR/replay_result.env"
printf 'regenerate_visual_factor_score=%s\nbackend_visual_factor_score_topic=%s\nreplay_rgbd_max_depth_m=%s\n' \
  "$regenerate_visual_factor_score" "$backend_visual_factor_score_topic" \
  "$REPLAY_RGBD_MAX_DEPTH_M" >>"$OUTPUT_DIR/replay_result.env"
if [[ ! -s "$OUTPUT_DIR/replay_metrics.json" ]]; then
  printf 'Replay metrics were not written.\n' >&2
  exit 3
fi
cat "$OUTPUT_DIR/replay_metrics.json"
if [[ "$STRICT_REPLAY_ACCEPTANCE" == 1 ]]; then
  cat "$OUTPUT_DIR/replay_acceptance.json"
fi
if [[ "$play_status" != 0 ]]; then
  exit "$play_status"
fi
exit "$acceptance_status"
