#!/usr/bin/env bash
set -Eeo pipefail

export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
RUN_ID=${RUN_ID:-full_online_backend_capture_$(date +%Y%m%d_%H%M%S)}
RUN_DIR=${RUN_DIR:-$REPO_ROOT/logs/tmp/$RUN_ID}
BAG_DIR=${BAG_DIR:-$RUN_DIR/full_online_backend_replay}
# Child launchers change working directories while starting SITL and Gazebo.
# Canonical paths keep PID files, logs, and lifecycle ownership anchored to the
# selected run even when callers supply a repository-relative output path.
RUN_DIR=$(realpath -m "$RUN_DIR")
BAG_DIR=$(realpath -m "$BAG_DIR")
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
PR6_START_RTABMAP=${PR6_START_RTABMAP:-0}
VISUAL_BRIDGE_ENABLED=${VISUAL_BRIDGE_ENABLED:-1}
VISUAL_FRONTEND_ENABLED=${VISUAL_FRONTEND_ENABLED:-1}
VISUAL_FACTOR_MODE=${VISUAL_FACTOR_MODE:-paper_reprojection}
VISUAL_KEYFRAME_PROFILE=${VISUAL_KEYFRAME_PROFILE:-balanced}
VISUAL_CANDIDATE_QUALITY_ENABLED=${VISUAL_CANDIDATE_QUALITY_ENABLED:-1}
VISUAL_PENDING_ENABLED=${VISUAL_PENDING_ENABLED:-1}
VISUAL_REQUIRE_TIME_LOCK=${VISUAL_REQUIRE_TIME_LOCK:-0}
ONLINE_MAPPING_MODE=${ONLINE_MAPPING_MODE:-joint}
MISSION_MODE=${MISSION_MODE:-rectangle}
MISSION_START_WAIT_S=${MISSION_START_WAIT_S:-300}
MISSION_WALL_TIMEOUT_S=${MISSION_WALL_TIMEOUT_S:-1200}
BACKEND_CPUSET=${BACKEND_CPUSET:-}
BACKEND_NUMERIC_THREADS=${BACKEND_NUMERIC_THREADS:-1}
REQUIRE_VISUAL_BAG=${REQUIRE_VISUAL_BAG:-auto}
ACCURACY_ENABLED=${ACCURACY_ENABLED:-1}
case "$MISSION_MODE" in
  rectangle|s_curve) ;;
  *) printf 'MISSION_MODE must be rectangle or s_curve.\n' >&2; exit 2 ;;
esac
case "$REQUIRE_VISUAL_BAG" in
  auto)
    if [[ "$VISUAL_FRONTEND_ENABLED" == 1 && \
          "$VISUAL_FACTOR_MODE" == paper_reprojection ]]; then
      REQUIRE_VISUAL_BAG=1
    else
      REQUIRE_VISUAL_BAG=0
    fi
    ;;
  0|1) ;;
  *) printf 'REQUIRE_VISUAL_BAG must be auto, 0, or 1.\n' >&2; exit 2 ;;
esac

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
if [[ -f "$LIDAR_WS/install/setup.bash" ]]; then
  source "$LIDAR_WS/install/setup.bash"
fi
mkdir -p "$RUN_DIR"

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

wait_for_mission_gate() {
  local headless_pid=$1 timeout_s=$2 started=$SECONDS
  local helper="$REPO_ROOT/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/wait_for_ros_message.py"
  while (( SECONDS - started < timeout_s )); do
    if ! kill -0 "$headless_pid" 2>/dev/null; then
      printf 'Headless stack exited before ExternalNav became ready.\n' >&2
      return 1
    fi
    if python3 "$helper" --topic /fusion/runtime_external_nav \
        --timeout 5 --reliability best_effort >/dev/null 2>&1; then
      return 0
    fi
  done
  printf 'Timed out waiting for /fusion/runtime_external_nav.\n' >&2
  return 1
}

# Store estimator inputs, the scan handshake, and compact timing diagnostics.
# Camera images, maps, and simulation truth remain excluded. The bag stays
# under logs/tmp and must never be committed.
setsid ros2 bag record --storage sqlite3 --output "$BAG_DIR" \
  /clock \
  /fast_lio/frontend_scan_request \
  /fast_lio/native_lidar_factor \
  /sensors/imu \
  /sensors/gnss/fix \
  /sensors/gnss/raw \
  /mavros/imu/static_pressure \
  /sensors/optical_flow/rad \
  /vision/feature_tracks \
  /vision/rgbd_geometry_tracks \
  /reliability/scheduler_state \
  /reliability/lidar_score \
  /reliability/imu_score \
  /reliability/gnss_score \
  /reliability/optical_flow_score \
  /reliability/vision_score \
  /reliability/vision_factor_score \
  /calibration/lidar_relative_motion \
  /external_nav/diagnostics \
  /vision/frontend_diagnostics \
  /fusion/unified/visual_timing \
  /fusion/unified/diagnostics \
  /fusion/unified/odom \
  /sim/mid360/ground_truth_odom \
  /mission/phase \
  /mission/checkpoint \
  >"$RUN_DIR/rosbag_record.log" 2>&1 &
pids+=("$!")
sleep 3

if [[ "$ACCURACY_ENABLED" == 1 ]]; then
  setsid ros2 run multi_slam_uav_sim external_nav_accuracy --ros-args \
    -p use_sim_time:=true \
    -p odom_topic:=/fusion/unified/odom \
    -p truth_odom_topic:=/sim/mid360/ground_truth_odom \
    -p output_path:="$RUN_DIR/external_nav_accuracy.json" \
    -p initial_alignment_duration_s:=10.0 \
    >"$RUN_DIR/external_nav_accuracy.log" 2>&1 &
  pids+=("$!")
fi

if [[ "$MISSION_MODE" == rectangle ]]; then
  run_small_rectangle=1
  exit_after_rectangle=1
  expect_external_visual_motion=0
else
  run_small_rectangle=0
  exit_after_rectangle=0
  expect_external_visual_motion=1
fi
headless_command=(
  env
  RUN_ID="$RUN_ID"
  RUN_DIR="$RUN_DIR/online"
  RUN_SMALL_RECTANGLE="$run_small_rectangle"
  EXIT_AFTER_RECTANGLE="$exit_after_rectangle"
  EXPECT_EXTERNAL_VISUAL_MOTION="$expect_external_visual_motion"
  PR6_START_RTABMAP="$PR6_START_RTABMAP"
  VISUAL_BRIDGE_ENABLED="$VISUAL_BRIDGE_ENABLED"
  VISUAL_FRONTEND_ENABLED="$VISUAL_FRONTEND_ENABLED"
  VISUAL_FACTOR_MODE="$VISUAL_FACTOR_MODE"
  VISUAL_KEYFRAME_PROFILE="$VISUAL_KEYFRAME_PROFILE"
  VISUAL_CANDIDATE_QUALITY_ENABLED="$VISUAL_CANDIDATE_QUALITY_ENABLED"
  VISUAL_PENDING_ENABLED="$VISUAL_PENDING_ENABLED"
  VISUAL_REQUIRE_TIME_LOCK="$VISUAL_REQUIRE_TIME_LOCK"
  PERFORMANCE_PROFILING_ENABLED=1
  ONLINE_MAPPING_MODE="$ONLINE_MAPPING_MODE"
  BACKEND_CPUSET="$BACKEND_CPUSET"
  BACKEND_NUMERIC_THREADS="$BACKEND_NUMERIC_THREADS"
  EVIDENCE_ROS_DURATION_S="${EVIDENCE_ROS_DURATION_S:-240}"
  EVIDENCE_WALL_TIMEOUT_S="${EVIDENCE_WALL_TIMEOUT_S:-1200}"
  TRAJECTORY_ROS_DURATION_S="${TRAJECTORY_ROS_DURATION_S:-240}"
  TRAJECTORY_WALL_TIMEOUT_S="${TRAJECTORY_WALL_TIMEOUT_S:-1200}"
  bash
  "$REPO_ROOT/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_pr6_d435i_visual_headless.sh"
)

set +e
if [[ "$MISSION_MODE" == rectangle ]]; then
  "${headless_command[@]}" >"$RUN_DIR/headless_capture.log" 2>&1
  headless_status=$?
  mission_status=$headless_status
else
  setsid "${headless_command[@]}" >"$RUN_DIR/headless_capture.log" 2>&1 &
  headless_pid=$!
  pids+=("$headless_pid")
  if wait_for_mission_gate "$headless_pid" "$MISSION_START_WAIT_S" \
      >"$RUN_DIR/mission_start_gate.log" 2>&1; then
    timeout --signal=TERM --kill-after=20s "$MISSION_WALL_TIMEOUT_S" \
      env \
      TAKEOFF_ALT="${TAKEOFF_ALT:-3.5}" \
      MINIMUM_CLEARANCE_ALT="${MINIMUM_CLEARANCE_ALT:-2.5}" \
      S_CURVE_SPAN="${S_CURVE_SPAN:-4.0}" \
      S_CURVE_AMPLITUDE="${S_CURVE_AMPLITUDE:-1.5}" \
      S_CURVE_VERTICAL_AMPLITUDE="${S_CURVE_VERTICAL_AMPLITUDE:-0.5}" \
      S_CURVE_VERTICAL_CYCLES="${S_CURVE_VERTICAL_CYCLES:-1}" \
      S_CURVE_PASSES="${S_CURVE_PASSES:-1}" \
      S_CURVE_SPEED="${S_CURVE_SPEED:-0.30}" \
      S_CURVE_HOLD_TIME="${S_CURVE_HOLD_TIME:-1.0}" \
      S_CURVE_WAYPOINT_SPACING="${S_CURVE_WAYPOINT_SPACING:-2.0}" \
      S_CURVE_WAYPOINT_HOLD="${S_CURVE_WAYPOINT_HOLD:-0.5}" \
      POST_TAKEOFF_HOLD_TIME="${POST_TAKEOFF_HOLD_TIME:-2.0}" \
      CALIBRATION_YAW_SWEEP_DEG="${CALIBRATION_YAW_SWEEP_DEG:-12.0}" \
      CALIBRATION_YAW_CYCLES="${CALIBRATION_YAW_CYCLES:-3.0}" \
      LOCALIZATION_SAFETY_ENABLED="${LOCALIZATION_SAFETY_ENABLED:-true}" \
      LAND_AT_END="${LAND_AT_END:-true}" \
      LOG_DIR="$RUN_DIR/s_curve" \
      bash "$REPO_ROOT/tools/run_s_curve_state_machine.sh" \
      >"$RUN_DIR/s_curve_console.log" 2>&1
    mission_status=$?
  else
    mission_status=8
  fi
  sleep 10
  kill -TERM -- "-$headless_pid" 2>/dev/null || true
  kill -TERM "$headless_pid" 2>/dev/null || true
  wait "$headless_pid" 2>/dev/null
  headless_process_status=$?
  headless_status=$mission_status
fi
set -e

cleanup
trap - EXIT INT TERM
sleep 3
ros2 bag info "$BAG_DIR" >"$RUN_DIR/bag_info.txt"
verify_args=(
  python3 "$REPO_ROOT/tools/verify_estimator_input_bag.py"
  --bag "$BAG_DIR"
  --output "$RUN_DIR/bag_contract.json"
)
if [[ "$REQUIRE_VISUAL_BAG" == 1 ]]; then
  verify_args+=(
    --require-visual
    --require-visual-factor-score
    --require-rgbd-geometry
  )
fi
set +e
"${verify_args[@]}" \
  >"$RUN_DIR/bag_contract.log" 2>&1
bag_contract_status=$?
set -e
printf 'capture_status=%s\nbag=%s\n' "$headless_status" "$BAG_DIR" \
  >"$RUN_DIR/capture_result.env"
printf 'bag_contract_status=%s\n' "$bag_contract_status" \
  >>"$RUN_DIR/capture_result.env"
printf 'visual_bridge_enabled=%s\nvisual_frontend_enabled=%s\n' \
  "$VISUAL_BRIDGE_ENABLED" "$VISUAL_FRONTEND_ENABLED" \
  >>"$RUN_DIR/capture_result.env"
printf 'visual_factor_mode=%s\nrequire_visual_bag=%s\n' \
  "$VISUAL_FACTOR_MODE" "$REQUIRE_VISUAL_BAG" \
  >>"$RUN_DIR/capture_result.env"
printf 'mission_mode=%s\nmission_status=%s\nbackend_cpuset=%s\nbackend_numeric_threads=%s\n' \
  "$MISSION_MODE" "$mission_status" "${BACKEND_CPUSET:-normal}" \
  "$BACKEND_NUMERIC_THREADS" >>"$RUN_DIR/capture_result.env"
if [[ -n "${headless_process_status:-}" ]]; then
  printf 'headless_process_status=%s\n' "$headless_process_status" \
    >>"$RUN_DIR/capture_result.env"
fi
cat "$RUN_DIR/bag_info.txt"
if [[ "$headless_status" != 0 ]]; then
  exit "$headless_status"
fi
exit "$bag_contract_status"
