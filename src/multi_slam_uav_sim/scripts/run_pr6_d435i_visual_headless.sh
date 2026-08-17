#!/usr/bin/env bash
set -Eeo pipefail

# Fast DDS discovery is unreliable in the restored WSL environment.  Keep an
# explicit caller override, but make the validated ROS 2 transport the default.
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
if [[ -f "$LIDAR_WS/install/setup.bash" ]]; then
  # Source the selected FAST-LIO overlay in this parent process as well as in
  # the mapping wrapper.  The Python backend must be able to import the
  # NativeLidarFactor type before its subscriptions are constructed.
  source "$LIDAR_WS/install/setup.bash"
fi
source "$PKG_SHARE/scripts/d435i_active_run_lifecycle.sh"

RUN_ID=${RUN_ID:-paper_visual_$(date +%Y%m%d_%H%M%S)}
RUN_DIR=${RUN_DIR:-$WS_ROOT/logs/paper_visual/$RUN_ID}
ACTIVE_FILE=${ACTIVE_FILE:-$WS_ROOT/logs/d435i_visual_slam/.active_headless}
RUN_SMALL_RECTANGLE=${RUN_SMALL_RECTANGLE:-0}
EXIT_AFTER_RECTANGLE=${EXIT_AFTER_RECTANGLE:-0}
EXPECT_EXTERNAL_VISUAL_MOTION=${EXPECT_EXTERNAL_VISUAL_MOTION:-0}
RECTANGLE_LENGTH_X=${RECTANGLE_LENGTH_X:-2.0}
RECTANGLE_LENGTH_Y=${RECTANGLE_LENGTH_Y:-1.2}
RECTANGLE_TAKEOFF_ALT=${RECTANGLE_TAKEOFF_ALT:-2.0}
RECTANGLE_SPEED_MPS=${RECTANGLE_SPEED_MPS:-0.8}
RECTANGLE_YAW_RATE_DEG_S=${RECTANGLE_YAW_RATE_DEG_S:-12.0}
RECTANGLE_FACE_EDGES=${RECTANGLE_FACE_EDGES:-1}
case "$RECTANGLE_FACE_EDGES" in
  0) RECTANGLE_FACE_EDGES_BOOL=false ;;
  1) RECTANGLE_FACE_EDGES_BOOL=true ;;
  *) printf 'RECTANGLE_FACE_EDGES must be 0 or 1.\n' >&2; exit 2 ;;
esac
case "$EXIT_AFTER_RECTANGLE" in
  0|1) ;;
  *) printf 'EXIT_AFTER_RECTANGLE must be 0 or 1.\n' >&2; exit 2 ;;
esac
case "$EXPECT_EXTERNAL_VISUAL_MOTION" in
  0|1) ;;
  *) printf 'EXPECT_EXTERNAL_VISUAL_MOTION must be 0 or 1.\n' >&2; exit 2 ;;
esac
PR6_START_RTABMAP=${PR6_START_RTABMAP:-1}
VISUAL_BRIDGE_ENABLED=${VISUAL_BRIDGE_ENABLED:-1}
VISUAL_FRONTEND_ENABLED=${VISUAL_FRONTEND_ENABLED:-1}
EXTERNAL_NAV_ENABLED=${EXTERNAL_NAV_ENABLED:-1}
NATIVE_LIDAR_WAIT_S=${NATIVE_LIDAR_WAIT_S:-240}
EXTERNAL_NAV_WAIT_S=${EXTERNAL_NAV_WAIT_S:-120}
case "$PR6_START_RTABMAP" in
  0) PR6_START_RTABMAP_BOOL=false ;;
  1) PR6_START_RTABMAP_BOOL=true ;;
  *) printf 'PR6_START_RTABMAP must be 0 or 1.\n' >&2; exit 2 ;;
esac
case "$VISUAL_BRIDGE_ENABLED" in
  0) VISUAL_BRIDGE_ENABLED_BOOL=false ;;
  1) VISUAL_BRIDGE_ENABLED_BOOL=true ;;
  *) printf 'VISUAL_BRIDGE_ENABLED must be 0 or 1.\n' >&2; exit 2 ;;
esac
case "$VISUAL_FRONTEND_ENABLED" in
  0) VISUAL_FRONTEND_ENABLED_BOOL=false ;;
  1) VISUAL_FRONTEND_ENABLED_BOOL=true ;;
  *) printf 'VISUAL_FRONTEND_ENABLED must be 0 or 1.\n' >&2; exit 2 ;;
esac
case "$EXTERNAL_NAV_ENABLED" in
  0) EXTERNAL_NAV_ENABLED_BOOL=false ;;
  1) EXTERNAL_NAV_ENABLED_BOOL=true ;;
  *) printf 'EXTERNAL_NAV_ENABLED must be 0 or 1.\n' >&2; exit 2 ;;
esac
VISUAL_FACTOR_MODE=${VISUAL_FACTOR_MODE:-paper_reprojection}
SIM_RGBD_MIN_DEPTH_M=${SIM_RGBD_MIN_DEPTH_M:-0.30}
SIM_RGBD_MAX_DEPTH_M=${SIM_RGBD_MAX_DEPTH_M:-10.0}
case "$VISUAL_FACTOR_MODE" in
  disabled|paper_reprojection) ;;
  *) printf 'VISUAL_FACTOR_MODE must be disabled or paper_reprojection.\n' >&2; exit 2 ;;
esac
VISUAL_KEYFRAME_PROFILE=${VISUAL_KEYFRAME_PROFILE:-balanced}
case "$VISUAL_KEYFRAME_PROFILE" in
  conservative|balanced_light|balanced|balanced_plus|dense|custom) ;;
  *) printf 'VISUAL_KEYFRAME_PROFILE is not a supported cadence.\n' >&2; exit 2 ;;
esac
VISUAL_CANDIDATE_QUALITY_ENABLED=${VISUAL_CANDIDATE_QUALITY_ENABLED:-1}
VISUAL_PENDING_ENABLED=${VISUAL_PENDING_ENABLED:-1}
VISUAL_REQUIRE_TIME_LOCK=${VISUAL_REQUIRE_TIME_LOCK:-0}
PERFORMANCE_PROFILING_ENABLED=${PERFORMANCE_PROFILING_ENABLED:-0}
BACKEND_CPUSET=${BACKEND_CPUSET:-}
BACKEND_NUMERIC_THREADS=${BACKEND_NUMERIC_THREADS:-1}
case "$VISUAL_CANDIDATE_QUALITY_ENABLED" in
  0) VISUAL_CANDIDATE_QUALITY_ENABLED_BOOL=false ;;
  1) VISUAL_CANDIDATE_QUALITY_ENABLED_BOOL=true ;;
  *) printf 'VISUAL_CANDIDATE_QUALITY_ENABLED must be 0 or 1.\n' >&2; exit 2 ;;
esac
case "$VISUAL_PENDING_ENABLED" in
  0) VISUAL_PENDING_ENABLED_BOOL=false ;;
  1) VISUAL_PENDING_ENABLED_BOOL=true ;;
  *) printf 'VISUAL_PENDING_ENABLED must be 0 or 1.\n' >&2; exit 2 ;;
esac
case "$VISUAL_REQUIRE_TIME_LOCK" in
  0) VISUAL_REQUIRE_TIME_LOCK_BOOL=false ;;
  1) VISUAL_REQUIRE_TIME_LOCK_BOOL=true ;;
  *) printf 'VISUAL_REQUIRE_TIME_LOCK must be 0 or 1.\n' >&2; exit 2 ;;
esac
case "$PERFORMANCE_PROFILING_ENABLED" in
  0) PERFORMANCE_PROFILING_ENABLED_BOOL=false ;;
  1) PERFORMANCE_PROFILING_ENABLED_BOOL=true ;;
  *) printf 'PERFORMANCE_PROFILING_ENABLED must be 0 or 1.\n' >&2; exit 2 ;;
esac
if [[ ! "$BACKEND_NUMERIC_THREADS" =~ ^[1-9][0-9]*$ ]]; then
  printf 'BACKEND_NUMERIC_THREADS must be a positive integer.\n' >&2
  exit 2
fi
BACKEND_PROCESS_PREFIX=""
if [[ -n "$BACKEND_CPUSET" ]]; then
  if ! taskset --cpu-list "$BACKEND_CPUSET" true 2>/dev/null; then
    printf 'BACKEND_CPUSET is not a valid CPU list: %s\n' "$BACKEND_CPUSET" >&2
    exit 2
  fi
  BACKEND_PROCESS_PREFIX="taskset --cpu-list $BACKEND_CPUSET"
fi
ONLINE_MAPPING_MODE=${ONLINE_MAPPING_MODE:-disabled}
case "$ONLINE_MAPPING_MODE" in
  disabled) SHARED_MAPPING_ENABLED=false; SHARED_MAPPING_RGBD_ENABLED=false ;;
  lidar_only) SHARED_MAPPING_ENABLED=true; SHARED_MAPPING_RGBD_ENABLED=false ;;
  joint) SHARED_MAPPING_ENABLED=true; SHARED_MAPPING_RGBD_ENABLED=true ;;
  *) printf 'ONLINE_MAPPING_MODE must be disabled, lidar_only, or joint.\n' >&2; exit 2 ;;
esac
mkdir -p "$RUN_DIR" "$(dirname "$ACTIVE_FILE")"
printf 'visual_bridge_enabled=%s\nvisual_frontend_enabled=%s\n' \
  "$VISUAL_BRIDGE_ENABLED" "$VISUAL_FRONTEND_ENABLED" \
  >"$RUN_DIR/visual_ablation_mode.env"
printf 'visual_factor_mode=%s\nvisual_keyframe_profile=%s\n' \
  "$VISUAL_FACTOR_MODE" "$VISUAL_KEYFRAME_PROFILE" \
  >>"$RUN_DIR/visual_ablation_mode.env"
printf 'rtabmap_enabled=%s\nonline_mapping_mode=%s\n' \
  "$PR6_START_RTABMAP" "$ONLINE_MAPPING_MODE" \
  >>"$RUN_DIR/visual_ablation_mode.env"
printf 'external_nav_enabled=%s\n' "$EXTERNAL_NAV_ENABLED" \
  >>"$RUN_DIR/visual_ablation_mode.env"
printf 'backend_cpuset=%s\nbackend_numeric_threads=%s\n' \
  "${BACKEND_CPUSET:-normal}" "$BACKEND_NUMERIC_THREADS" \
  >>"$RUN_DIR/visual_ablation_mode.env"
printf 'visual_require_time_lock=%s\n' "$VISUAL_REQUIRE_TIME_LOCK" \
  >>"$RUN_DIR/visual_ablation_mode.env"
printf 'sim_rgbd_depth_range_m=%s..%s\n' \
  "$SIM_RGBD_MIN_DEPTH_M" "$SIM_RGBD_MAX_DEPTH_M" \
  >>"$RUN_DIR/visual_ablation_mode.env"
STARTUP_TRACE="$RUN_DIR/startup_chain.tsv"
printf 'stage\twall_utc\telapsed_wall_s\n' >"$STARTUP_TRACE"

trace_stage() {
  printf '%s\t%s\t%s\n' "$1" "$(date -u +%Y-%m-%dT%H:%M:%SZ)" "$SECONDS" \
    >>"$STARTUP_TRACE"
}

if [[ -f "$ACTIVE_FILE" ]] && d435i_active_read "$ACTIVE_FILE" && \
   kill -0 "$D435I_ACTIVE_PID" 2>/dev/null; then
  printf 'A D435i headless stack is already active: pid=%s run=%s\n' \
    "$D435I_ACTIVE_PID" "$D435I_ACTIVE_RUN_DIR" >&2
  exit 2
fi

RUN_TOKEN="$$-$(date +%s%N)"
RUN_BRANCH=$(git -C "$WS_ROOT" branch --show-current 2>/dev/null || true)
PID_MANIFEST="$RUN_DIR/pids.tsv"
printf 'component\tpid\tprocess_group\tstart_ticks\n' >"$PID_MANIFEST"
d435i_active_write "$ACTIVE_FILE" "$$" "$RUN_DIR" "$WS_ROOT" \
  "$RUN_BRANCH" "$RUN_ID" "$RUN_TOKEN" "$(realpath -m "${BASH_SOURCE[0]}")"

record_pid() {
  local component=$1 pid=$2 ticks=
  ticks=$(d435i_process_start_ticks "$pid" 2>/dev/null || true)
  printf '%s\t%s\t%s\t%s\n' "$component" "$pid" "$pid" "$ticks" >>"$PID_MANIFEST"
}

cleanup_started=0
stop_recorded_groups() {
  local component pid pgid ticks current_ticks
  [[ -f "$PID_MANIFEST" ]] || return 0
  while IFS=$'\t' read -r component pid pgid ticks; do
    [[ "$component" == component || -z "$pid" ]] && continue
    current_ticks=$(d435i_process_start_ticks "$pid" 2>/dev/null || true)
    [[ -n "$ticks" && "$current_ticks" == "$ticks" ]] || continue
    kill -TERM -- "-$pgid" 2>/dev/null || true
  done <"$PID_MANIFEST"
  sleep 2
  while IFS=$'\t' read -r component pid pgid ticks; do
    [[ "$component" == component || -z "$pid" ]] && continue
    current_ticks=$(d435i_process_start_ticks "$pid" 2>/dev/null || true)
    [[ -n "$ticks" && "$current_ticks" == "$ticks" ]] || continue
    kill -KILL -- "-$pgid" 2>/dev/null || true
  done <"$PID_MANIFEST"
}

cleanup() {
  local status=$?
  [[ "$cleanup_started" == 0 ]] || return
  cleanup_started=1
  trap - EXIT INT TERM
  # The paper-mode overlay is a new launch entry point. Stop only process
  # groups recorded by this run, guarded by /proc start ticks so PID reuse or
  # unrelated simulation jobs cannot be targeted.
  stop_recorded_groups
  d435i_cleanup_run_manifests "$RUN_DIR" "$WS_ROOT" \
    "$RUN_DIR/process_cleanup.log"
  d435i_active_remove_owned "$ACTIVE_FILE" "$$" "$RUN_TOKEN" || true
  printf 'Paper visual stack stopped (status=%s). Logs: %s\n' "$status" "$RUN_DIR"
  exit "$status"
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic=$1 timeout_s=${2:-90}
  if python3 "$PKG_SHARE/scripts/wait_for_ros_message.py" \
      --topic "$topic" --timeout "$timeout_s" \
      --reliability best_effort >/dev/null; then
    printf 'ready: %s\n' "$topic"
    return 0
  fi
  printf 'Timed out waiting for %s\n' "$topic" >&2
  return 1
}

wait_for_publisher() {
  local topic=$1 timeout_s=${2:-45} started=$SECONDS info=
  while (( SECONDS - started < timeout_s )); do
    info=$(timeout 5s ros2 topic info "$topic" --no-daemon 2>/dev/null || true)
    if grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$info"; then
      printf 'ready publisher: %s\n' "$topic"
      return 0
    fi
    sleep 1
  done
  printf 'Timed out waiting for publisher %s\n' "$topic" >&2
  return 1
}

existing_lio_adapter_pids() {
  ps -eo pid=,args= | awk '
    $0 ~ /[/]uf_lio_adapter[/]lib[/]uf_lio_adapter[/]lio_adapter([[:space:]]|$)/ {
      print $1
    }'
}

wait_for_single_publisher() {
  local topic=$1 timeout_s=${2:-45} started=$SECONDS info= count=0 stable=0
  while (( SECONDS - started < timeout_s )); do
    info=$(timeout 5s ros2 topic info "$topic" --no-daemon 2>/dev/null || true)
    count=$(sed -n 's/^Publisher count: \([0-9][0-9]*\)$/\1/p' <<<"$info")
    count=${count:-0}
    if (( count > 1 )); then
      printf 'Refusing duplicate ownership of %s: publishers=%s\n' \
        "$topic" "$count" >&2
      return 1
    fi
    if (( count == 1 )); then
      stable=$((stable + 1))
      (( stable >= 2 )) && return 0
    else
      stable=0
    fi
    sleep 1
  done
  printf 'Timed out waiting for single publisher of %s: publishers=%s\n' \
    "$topic" "$count" >&2
  return 1
}

get_parameter_with_discovery_retry() {
  local node=$1 parameter=$2 timeout_s=${3:-45}
  python3 "$PKG_SHARE/scripts/wait_for_ros_parameter.py" \
    --node "$node" --parameter "$parameter" --timeout "$timeout_s"
}

wait_for_livox_ownership() {
  local timeout_s=${1:-90} started=$SECONDS stable=0 lidar_info imu_info
  local lidar_count=0 imu_count=0
  while (( SECONDS - started < timeout_s )); do
    # Query DDS directly.  The ROS graph daemon may retain the previous RMW
    # graph after switching middleware and can otherwise report zero owners.
    lidar_info=$(timeout 5s ros2 topic info /livox/lidar --no-daemon 2>/dev/null || true)
    imu_info=$(timeout 5s ros2 topic info /livox/imu --no-daemon 2>/dev/null || true)
    lidar_count=$(sed -n 's/^Publisher count: \([0-9][0-9]*\)$/\1/p' <<<"$lidar_info")
    imu_count=$(sed -n 's/^Publisher count: \([0-9][0-9]*\)$/\1/p' <<<"$imu_info")
    lidar_count=${lidar_count:-0}
    imu_count=${imu_count:-0}
    if (( lidar_count == 1 && imu_count == 1 )); then
      stable=$((stable + 1))
      if (( stable >= 2 )); then
        printf 'ready stable ownership: /livox/lidar=1 /livox/imu=1\n'
        return 0
      fi
    else
      stable=0
    fi
    sleep 1
  done
  printf 'Timed out waiting for stable Livox ownership: lidar=%s imu=%s\n' \
    "$lidar_count" "$imu_count" >&2
  return 1
}

wait_for_valid_vision() {
  local timeout_s=${1:-60}
  if python3 "$PKG_SHARE/scripts/wait_for_ros_message.py" \
      --topic /reliability/vision_score --timeout "$timeout_s" \
      --reliability best_effort --field valid --equals true >/dev/null; then
    printf 'ready: D_V_rgbd valid\n'
    return 0
  fi
  printf 'Timed out waiting for valid D_V_rgbd\n' >&2
  return 1
}

printf 'Starting PR #6 sensor stack. Logs: %s\n' "$RUN_DIR"
PR6_HEADLESS=${PR6_HEADLESS:-1}
if [[ "$PR6_HEADLESS" == "1" ]]; then
  PR6_GAZEBO_GUI=0
elif [[ "$PR6_HEADLESS" == "0" ]]; then
  PR6_GAZEBO_GUI=1
else
  printf 'PR6_HEADLESS must be 0 or 1, got %s\n' "$PR6_HEADLESS" >&2
  exit 2
fi
setsid env \
  HEADLESS="$PR6_HEADLESS" GAZEBO_GUI="$PR6_GAZEBO_GUI" SHOW_FLOW_WINDOW=0 \
  LOG_DIR="$RUN_DIR/sensor_stack" \
  LOCK_FILE="$RUN_DIR/apm_sensor_stack.lock" \
  ENABLE_D435_BRIDGE=0 ENABLE_D435_POINTCLOUD=false \
  MID360_SIM_BRIDGE_MODE=direct_livox \
  ENABLE_GAZEBO_FLOW=1 ENABLE_FCU_FLOW=0 ENABLE_FCU_FLOW_ROUTER=0 \
  START_SITL=1 START_MAVROS=1 RECTANGLE_FLOW_TEST=0 AUTO_FLIGHT=0 \
  REQUIRE_GAZEBO_GPU=${REQUIRE_GAZEBO_GPU:-0} \
  LIDAR_WS="$LIDAR_WS" \
  bash "$PKG_SHARE/scripts/run_apm_sensor_stack.sh" \
  >"$RUN_DIR/sensor_stack_supervisor.log" 2>&1 &
record_pid stack_supervisor "$!"

# Construct every backend subscription while Gazebo/SITL are still starting.
# This removes CLI readiness polling from the estimator critical path while
# preserving the volatile NativeLidarFactor bootstrap contract.
stale_lio_pids=$(existing_lio_adapter_pids)
if [[ -n "$stale_lio_pids" ]]; then
  printf 'Refusing to start: an independent lio_adapter is already running (PIDs: %s).\n' \
    "$(tr '\n' ' ' <<<"$stale_lio_pids")" >&2
  exit 2
fi
backend_prefix_launch_args=()
if [[ -n "$BACKEND_PROCESS_PREFIX" ]]; then
  backend_prefix_launch_args+=(
    "backend_process_prefix:=$BACKEND_PROCESS_PREFIX"
  )
fi
setsid ros2 launch multi_slam_uav_sim d435i_paper_visual_integration.launch.py \
  use_sim_time:=true \
  start_rgbd_bridge:="$VISUAL_BRIDGE_ENABLED_BOOL" \
  start_visual_frontend:="$VISUAL_FRONTEND_ENABLED_BOOL" \
  start_rtabmap:="$PR6_START_RTABMAP_BOOL" \
  visual_factor_mode:="$VISUAL_FACTOR_MODE" \
  visual_keyframe_profile:="$VISUAL_KEYFRAME_PROFILE" \
  visual_candidate_quality_enabled:="$VISUAL_CANDIDATE_QUALITY_ENABLED_BOOL" \
  visual_pending_enabled:="$VISUAL_PENDING_ENABLED_BOOL" \
  rgbd_minimum_depth_m:="$SIM_RGBD_MIN_DEPTH_M" \
  rgbd_maximum_depth_m:="$SIM_RGBD_MAX_DEPTH_M" \
  visual_initialization_require_time_lock:="$VISUAL_REQUIRE_TIME_LOCK_BOOL" \
  external_nav_enabled:="$EXTERNAL_NAV_ENABLED_BOOL" \
  performance_profiling_enabled:="$PERFORMANCE_PROFILING_ENABLED_BOOL" \
  performance_trace_path:="$RUN_DIR/backend_cycle_trace.jsonl" \
  barometer_topic:="${BAROMETER_TOPIC:-/sim/barometer/pressure}" \
  "${backend_prefix_launch_args[@]}" \
  backend_numeric_threads:="$BACKEND_NUMERIC_THREADS" \
  shared_mapping_enabled:="$SHARED_MAPPING_ENABLED" \
  shared_mapping_rgbd_enabled:="$SHARED_MAPPING_RGBD_ENABLED" \
  shared_mapping_output_directory:="$RUN_DIR/shared_map" \
  database_path:="$RUN_DIR/rtabmap.db" \
  >"$RUN_DIR/integration_overlay.log" 2>&1 &
record_pid integration_overlay "$!"
wait_for_publisher /fusion/unified/diagnostics 60
wait_for_single_publisher /lio/diagnostics 60
trace_stage backend_subscriber_ready

# The latest mainline sensor supervisor owns the single Gazebo-to-ROS clock
# bridge and validates that it advances. Reuse it here to avoid a duplicate
# publisher and subscribe with the offered best-effort clock QoS.
wait_for_topic /clock 90
trace_stage clock_ready
wait_for_topic /livox/lidar 120
trace_stage lidar_ready
wait_for_topic /livox/imu 120
trace_stage lidar_imu_ready
wait_for_livox_ownership 90
trace_stage livox_ownership_stable

setsid env \
  LOG_DIR="$RUN_DIR/fastlio" RVIZ=0 LIDAR_WS="$LIDAR_WS" \
  FASTLIO_INPUT_MODE=livox START_LIVOX_POINTCLOUD_BRIDGE=0 \
  FASTLIO_NATIVE_FACTOR_EXPORT=1 \
  FASTLIO_DOWNSTREAM_BACKEND=1 \
  FASTLIO_MAP_INSERTION_MODE=backend_confirmed \
  FASTLIO_BACKEND_TRAJECTORY_FRONTEND=1 \
  bash "$PKG_SHARE/scripts/run_mid360_fastlio_mapping.sh" \
  >"$RUN_DIR/fastlio_supervisor.log" 2>&1 &
record_pid fastlio_supervisor "$!"
if ! wait_for_topic /fast_lio/native_lidar_factor "$NATIVE_LIDAR_WAIT_S"; then
  timeout 10s ros2 topic info /fast_lio/native_lidar_factor --verbose \
    >"$RUN_DIR/startup_failure_native_factor_topic.txt" 2>&1 || true
  timeout 10s ros2 topic echo /fusion/unified/diagnostics \
    --no-daemon --spin-time 7.0 --once --full-length \
    >"$RUN_DIR/startup_failure_native_factor_diagnostics.yaml" 2>&1 || true
  printf 'NativeLidarFactor is required for the paper-mode regression.\n' >&2
  exit 3
fi
trace_stage native_lidar_factor_ready
printf 'input_trigger=native_factor\nnative_factor=true\nlio_pose_fallback=false\n' \
  >"$RUN_DIR/backend_runtime_mode.env"
if ! wait_for_topic /fusion/unified/odom 120; then
  timeout 10s ros2 topic echo /fusion/unified/diagnostics \
    --no-daemon --spin-time 7.0 --once --full-length \
    >"$RUN_DIR/startup_failure_backend_diagnostics.yaml" 2>&1 || true
  printf 'Unified backend did not publish after NativeLidarFactor bootstrap.\n' >&2
  exit 4
fi
trace_stage first_unified_odom
setsid python3 "$WS_ROOT/src/ultra_fusion_nav/scripts/record_reliability_timeline.py" \
  --duration "${EVIDENCE_ROS_DURATION_S:-60}" \
  --wall-timeout "${EVIDENCE_WALL_TIMEOUT_S:-180}" \
  --output "$RUN_DIR/runtime_evidence.json" \
  >"$RUN_DIR/runtime_evidence.log" 2>&1 &
record_pid runtime_evidence "$!"
setsid python3 "$WS_ROOT/src/ultra_fusion_nav/scripts/record_lio_trajectory.py" \
  --duration "${TRAJECTORY_ROS_DURATION_S:-45}" \
  --wall-timeout "${TRAJECTORY_WALL_TIMEOUT_S:-600}" \
  --estimate-topic /fusion/unified/odom \
  --truth-topic /sim/mid360/ground_truth_odom \
  --output-dir "$RUN_DIR/trajectory" \
  >"$RUN_DIR/trajectory_recorder.log" 2>&1 &
record_pid trajectory_recorder "$!"
if [[ "$EXTERNAL_NAV_ENABLED" == 1 ]]; then
  setsid timeout 20s ros2 topic echo /external_nav/diagnostics \
    --no-daemon --spin-time 10.0 --once --full-length \
    >"$RUN_DIR/external_nav_first_diagnostics.yaml" 2>&1 &
  record_pid external_nav_diagnostics "$!"
fi
setsid ros2 run multi_slam_uav_sim simulation_performance_monitor --ros-args \
  -p use_sim_time:=true \
  -p output_path:="$RUN_DIR/simulation_performance.json" \
  -p fusion_topic:=/fusion/unified/odom \
  -p fusion_diagnostic_topic:=/fusion/unified/diagnostics \
  -p include_compute_time_series:="$PERFORMANCE_PROFILING_ENABLED_BOOL" \
  >"$RUN_DIR/simulation_performance.log" 2>&1 &
record_pid simulation_performance "$!"

# These checks are mission-readiness gates, not prerequisites for creating the
# first native LiDAR state. Keeping them after first odom removes serial ROS CLI
# startup latency without weakening any estimator observability condition.
wait_for_topic /mavros/imu/data_raw 120
trace_stage imu_ready
wait_for_topic /sim/optical_flow/rad 90
trace_stage flow_ready
wait_for_topic /mavros/global_position/raw/fix 120
trace_stage gnss_ready
if [[ "$VISUAL_BRIDGE_ENABLED" == 1 ]]; then
  wait_for_topic /sensors/rgbd/color 90
  wait_for_topic /sensors/rgbd/depth 45
  wait_for_topic /front/d435i/color/camera_info 45
  trace_stage visual_bridge_ready
else
  trace_stage visual_bridge_disabled
fi
if [[ "$VISUAL_FRONTEND_ENABLED" == 1 ]]; then
  wait_for_publisher /reliability/vision_score 45
  wait_for_publisher /vision/frontend_diagnostics 45
  wait_for_publisher /fusion/unified/visual_timing 45
  trace_stage visual_frontend_ready
else
  trace_stage visual_frontend_disabled
fi
backend_visual_mode=$(get_parameter_with_discovery_retry \
  /unified_backend_fusion visual_factor_mode 45 || true)
if [[ "$backend_visual_mode" != *"$VISUAL_FACTOR_MODE"* ]]; then
  printf 'Visual runtime contract mismatch: requested=%s actual=%s\n' \
    "$VISUAL_FACTOR_MODE" "$backend_visual_mode" >&2
  exit 5
fi
printf 'visual_factor_mode=%s\n' "$VISUAL_FACTOR_MODE" \
  >>"$RUN_DIR/backend_runtime_mode.env"
if [[ "$PR6_START_RTABMAP" == 1 ]]; then
  wait_for_publisher /rtabmap/odom 120
fi
wait_for_publisher /reliability/scheduler_state 45
if [[ "$EXTERNAL_NAV_ENABLED" == 1 ]]; then
  if ! wait_for_topic /fusion/runtime_external_nav "$EXTERNAL_NAV_WAIT_S"; then
    timeout 10s ros2 topic echo /external_nav/diagnostics \
      --no-daemon --spin-time 7.0 --once --full-length \
      >"$RUN_DIR/startup_failure_external_nav_diagnostics.yaml" 2>&1 || true
    timeout 10s ros2 topic echo /reliability/scheduler_state \
      --no-daemon --spin-time 7.0 --once --full-length \
      >"$RUN_DIR/startup_failure_scheduler_state.yaml" 2>&1 || true
    printf 'ExternalNav gate did not become usable.\n' >&2
    exit 7
  fi
  trace_stage external_nav_gate_open
else
  trace_stage external_nav_gate_disabled
fi
if [[ "$ONLINE_MAPPING_MODE" != disabled ]]; then
  # At startup the shared map is intentionally empty. Under software-rendered
  # simulation load, waiting for a serialized empty PointCloud2 can race the
  # first LiDAR callback even though the mapping node is healthy. Publisher
  # ownership is the correct readiness contract; exported non-empty map and
  # source/conflict metrics remain mandatory post-mission validation.
  wait_for_publisher /mapping/shared/points 45
  mapping_output=$(timeout 10s ros2 param get \
    /uf_shared_mapping output_directory \
    --no-daemon --spin-time 3.0 2>/dev/null || true)
  if [[ "$mapping_output" != *"$RUN_DIR/shared_map"* ]]; then
    printf 'Shared-map output contract mismatch: expected=%s actual=%s\n' \
      "$RUN_DIR/shared_map" "$mapping_output" >&2
    exit 6
  fi
  trace_stage shared_mapping_ready
fi

if [[ "$RUN_SMALL_RECTANGLE" == 1 ]]; then
  setsid ros2 run multi_slam_uav_sim guided_rectangle_waypoints --ros-args \
    -p use_sim_time:=true \
    -p takeoff_alt:="$RECTANGLE_TAKEOFF_ALT" -p length_x:="$RECTANGLE_LENGTH_X" \
    -p length_y:="$RECTANGLE_LENGTH_Y" \
    -p speed_mps:="$RECTANGLE_SPEED_MPS" \
    -p yaw_rate_deg_s:="$RECTANGLE_YAW_RATE_DEG_S" \
    -p face_rectangle_edges:="$RECTANGLE_FACE_EDGES_BOOL" \
    -p land_at_end:=true \
    >"$RUN_DIR/small_rectangle.log" 2>&1 &
  RECTANGLE_PID=$!
  record_pid rectangle_motion "$RECTANGLE_PID"
fi
# A quality-gated visual candidate needs real parallax.  Waiting for feature
# tracks before starting a configured motion mission deadlocks at the static
# takeoff pose.  The raw RGB-D stream and frontend publishers are verified
# above; only the first quality-valid candidate is allowed to wait for motion.
if [[ "$VISUAL_FRONTEND_ENABLED" == 1 ]]; then
  if [[ "$RUN_SMALL_RECTANGLE" == 1 ]]; then
    wait_for_topic /vision/feature_tracks 120
    wait_for_valid_vision 120
    trace_stage visual_ready
  elif [[ "$EXPECT_EXTERNAL_VISUAL_MOTION" == 1 ]]; then
    # The external mission starts only after this wrapper reports ready.  Let
    # the scheduler keep vision disabled until that motion creates parallax.
    trace_stage visual_waiting_for_external_motion
  else
    wait_for_topic /vision/feature_tracks 90
    wait_for_valid_vision 120
    trace_stage visual_ready
  fi
else
  trace_stage visual_candidate_gate_disabled
fi

printf 'Paper reprojection + D435i visual integration is ready.\n'
printf '  RTAB startup: %s\n' "$PR6_START_RTABMAP_BOOL"
printf '  RGB-D: /sensors/rgbd/{color,depth}\n'
printf '  RTAB odom: /rtabmap/odom\n'
printf '  D_V_rgbd: /reliability/vision_score\n'
printf '  Features: /vision/feature_tracks\n'
printf '  FAST-LIO: /fast_lio/native_lidar_factor\n'
printf '  Unified backend: /fusion/unified/odom\n'
printf '  Logs: %s\n' "$RUN_DIR"
if [[ "$RUN_SMALL_RECTANGLE" == 1 && "$EXIT_AFTER_RECTANGLE" == 1 ]]; then
  set +e
  wait "$RECTANGLE_PID"
  rectangle_status=$?
  set -e
  if [[ "$ONLINE_MAPPING_MODE" != disabled ]]; then
    timeout 20s ros2 service call /mapping/shared/export std_srvs/srv/Trigger '{}' \
      >"$RUN_DIR/shared_map_export.log" 2>&1 || true
  fi
  for _ in {1..20}; do
    [[ -s "$RUN_DIR/trajectory/estimate.tum" ]] && break
    sleep 0.5
  done
  if [[ -s "$RUN_DIR/trajectory/estimate.tum" && \
        -s "$RUN_DIR/trajectory/ground_truth.tum" ]]; then
    python3 "$WS_ROOT/src/ultra_fusion_nav/scripts/evaluate_lio_trajectory.py" \
      --estimate "$RUN_DIR/trajectory/estimate.tum" \
      --truth "$RUN_DIR/trajectory/ground_truth.tum" \
      --output "$RUN_DIR/trajectory_metrics.json" \
      >"$RUN_DIR/trajectory_evaluation.log" 2>&1 || true
  fi
  printf 'small_rectangle_exit=%s\n' "$rectangle_status" \
    >"$RUN_DIR/rectangle_result.env"
  exit "$rectangle_status"
fi
wait
