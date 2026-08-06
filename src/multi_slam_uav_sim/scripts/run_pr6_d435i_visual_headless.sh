#!/usr/bin/env bash
set -Eeo pipefail

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

RUN_ID=${RUN_ID:-pr6_d435i_visual_$(date +%Y%m%d_%H%M%S)}
RUN_DIR=${RUN_DIR:-$WS_ROOT/logs/pr6_d435i_visual/$RUN_ID}
ACTIVE_FILE=${ACTIVE_FILE:-$WS_ROOT/logs/d435i_visual_slam/.active_headless}
RUN_SMALL_RECTANGLE=${RUN_SMALL_RECTANGLE:-0}
EXIT_AFTER_RECTANGLE=${EXIT_AFTER_RECTANGLE:-0}
case "$EXIT_AFTER_RECTANGLE" in
  0|1) ;;
  *) printf 'EXIT_AFTER_RECTANGLE must be 0 or 1.\n' >&2; exit 2 ;;
esac
PR6_START_RTABMAP=${PR6_START_RTABMAP:-1}
case "$PR6_START_RTABMAP" in
  0) PR6_START_RTABMAP_BOOL=false ;;
  1) PR6_START_RTABMAP_BOOL=true ;;
  *) printf 'PR6_START_RTABMAP must be 0 or 1.\n' >&2; exit 2 ;;
esac
mkdir -p "$RUN_DIR" "$(dirname "$ACTIVE_FILE")"

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
cleanup() {
  local status=$?
  [[ "$cleanup_started" == 0 ]] || return
  cleanup_started=1
  trap - EXIT INT TERM
  d435i_cleanup_run_manifests "$RUN_DIR" "$WS_ROOT" \
    "$RUN_DIR/process_cleanup.log"
  d435i_active_remove_owned "$ACTIVE_FILE" "$$" "$RUN_TOKEN" || true
  printf 'PR #6 D435i stack stopped (status=%s). Logs: %s\n' "$status" "$RUN_DIR"
  exit "$status"
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic=$1 timeout_s=${2:-90} started=$SECONDS
  while (( SECONDS - started < timeout_s )); do
    if timeout 5s ros2 topic echo "$topic" --no-daemon --spin-time 2.0 \
        --once --qos-reliability best_effort >/dev/null 2>&1; then
      printf 'ready: %s\n' "$topic"
      return 0
    fi
    sleep 1
  done
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

wait_for_valid_vision() {
  local timeout_s=${1:-60} started=$SECONDS sample=
  while (( SECONDS - started < timeout_s )); do
    sample=$(timeout 5s ros2 topic echo /reliability/vision_score \
      --no-daemon --spin-time 2.0 --once --field valid \
      --qos-reliability best_effort 2>/dev/null || true)
    if grep -qi '^true$' <<<"$sample"; then
      printf 'ready: D_V_rgbd valid\n'
      return 0
    fi
    sleep 1
  done
  printf 'Timed out waiting for valid D_V_rgbd\n' >&2
  return 1
}

printf 'Starting PR #6 sensor stack. Logs: %s\n' "$RUN_DIR"
setsid env \
  HEADLESS=1 GAZEBO_GUI=0 SHOW_FLOW_WINDOW=0 \
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

# The latest mainline sensor supervisor owns the single Gazebo-to-ROS clock
# bridge and validates that it advances. Reuse it here to avoid a duplicate
# publisher and subscribe with the offered best-effort clock QoS.
wait_for_topic /clock 90
wait_for_topic /livox/lidar 120
wait_for_topic /mavros/imu/data_raw 120
wait_for_topic /sim/optical_flow/rad 90
wait_for_topic /mavros/global_position/raw/fix 120

setsid env \
  LOG_DIR="$RUN_DIR/fastlio" RVIZ=0 LIDAR_WS="$LIDAR_WS" \
  FASTLIO_INPUT_MODE=livox START_LIVOX_POINTCLOUD_BRIDGE=0 \
  FASTLIO_NATIVE_FACTOR_EXPORT=1 \
  bash "$PKG_SHARE/scripts/run_mid360_fastlio_mapping.sh" \
  >"$RUN_DIR/fastlio_supervisor.log" 2>&1 &
record_pid fastlio_supervisor "$!"
wait_for_topic /Odometry 120

BACKEND_INPUT_TRIGGER=native_factor
BACKEND_NATIVE_FACTOR=true
BACKEND_LIO_FALLBACK=false
BACKEND_IMU_FACTOR=true
INTEGRATION_START_BACKEND=true
if ! timeout 15s ros2 topic echo /fast_lio/native_lidar_factor \
    --no-daemon --spin-time 2.0 --once --qos-reliability best_effort \
    >/dev/null 2>&1; then
  wait_for_topic /Odometry 30
  BACKEND_INPUT_TRIGGER=lio_pair
  BACKEND_NATIVE_FACTOR=false
  BACKEND_LIO_FALLBACK=true
  # FAST-LIO /Odometry already contains its internal IMU update. Do not add
  # the backend IMU factor again in this explicit dependency fallback.
  BACKEND_IMU_FACTOR=false
  INTEGRATION_START_BACKEND=false
fi
printf 'input_trigger=%s\nnative_factor=%s\nlio_pose_fallback=%s\nimu_factor=%s\n' \
  "$BACKEND_INPUT_TRIGGER" "$BACKEND_NATIVE_FACTOR" \
  "$BACKEND_LIO_FALLBACK" "$BACKEND_IMU_FACTOR" \
  >"$RUN_DIR/backend_runtime_mode.env"

setsid ros2 launch multi_slam_uav_sim pr6_d435i_visual_integration.launch.py \
  use_sim_time:=true start_backend:="$INTEGRATION_START_BACKEND" \
  start_rtabmap:="$PR6_START_RTABMAP_BOOL" \
  database_path:="$RUN_DIR/rtabmap.db" \
  >"$RUN_DIR/integration_overlay.log" 2>&1 &
record_pid integration_overlay "$!"

if [[ "$INTEGRATION_START_BACKEND" == false ]]; then
  setsid ros2 launch uf_lio_adapter lio_adapter.launch.py \
    use_sim_time:=true prefer_native_factor_diagnostics:=false \
    >"$RUN_DIR/lio_adapter_fallback.log" 2>&1 &
  record_pid lio_adapter_fallback "$!"
  setsid ros2 launch uf_backend_fusion online_backend_visual.launch.py \
    preserve_lio_anchor:=true input_trigger_mode:="$BACKEND_INPUT_TRIGGER" \
    native_lidar_factor_enabled:="$BACKEND_NATIVE_FACTOR" \
    allow_lio_pose_fallback:="$BACKEND_LIO_FALLBACK" \
    imu_factor_enabled:="$BACKEND_IMU_FACTOR" \
    >"$RUN_DIR/backend_fallback.log" 2>&1 &
  record_pid backend_fallback "$!"
fi

wait_for_topic /sensors/rgbd/color 90
wait_for_topic /sensors/rgbd/depth 45
wait_for_topic /front/d435i/color/camera_info 45
wait_for_topic /reliability/vision_score 45
if [[ "$PR6_START_RTABMAP" == 1 ]]; then
  wait_for_topic /rtabmap/odom 120
fi
wait_for_publisher /reliability/scheduler_state 45
wait_for_topic /fusion/unified/odom 120

if [[ "$RUN_SMALL_RECTANGLE" == 1 ]]; then
  setsid ros2 run multi_slam_uav_sim guided_rectangle_waypoints --ros-args \
    -p takeoff_alt:=3.0 -p length_x:=6.0 -p length_y:=4.0 \
    -p speed_mps:=0.8 -p land_at_end:=true \
    >"$RUN_DIR/small_rectangle.log" 2>&1 &
  RECTANGLE_PID=$!
  record_pid rectangle_motion "$RECTANGLE_PID"
fi
if [[ "$PR6_START_RTABMAP" == 1 && "$RUN_SMALL_RECTANGLE" == 1 ]]; then
  # RTAB odometry can require initial camera motion before its first valid
  # increment.  Start the optional mission first so validation does not
  # deadlock waiting for motion that the wrapper has not yet launched.
  wait_for_valid_vision 120
fi

printf 'PR #6 + D435i visual integration is ready.\n'
printf '  RTAB startup: %s\n' "$PR6_START_RTABMAP_BOOL"
printf '  RGB-D: /sensors/rgbd/{color,depth}\n'
printf '  RTAB odom: /rtabmap/odom\n'
printf '  D_V_rgbd: /reliability/vision_score\n'
printf '  FAST-LIO: /fast_lio/native_lidar_factor\n'
printf '  Unified backend: /fusion/unified/odom\n'
printf '  Logs: %s\n' "$RUN_DIR"
if [[ "$RUN_SMALL_RECTANGLE" == 1 && "$EXIT_AFTER_RECTANGLE" == 1 ]]; then
  set +e
  wait "$RECTANGLE_PID"
  rectangle_status=$?
  set -e
  printf 'small_rectangle_exit=%s\n' "$rectangle_status" \
    >"$RUN_DIR/rectangle_result.env"
  exit "$rectangle_status"
fi
wait
