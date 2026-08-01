#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
source "$PKG_SHARE/scripts/d435i_active_run_lifecycle.sh"

LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
RUN_ID=${RUN_ID:-pr6_d435i_visual_$(date +%Y%m%d_%H%M%S)}
RUN_DIR=${RUN_DIR:-$WS_ROOT/logs/pr6_d435i_visual/$RUN_ID}
ACTIVE_FILE=${ACTIVE_FILE:-$WS_ROOT/logs/d435i_visual_slam/.active_headless}
RUN_SMALL_RECTANGLE=${RUN_SMALL_RECTANGLE:-0}
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
    if timeout 3s ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      printf 'ready: %s\n' "$topic"
      return 0
    fi
    sleep 1
  done
  printf 'Timed out waiting for %s\n' "$topic" >&2
  return 1
}

wait_for_valid_vision() {
  local timeout_s=${1:-60} started=$SECONDS sample=
  while (( SECONDS - started < timeout_s )); do
    sample=$(timeout 3s ros2 topic echo /reliability/vision_score \
      --once --field valid 2>/dev/null || true)
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

# PR #6's sensor supervisor runs Gazebo without a ROS /clock bridge. RTAB uses
# simulation time, so this wrapper owns exactly one clock publisher.
setsid ros2 run ros_gz_bridge parameter_bridge \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  >"$RUN_DIR/clock_bridge.log" 2>&1 &
CLOCK_BRIDGE_PID=$!
record_pid clock_bridge "$CLOCK_BRIDGE_PID"

if ! wait_for_topic /clock 45; then
  # ros_gz_bridge occasionally starts before the fresh Gazebo Transport
  # partition is discoverable.  Restart only this owned bridge once, after the
  # simulator and FCU have had time to become ready.
  kill -INT -- "-$CLOCK_BRIDGE_PID" 2>/dev/null || true
  timeout 5s tail --pid="$CLOCK_BRIDGE_PID" -f /dev/null 2>/dev/null || true
  setsid ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
    >"$RUN_DIR/clock_bridge_retry.log" 2>&1 &
  CLOCK_BRIDGE_PID=$!
  record_pid clock_bridge_retry "$CLOCK_BRIDGE_PID"
  wait_for_topic /clock 45
fi
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
if ! timeout 15s ros2 topic echo /fast_lio/native_lidar_factor --once \
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
wait_for_topic /rtabmap/odom 120
wait_for_topic /reliability/vision_score 45
wait_for_valid_vision 60
wait_for_topic /reliability/scheduler_state 45
wait_for_topic /fusion/unified/odom 120

if [[ "$RUN_SMALL_RECTANGLE" == 1 ]]; then
  setsid ros2 run multi_slam_uav_sim guided_rectangle_waypoints --ros-args \
    -p takeoff_alt:=3.0 -p length_x:=6.0 -p length_y:=4.0 \
    -p speed_mps:=0.8 -p land_at_end:=true \
    >"$RUN_DIR/small_rectangle.log" 2>&1 &
  record_pid rectangle_motion "$!"
fi

printf 'PR #6 + D435i visual integration is ready.\n'
printf '  RGB-D: /sensors/rgbd/{color,depth}\n'
printf '  RTAB odom: /rtabmap/odom\n'
printf '  D_V_rgbd: /reliability/vision_score\n'
printf '  FAST-LIO: /fast_lio/native_lidar_factor\n'
printf '  Unified backend: /fusion/unified/odom\n'
printf '  Logs: %s\n' "$RUN_DIR"
wait
