#!/usr/bin/env bash
set -Eeuo pipefail

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

wait_for_topic /clock 45
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
wait_for_topic /fast_lio/native_lidar_factor 120

setsid ros2 launch multi_slam_uav_sim pr6_d435i_visual_integration.launch.py \
  use_sim_time:=true start_backend:=true \
  database_path:="$RUN_DIR/rtabmap.db" \
  >"$RUN_DIR/integration_overlay.log" 2>&1 &
record_pid integration_overlay "$!"

wait_for_topic /sensors/rgbd/color 90
wait_for_topic /sensors/rgbd/depth 45
wait_for_topic /front/d435i/color/camera_info 45
wait_for_topic /rtabmap/odom 120
wait_for_topic /reliability/vision_score 45
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
