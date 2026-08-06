#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
USE_SIM_TIME=${USE_SIM_TIME:-true}
LOG_DIR=${LOG_DIR:-$WS_ROOT/logs/mid360_fastlio_mapping_$(date +%Y%m%d_%H%M%S)}
RVIZ=${RVIZ:-1}
# FAST-LIO requires per-point timing for MID360 de-skewing. The livox mode
# consumes the CustomMsg produced by the protocol-compatible bridge and is the
# default for simulation and hardware-aligned tests.
FASTLIO_INPUT_MODE=${FASTLIO_INPUT_MODE:-livox}
FASTLIO_CLOUD_TOPIC=/sim/mid360/points_raw
FASTLIO_NATIVE_FACTOR_EXPORT=${FASTLIO_NATIVE_FACTOR_EXPORT:-0}
FASTLIO_NATIVE_FACTOR_TOPIC=${FASTLIO_NATIVE_FACTOR_TOPIC:-/fast_lio/native_lidar_factor}
FASTLIO_NATIVE_FACTOR_SENSOR_FRAME=${FASTLIO_NATIVE_FACTOR_SENSOR_FRAME:-mid360_link}
FASTLIO_DOWNSTREAM_BACKEND=${FASTLIO_DOWNSTREAM_BACKEND:-0}
FASTLIO_DIAGNOSTIC_ODOMETRY=${FASTLIO_DIAGNOSTIC_ODOMETRY:-0}
FASTLIO_DIAGNOSTIC_PATH=${FASTLIO_DIAGNOSTIC_PATH:-0}
FASTLIO_DIAGNOSTIC_TF=${FASTLIO_DIAGNOSTIC_TF:-0}
FASTLIO_MAP_INSERTION_MODE=${FASTLIO_MAP_INSERTION_MODE:-fast_lio_posterior}
FASTLIO_BACKEND_STATE_TOPIC=${FASTLIO_BACKEND_STATE_TOPIC:-/fusion/unified/map_pose}
FASTLIO_BACKEND_ACTIVATION_STATE_TOPIC=${FASTLIO_BACKEND_ACTIVATION_STATE_TOPIC:-/fusion/unified/odom}
FASTLIO_STATE_SEED_MODE=${FASTLIO_STATE_SEED_MODE:-disabled}
FASTLIO_STATE_SEED_TOPIC=${FASTLIO_STATE_SEED_TOPIC:-/fusion/unified/frontend_state_seed}
FASTLIO_STATE_SEED_TOLERANCE_S=${FASTLIO_STATE_SEED_TOLERANCE_S:-0.015}
FASTLIO_STATE_SEED_MINIMUM_QUALITY=${FASTLIO_STATE_SEED_MINIMUM_QUALITY:-20}
FASTLIO_STATE_SEED_HISTORY_SIZE=${FASTLIO_STATE_SEED_HISTORY_SIZE:-64}
FASTLIO_STATE_SEED_MAX_TRANSLATION_M=${FASTLIO_STATE_SEED_MAX_TRANSLATION_M:-1.0}
FASTLIO_STATE_SEED_MAX_ROTATION_RAD=${FASTLIO_STATE_SEED_MAX_ROTATION_RAD:-0.50}
FASTLIO_BACKEND_TRAJECTORY_FRONTEND=${FASTLIO_BACKEND_TRAJECTORY_FRONTEND:-0}
FASTLIO_BACKEND_TRAJECTORY_TOPIC=${FASTLIO_BACKEND_TRAJECTORY_TOPIC:-/fusion/unified/backend_deskew_trajectory}
FASTLIO_FRONTEND_SCAN_REQUEST_TOPIC=${FASTLIO_FRONTEND_SCAN_REQUEST_TOPIC:-/fast_lio/frontend_scan_request}
FASTLIO_FRONTEND_SCAN_REQUEST_RETRY_S=${FASTLIO_FRONTEND_SCAN_REQUEST_RETRY_S:-0.20}
FASTLIO_FRONTEND_SCAN_REQUEST_TIMEOUT_S=${FASTLIO_FRONTEND_SCAN_REQUEST_TIMEOUT_S:-2.0}
FASTLIO_NATIVE_FACTOR_EXPORT_BOOL=false
case "$FASTLIO_NATIVE_FACTOR_EXPORT" in
  1|true|TRUE|yes|YES) FASTLIO_NATIVE_FACTOR_EXPORT_BOOL=true ;;
  0|false|FALSE|no|NO) FASTLIO_NATIVE_FACTOR_EXPORT_BOOL=false ;;
  *)
    printf 'Unsupported FASTLIO_NATIVE_FACTOR_EXPORT=%s. Use 0/1 or true/false.\n' "$FASTLIO_NATIVE_FACTOR_EXPORT" >&2
    exit 2
    ;;
esac

parse_bool() {
  case "$1" in
    1|true|TRUE|yes|YES) printf true ;;
    0|false|FALSE|no|NO) printf false ;;
    *)
      printf 'Unsupported boolean value: %s\n' "$1" >&2
      return 2
      ;;
  esac
}
FASTLIO_DOWNSTREAM_BACKEND_BOOL=$(parse_bool "$FASTLIO_DOWNSTREAM_BACKEND")
FASTLIO_DIAGNOSTIC_ODOMETRY_BOOL=$(parse_bool "$FASTLIO_DIAGNOSTIC_ODOMETRY")
FASTLIO_DIAGNOSTIC_PATH_BOOL=$(parse_bool "$FASTLIO_DIAGNOSTIC_PATH")
FASTLIO_DIAGNOSTIC_TF_BOOL=$(parse_bool "$FASTLIO_DIAGNOSTIC_TF")
FASTLIO_BACKEND_TRAJECTORY_FRONTEND_BOOL=$(parse_bool "$FASTLIO_BACKEND_TRAJECTORY_FRONTEND")
case "$FASTLIO_MAP_INSERTION_MODE" in
  fast_lio_posterior|backend_confirmed) ;;
  *)
    printf 'Unsupported FASTLIO_MAP_INSERTION_MODE=%s.\n' \
      "$FASTLIO_MAP_INSERTION_MODE" >&2
    exit 2
    ;;
esac
case "$FASTLIO_STATE_SEED_MODE" in
  disabled|shadow|backend_seeded) ;;
  *)
    printf 'Unsupported FASTLIO_STATE_SEED_MODE=%s. Use disabled, shadow, or backend_seeded.\n' \
      "$FASTLIO_STATE_SEED_MODE" >&2
    exit 2
    ;;
esac
if [[ "$FASTLIO_STATE_SEED_MODE" != "disabled" && \
      "$FASTLIO_DOWNSTREAM_BACKEND_BOOL" != "true" ]]; then
  printf '%s requires FASTLIO_DOWNSTREAM_BACKEND=1.\n' \
    "$FASTLIO_STATE_SEED_MODE" >&2
  exit 2
fi
if [[ "$FASTLIO_STATE_SEED_MODE" == "backend_seeded" && \
      "$FASTLIO_MAP_INSERTION_MODE" != "backend_confirmed" ]]; then
  printf 'backend_seeded requires FASTLIO_MAP_INSERTION_MODE=backend_confirmed.\n' >&2
  exit 2
fi
if [[ "$FASTLIO_BACKEND_TRAJECTORY_FRONTEND_BOOL" == "true" ]]; then
  if [[ "$FASTLIO_NATIVE_FACTOR_EXPORT_BOOL" != "true" || \
        "$FASTLIO_DOWNSTREAM_BACKEND_BOOL" != "true" || \
        "$FASTLIO_MAP_INSERTION_MODE" != "backend_confirmed" ]]; then
    printf 'Backend trajectory mode requires native factor export, downstream backend, and backend_confirmed map insertion.\n' >&2
    exit 2
  fi
fi

source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
if [[ ! -f "$LIDAR_WS/install/setup.bash" ]]; then
  cat <<EOF
MID360 / FAST-LIO workspace was not found:
  $LIDAR_WS/install/setup.bash

Run the repository installer first. It creates the workspace containing
fast_lio and livox_ros_driver2 at the default path below.

Example:
  bash $WS_ROOT/tools/setup_ubuntu.sh
EOF
  exit 2
fi
source "$LIDAR_WS/install/setup.bash"

START_LIVOX_POINTCLOUD_BRIDGE=${START_LIVOX_POINTCLOUD_BRIDGE:-auto}
LIVOX_INPUT_WAIT_S=${LIVOX_INPUT_WAIT_S:-90}
LIVOX_INPUT_CHECK_PERIOD_S=${LIVOX_INPUT_CHECK_PERIOD_S:-1}
LIVOX_INPUT_MISSED_CHECKS=${LIVOX_INPUT_MISSED_CHECKS:-5}

topic_publisher_count() {
  local topic=$1
  local info count
  info=$(timeout 5s ros2 topic info --no-daemon --spin-time 1.0 "$topic" 2>/dev/null || true)
  count=$(sed -n 's/^Publisher count: \([0-9][0-9]*\)$/\1/p' <<<"$info")
  printf '%s' "${count:-0}"
}

if [[ "$START_LIVOX_POINTCLOUD_BRIDGE" == "auto" ]]; then
  # `livox` is the hardware-compatible CustomMsg path.  The simulator owns
  # this path through the C++ Gazebo bridge, which may start several seconds
  # after this wrapper.  A one-shot publisher check races that startup and can
  # launch the legacy Python adapter as a second /livox/imu publisher.  That
  # interleaves two timestamp histories and makes FAST-LIO clear its buffers.
  # Keep the Python bridge an explicit opt-in for pointcloud/legacy tests.
  if [[ "$FASTLIO_INPUT_MODE" == "livox" ]]; then
    START_LIVOX_POINTCLOUD_BRIDGE=0
  else
    START_LIVOX_POINTCLOUD_BRIDGE=1
  fi
fi
case "$START_LIVOX_POINTCLOUD_BRIDGE" in
  0|1) ;;
  *)
    printf 'START_LIVOX_POINTCLOUD_BRIDGE must be auto, 0, or 1.\n' >&2
    exit 2
    ;;
esac

mkdir -p "$LOG_DIR"
pids=()
cleanup() {
  printf '\nStopping MID360 FAST-LIO mapping...\n'
  for pid in "${pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]:-}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    kill -KILL "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

cat <<EOF
MID360 FAST-LIO mapping starting.

Logs:
  $LOG_DIR/livox_mid360_bridge.log
  $LOG_DIR/fast_lio.log
  $LOG_DIR/fastlio_cloud_mapper.log
  $LOG_DIR/fastlio_occupancy_grid.log

Inputs:
  $FASTLIO_CLOUD_TOPIC  -> /livox/lidar  (livox_ros_driver2/msg/CustomMsg, protocol reference)
  /mavros/imu/data_raw    -> /livox/imu    (FCU HIGHRES_IMU sensor_msgs/msg/Imu)
  FASTLIO_INPUT_MODE=$FASTLIO_INPUT_MODE
  START_LIVOX_POINTCLOUD_BRIDGE=$START_LIVOX_POINTCLOUD_BRIDGE

Outputs:
  /cloud_registered
  /Odometry
  /cloud_registered_reliable
  /fastlio_denoised_map
  /fastlio_occupancy_grid
  native factor export: $FASTLIO_NATIVE_FACTOR_EXPORT -> $FASTLIO_NATIVE_FACTOR_TOPIC
  downstream backend mode: $FASTLIO_DOWNSTREAM_BACKEND
  map insertion mode: $FASTLIO_MAP_INSERTION_MODE
  backend map pose topic: $FASTLIO_BACKEND_STATE_TOPIC
  backend trajectory activation topic: $FASTLIO_BACKEND_ACTIVATION_STATE_TOPIC
  frontend state seed: mode=$FASTLIO_STATE_SEED_MODE topic=$FASTLIO_STATE_SEED_TOPIC
  backend trajectory frontend: $FASTLIO_BACKEND_TRAJECTORY_FRONTEND
  diagnostic pose outputs: odom=$FASTLIO_DIAGNOSTIC_ODOMETRY path=$FASTLIO_DIAGNOSTIC_PATH tf=$FASTLIO_DIAGNOSTIC_TF

EOF

case "$FASTLIO_INPUT_MODE" in
  livox)
    FASTLIO_CONFIG=fast_lio_sim_mid360.yaml
    ;;
  pointcloud)
    FASTLIO_CONFIG=fast_lio_sim_mid360_pointcloud.yaml
    ;;
  filtered_pointcloud)
    FASTLIO_CONFIG=fast_lio_sim_mid360_filtered_pointcloud.yaml
    FASTLIO_CLOUD_TOPIC=/sensors/lidar/points
    ;;
  *)
    printf 'Unsupported FASTLIO_INPUT_MODE=%s. Use pointcloud, filtered_pointcloud, or livox.\n' "$FASTLIO_INPUT_MODE" >&2
    exit 2
    ;;
esac

if [[ "$START_LIVOX_POINTCLOUD_BRIDGE" == "1" ]]; then
  setsid ros2 run multi_slam_uav_sim livox_mid360_bridge --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p input_cloud_topic:="$FASTLIO_CLOUD_TOPIC" \
    -p input_imu_topic:=/mavros/imu/data_raw \
    -p livox_lidar_topic:=/livox/lidar \
    -p livox_imu_topic:=/livox/imu \
    -p lidar_frame_id:=mid360_link \
    -p imu_frame_id:=base_link \
    -p scan_lines:=4 \
    -p frame_rate_hz:=10.0 \
    -p max_points:=20000 \
    -p point_stride:=${MID360_POINT_STRIDE:-1} \
    >"$LOG_DIR/livox_mid360_bridge.log" 2>&1 &
  pids+=("$!")
else
  printf 'Using existing /livox/lidar and /livox/imu publishers; Python bridge is disabled.\n' \
    >"$LOG_DIR/livox_mid360_bridge.log"
fi

# FAST-LIO assumes one ordered LiDAR stream and one ordered IMU stream.  Wait
# for the selected adapter to become ready, then continuously enforce that
# ownership contract while FAST-LIO is running.
livox_lidar_publishers=0
livox_imu_publishers=0
wait_attempts=$(( LIVOX_INPUT_WAIT_S * 2 ))
for (( _attempt=1; _attempt<=wait_attempts; _attempt++ )); do
  livox_lidar_publishers=$(topic_publisher_count /livox/lidar)
  livox_imu_publishers=$(topic_publisher_count /livox/imu)
  if (( livox_lidar_publishers == 1 && livox_imu_publishers == 1 )); then
    break
  fi
  sleep 0.5
done
if (( livox_lidar_publishers != 1 || livox_imu_publishers != 1 )); then
  printf 'FAST-LIO input ownership error: /livox/lidar publishers=%s, /livox/imu publishers=%s; expected exactly one each.\n' \
    "$livox_lidar_publishers" "$livox_imu_publishers" >&2
  printf 'Stop duplicate Livox adapters, or set START_LIVOX_POINTCLOUD_BRIDGE=0 when the simulator/driver already provides /livox/*.\n' >&2
  exit 3
fi

setsid ros2 launch fast_lio mapping.launch.py \
  use_sim_time:="$USE_SIM_TIME" \
  config_path:="$PKG_SHARE/config" \
  config_file:="$FASTLIO_CONFIG" \
  rviz:="$RVIZ" \
  native_factor_export_enable:="$FASTLIO_NATIVE_FACTOR_EXPORT_BOOL" \
  native_factor_export_topic:="$FASTLIO_NATIVE_FACTOR_TOPIC" \
  native_factor_sensor_frame:="$FASTLIO_NATIVE_FACTOR_SENSOR_FRAME" \
  downstream_backend_enable:="$FASTLIO_DOWNSTREAM_BACKEND_BOOL" \
  downstream_publish_diagnostic_odometry:="$FASTLIO_DIAGNOSTIC_ODOMETRY_BOOL" \
  downstream_publish_diagnostic_path:="$FASTLIO_DIAGNOSTIC_PATH_BOOL" \
  downstream_publish_diagnostic_tf:="$FASTLIO_DIAGNOSTIC_TF_BOOL" \
  downstream_map_insertion_mode:="$FASTLIO_MAP_INSERTION_MODE" \
  downstream_backend_state_topic:="$FASTLIO_BACKEND_STATE_TOPIC" \
  downstream_backend_activation_state_topic:="$FASTLIO_BACKEND_ACTIVATION_STATE_TOPIC" \
  frontend_state_seed_mode:="$FASTLIO_STATE_SEED_MODE" \
  frontend_state_seed_topic:="$FASTLIO_STATE_SEED_TOPIC" \
  frontend_state_seed_tolerance_s:="$FASTLIO_STATE_SEED_TOLERANCE_S" \
  frontend_state_seed_minimum_quality:="$FASTLIO_STATE_SEED_MINIMUM_QUALITY" \
  frontend_state_seed_history_size:="$FASTLIO_STATE_SEED_HISTORY_SIZE" \
  frontend_state_seed_max_translation_correction_m:="$FASTLIO_STATE_SEED_MAX_TRANSLATION_M" \
  frontend_state_seed_max_rotation_correction_rad:="$FASTLIO_STATE_SEED_MAX_ROTATION_RAD" \
  backend_trajectory_frontend_enable:="$FASTLIO_BACKEND_TRAJECTORY_FRONTEND_BOOL" \
  backend_trajectory_frontend_topic:="$FASTLIO_BACKEND_TRAJECTORY_TOPIC" \
  frontend_scan_request_topic:="$FASTLIO_FRONTEND_SCAN_REQUEST_TOPIC" \
  frontend_scan_request_retry_period_s:="$FASTLIO_FRONTEND_SCAN_REQUEST_RETRY_S" \
  frontend_scan_request_timeout_s:="$FASTLIO_FRONTEND_SCAN_REQUEST_TIMEOUT_S" \
  >"$LOG_DIR/fast_lio.log" 2>&1 &
fastlio_pid="$!"
pids+=("$fastlio_pid")

monitor_livox_ownership() {
  local ownership_misses=0
  while kill -0 "$fastlio_pid" 2>/dev/null; do
    local lidar_count imu_count
    lidar_count=$(topic_publisher_count /livox/lidar)
    imu_count=$(topic_publisher_count /livox/imu)
    if (( lidar_count != 1 || imu_count != 1 )); then
      ownership_misses=$((ownership_misses + 1))
      printf 'FAST-LIO input ownership check %s/%s: /livox/lidar publishers=%s, /livox/imu publishers=%s.\n' \
        "$ownership_misses" "$LIVOX_INPUT_MISSED_CHECKS" "$lidar_count" "$imu_count" \
        >>"$LOG_DIR/livox_mid360_bridge.log"
      if (( ownership_misses >= LIVOX_INPUT_MISSED_CHECKS )); then
        printf 'FAST-LIO input ownership lost continuously: expected exactly one publisher for each topic.\n' \
          | tee -a "$LOG_DIR/livox_mid360_bridge.log" >&2
        kill -TERM "$fastlio_pid" 2>/dev/null || true
        return 3
      fi
    else
      ownership_misses=0
    fi
    sleep "$LIVOX_INPUT_CHECK_PERIOD_S"
  done
}
monitor_livox_ownership &
pids+=("$!")

sleep 3

setsid ros2 run mid360_reliable_mapper fastlio_cloud_mapper_node --ros-args \
  -p use_sim_time:="$USE_SIM_TIME" \
  --params-file "$PKG_SHARE/config/sim_fastlio_reliable_mapping_params.yaml" \
  >"$LOG_DIR/fastlio_cloud_mapper.log" 2>&1 &
pids+=("$!")

setsid ros2 run mid360_reliable_mapper pointcloud_occupancy_grid_node --ros-args \
  -p use_sim_time:="$USE_SIM_TIME" \
  --params-file "$PKG_SHARE/config/sim_fastlio_reliable_mapping_params.yaml" \
  >"$LOG_DIR/fastlio_occupancy_grid.log" 2>&1 &
pids+=("$!")

wait
