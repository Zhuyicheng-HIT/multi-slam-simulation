#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
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
FASTLIO_NATIVE_FACTOR_EXPORT_BOOL=false
case "$FASTLIO_NATIVE_FACTOR_EXPORT" in
  1|true|TRUE|yes|YES) FASTLIO_NATIVE_FACTOR_EXPORT_BOOL=true ;;
  0|false|FALSE|no|NO) FASTLIO_NATIVE_FACTOR_EXPORT_BOOL=false ;;
  *)
    printf 'Unsupported FASTLIO_NATIVE_FACTOR_EXPORT=%s. Use 0/1 or true/false.\n' "$FASTLIO_NATIVE_FACTOR_EXPORT" >&2
    exit 2
    ;;
esac

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
if [[ "$START_LIVOX_POINTCLOUD_BRIDGE" == "auto" ]]; then
  if [[ "$FASTLIO_INPUT_MODE" == "livox" ]] \
      && ros2 topic info /livox/lidar 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
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
    kill "$pid" 2>/dev/null || true
    kill -- "-$pid" 2>/dev/null || true
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

setsid ros2 launch fast_lio mapping.launch.py \
  use_sim_time:=false \
  config_path:="$PKG_SHARE/config" \
  config_file:="$FASTLIO_CONFIG" \
  rviz:="$RVIZ" \
  native_factor_export_enable:="$FASTLIO_NATIVE_FACTOR_EXPORT_BOOL" \
  native_factor_export_topic:="$FASTLIO_NATIVE_FACTOR_TOPIC" \
  native_factor_sensor_frame:="$FASTLIO_NATIVE_FACTOR_SENSOR_FRAME" \
  >"$LOG_DIR/fast_lio.log" 2>&1 &
pids+=("$!")

sleep 3

setsid ros2 run mid360_reliable_mapper fastlio_cloud_mapper_node --ros-args \
  --params-file "$PKG_SHARE/config/sim_fastlio_reliable_mapping_params.yaml" \
  >"$LOG_DIR/fastlio_cloud_mapper.log" 2>&1 &
pids+=("$!")

setsid ros2 run mid360_reliable_mapper pointcloud_occupancy_grid_node --ros-args \
  --params-file "$PKG_SHARE/config/sim_fastlio_reliable_mapping_params.yaml" \
  >"$LOG_DIR/fastlio_occupancy_grid.log" 2>&1 &
pids+=("$!")

wait
