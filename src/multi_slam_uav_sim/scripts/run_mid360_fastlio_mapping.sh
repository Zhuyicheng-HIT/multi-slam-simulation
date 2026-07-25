#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
LOG_DIR=${LOG_DIR:-$WS_ROOT/logs/mid360_fastlio_mapping_$(date +%Y%m%d_%H%M%S)}
RVIZ=${RVIZ:-1}
FASTLIO_INPUT_MODE=${FASTLIO_INPUT_MODE:-pointcloud}
FASTLIO_CLOUD_TOPIC=/sim/mid360/points_raw

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

Outputs:
  /cloud_registered
  /Odometry
  /cloud_registered_reliable
  /fastlio_denoised_map
  /fastlio_occupancy_grid

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

setsid ros2 run multi_slam_uav_sim livox_mid360_bridge --ros-args \
  -p input_cloud_topic:="$FASTLIO_CLOUD_TOPIC" \
  -p input_imu_topic:=/mavros/imu/data_raw \
  -p livox_lidar_topic:=/livox/lidar \
  -p livox_imu_topic:=/livox/imu \
  -p lidar_frame_id:=mid360_link \
  -p imu_frame_id:=base_link \
  -p scan_lines:=40 \
  -p frame_rate_hz:=10.0 \
  -p vertical_min_deg:=-7.0 \
  -p vertical_max_deg:=52.0 \
  -p max_points:=20000 \
  >"$LOG_DIR/livox_mid360_bridge.log" 2>&1 &
pids+=("$!")

setsid ros2 launch fast_lio mapping.launch.py \
  use_sim_time:=false \
  config_path:="$PKG_SHARE/config" \
  config_file:="$FASTLIO_CONFIG" \
  rviz:="$RVIZ" \
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
