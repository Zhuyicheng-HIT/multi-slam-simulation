#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOCK_FILE=${MTF01_LOCK_FILE:-/tmp/multi_slam_mtf01_hardware_bridge.lock}

exec 9>"$LOCK_FILE"
if ! flock -n 9; then
  printf 'An MTF-01 WSL bridge is already running (lock: %s).\n' "$LOCK_FILE" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u

WINDOWS_HOST_IP=${WINDOWS_HOST_IP:-127.0.0.1}
MTF01_TCP_PORT=${MTF01_TCP_PORT:-5764}
MTF01_FLOW_TOPIC=${MTF01_FLOW_TOPIC:-/hardware/mtf01/optical_flow/rad}
MTF01_RANGE_TOPIC=${MTF01_RANGE_TOPIC:-/hardware/mtf01/range}
MTF01_RAW_TOPIC=${MTF01_RAW_TOPIC:-/hardware/mtf01/micolink_frame}
MTF01_IMU_TOPIC=${MTF01_IMU_TOPIC:-/mavros/imu/data_raw}
MTF01_REPORT_PATH=${MTF01_REPORT_PATH:-/tmp/mtf01_hardware_bridge.json}

printf 'MTF-01 TCP source: %s:%s\n' "$WINDOWS_HOST_IP" "$MTF01_TCP_PORT"
printf 'ROS flow output: %s\n' "$MTF01_FLOW_TOPIC"

exec ros2 run multi_slam_uav_sim mtf01_micolink_bridge --ros-args \
  -p mode:=tcp \
  -p tcp_host:="$WINDOWS_HOST_IP" \
  -p tcp_port:="$MTF01_TCP_PORT" \
  -p flow_topic:="$MTF01_FLOW_TOPIC" \
  -p range_topic:="$MTF01_RANGE_TOPIC" \
  -p raw_frame_topic:="$MTF01_RAW_TOPIC" \
  -p imu_topic:="$MTF01_IMU_TOPIC" \
  -p restamp_output:=true \
  -p report_path:="$MTF01_REPORT_PATH"
