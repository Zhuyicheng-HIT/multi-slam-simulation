#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
OUTPUT=${1:-$REPO_ROOT/bags/uf_sensor_$(date +%Y%m%d_%H%M%S)}
QOS_FILE="$REPO_ROOT/src/ultra_fusion_nav/uf_sensor_pipeline/config/rosbag_qos_overrides.yaml"

set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u
mkdir -p "$(dirname "$OUTPUT")"

exec ros2 bag record --storage sqlite3 --output "$OUTPUT" \
  --qos-profile-overrides-path "$QOS_FILE" \
  /clock /tf /tf_static \
  /sensors/lidar/points /sensors/lidar/body_removed_ratio \
  /sensors/imu /sensors/gnss/fix /sensors/optical_flow/rad \
  /sensors/rgbd/color /sensors/rgbd/depth \
  /fault/state /sensor_contract/diagnostics \
  /Odometry /cloud_registered \
  /lio/odom /lio/path /lio/local_map /lio/diagnostics /lidar/points_deskewed
