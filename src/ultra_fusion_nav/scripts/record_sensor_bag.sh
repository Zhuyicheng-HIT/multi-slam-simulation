#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
OUTPUT=${1:-$REPO_ROOT/bags/uf_sensor_$(date +%Y%m%d_%H%M%S)}
PROFILE=${UF_BAG_PROFILE:-full}
QOS_FILE="$REPO_ROOT/src/ultra_fusion_nav/uf_sensor_pipeline/config/rosbag_qos_overrides.yaml"

set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u
mkdir -p "$(dirname "$OUTPUT")"

case "$PROFILE" in
  nav)
    topics=(
      /clock /tf /tf_static
      /mavros/imu/data
      /mavros/global_position/raw/fix /sim/optical_flow/rad
      /sensors/gnss/fix /sensors/optical_flow/rad
      /fusion/gps_flow/odom /mavros/odometry/out
      /fusion/gps_flow/diagnostics /external_nav/diagnostics
      /reliability/gnss_score /reliability/optical_flow_score
      /fault/state /simulation/performance
    )
    ;;
  full)
    topics=(
      /clock /tf /tf_static
      /sensors/lidar/points /sensors/lidar/body_removed_ratio
      /sensors/imu /sensors/gnss/fix /sensors/optical_flow/rad
      /sensors/rgbd/color /sensors/rgbd/depth
      /fault/state /sensor_contract/diagnostics
      /Odometry /cloud_registered
      /lio/odom /lio/path /lio/local_map /lio/diagnostics /lidar/points_deskewed
    )
    ;;
  *)
    printf 'Unknown UF_BAG_PROFILE=%s (expected nav or full)\n' "$PROFILE" >&2
    exit 2
    ;;
esac

printf 'Recording rosbag2 profile=%s to %s\n' "$PROFILE" "$OUTPUT"
exec ros2 bag record --storage sqlite3 --output "$OUTPUT" \
  --qos-profile-overrides-path "$QOS_FILE" "${topics[@]}"
