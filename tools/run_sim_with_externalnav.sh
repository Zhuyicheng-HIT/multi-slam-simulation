#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
TARGET=install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_externalnav.sh

"$SCRIPT_DIR/ensure_built.sh" "$TARGET"
if [[ ! -f "$REPO_ROOT/install/uf_sensor_pipeline/share/uf_sensor_pipeline/launch/gps_flow_externalnav.launch.py" ]]; then
  cd "$REPO_ROOT"
  source /opt/ros/humble/setup.bash
  colcon build --symlink-install --packages-up-to uf_sensor_pipeline
fi
exec bash "$REPO_ROOT/$TARGET" "$@"
