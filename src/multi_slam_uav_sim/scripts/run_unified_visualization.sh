#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
USE_SIM_TIME=${USE_SIM_TIME:-true}
RVIZ_CONFIG=${RVIZ_CONFIG:-"$PKG_SHARE/config/ultra_fusion_demo.rviz"}

# ROS 2 Humble setup scripts probe optional variables that may be unset.
set +u
source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
set -u

if [[ ! -f "$RVIZ_CONFIG" ]]; then
  printf 'RViz configuration not found: %s\n' "$RVIZ_CONFIG" >&2
  exit 2
fi

cat <<'EOF'
Ultra-Fusion visualization starting.

Display legend:
  cyan    Ultra-Fusion unified trajectory
  orange  FAST-LIO diagnostic trajectory
  green   temporally stable LiDAR points
  red     dynamic LiDAR points
  yellow  uncertain LiDAR points

Ground truth is disabled by default and is evaluation-only.
EOF

for topic in \
  /cloud_registered \
  /fastlio_denoised_map \
  /fusion/unified/odom \
  /fusion/unified/path; do
  if ros2 topic info "$topic" 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
    printf '  ready:   %s\n' "$topic"
  else
    printf '  waiting: %s (the display will connect when its publisher starts)\n' "$topic"
  fi
done

exec rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:="$USE_SIM_TIME"
