#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
set -u

ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_visual_slam.active}
if [[ ! -f "$ACTIVE_FILE" ]]; then
  printf 'Start run_d435i_visual_slam_headless.sh first.\n' >&2
  exit 2
fi
read -r wrapper_pid run_dir <"$ACTIVE_FILE"
if ! kill -0 "$wrapper_pid" 2>/dev/null; then
  printf 'The recorded headless baseline is not running.\n' >&2
  exit 2
fi

for topic in /mavros/state /mavros/local_position/pose /rtabmap/odom; do
  if ! timeout 8s ros2 topic echo "$topic" --once >/dev/null 2>&1; then
    printf 'Required topic has no data: %s\n' "$topic" >&2
    exit 2
  fi
done

LOG_DIR=${LOG_DIR:-$run_dir/visual_friendly_flight_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOG_DIR"

INITIAL_TAKEOFF_ALT=${INITIAL_TAKEOFF_ALT:-0.25}
CLIMB_HEIGHT=${CLIMB_HEIGHT:-0.50}
VERTICAL_SPEED=${VERTICAL_SPEED:-0.10}
HORIZONTAL_DISTANCE=${HORIZONTAL_DISTANCE:-0.50}
HORIZONTAL_SPEED=${HORIZONTAL_SPEED:-0.10}
GROUND_STATIC_S=${GROUND_STATIC_S:-10.0}
VISUAL_HOLD_S=${VISUAL_HOLD_S:-5.0}

cat <<EOF | tee "$LOG_DIR/parameters.txt"
navigation_source=gps
initial_takeoff_alt=$INITIAL_TAKEOFF_ALT
climb_height=$CLIMB_HEIGHT
vertical_speed=$VERTICAL_SPEED
horizontal_distance=$HORIZONTAL_DISTANCE
horizontal_speed=$HORIZONTAL_SPEED
ground_static_s=$GROUND_STATIC_S
visual_hold_s=$VISUAL_HOLD_S
RTAB-Map is evaluation-only and does not control this flight.
EOF

set +e
ros2 run multi_slam_uav_sim visual_friendly_flight --ros-args \
  -p navigation_source:=gps \
  -p takeoff_alt:="$INITIAL_TAKEOFF_ALT" \
  -p takeoff_min_alt_m:=0.15 \
  -p takeoff_free_climb_s:=14.0 \
  -p ground_static_s:="$GROUND_STATIC_S" \
  -p climb_height_m:="$CLIMB_HEIGHT" \
  -p vertical_speed_mps:="$VERTICAL_SPEED" \
  -p horizontal_distance_m:="$HORIZONTAL_DISTANCE" \
  -p horizontal_speed_mps:="$HORIZONTAL_SPEED" \
  -p visual_hold_s:="$VISUAL_HOLD_S" \
  -p land_at_end:=true \
  2>&1 | tee "$LOG_DIR/visual_friendly_flight.log"
status=${PIPESTATUS[0]}
set -e

exit "$status"
