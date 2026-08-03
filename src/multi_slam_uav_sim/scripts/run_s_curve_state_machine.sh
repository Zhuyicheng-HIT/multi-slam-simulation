#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)

source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"

if ! timeout 8s bash -lc "ros2 topic list 2>/dev/null | grep -q '^/mavros/state$'"; then
  printf 'MAVROS / FCU topics are not visible. Start the simulation stack first.\n' >&2
  exit 2
fi
clock_ready=false
for _attempt in 1 2 3; do
  if timeout 8s ros2 topic echo /clock --once --field clock \
      --qos-reliability best_effort >/dev/null 2>&1; then
    clock_ready=true
    break
  fi
done
if [[ "$clock_ready" != true ]]; then
  printf 'ROS simulation /clock is unavailable. Start the updated simulation stack first.\n' >&2
  exit 2
fi

LOG_DIR=${LOG_DIR:-$WS_ROOT/logs/s_curve_state_machine_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOG_DIR"

TAKEOFF_ALT=${TAKEOFF_ALT:-5.0}
S_CURVE_SPAN=${S_CURVE_SPAN:-12.0}
S_CURVE_AMPLITUDE=${S_CURVE_AMPLITUDE:-4.5}
S_CURVE_VERTICAL_AMPLITUDE=${S_CURVE_VERTICAL_AMPLITUDE:-1.0}
S_CURVE_VERTICAL_CYCLES=${S_CURVE_VERTICAL_CYCLES:-1}
S_CURVE_PASSES=${S_CURVE_PASSES:-3}
S_CURVE_SPEED=${S_CURVE_SPEED:-0.45}
S_CURVE_HOLD_TIME=${S_CURVE_HOLD_TIME:-3.0}
S_CURVE_WAYPOINT_SPACING=${S_CURVE_WAYPOINT_SPACING:-3.0}
S_CURVE_WAYPOINT_HOLD=${S_CURVE_WAYPOINT_HOLD:-1.0}
S_CURVE_WAYPOINT_TOLERANCE=${S_CURVE_WAYPOINT_TOLERANCE:-0.60}
LOCALIZATION_SAFETY_ENABLED=${LOCALIZATION_SAFETY_ENABLED:-true}
LOCALIZATION_HOLD_S=${LOCALIZATION_HOLD_S:-1.0}
MINIMUM_CLEARANCE_ALT=${MINIMUM_CLEARANCE_ALT:-3.5}
CALIBRATION_YAW_SWEEP_DEG=${CALIBRATION_YAW_SWEEP_DEG:-12.0}
LOCKED_YAW_OFFSET_DEG=${LOCKED_YAW_OFFSET_DEG:-0.0}
LAND_AT_END=${LAND_AT_END:-true}
NAVIGATION_SOURCE=${NAVIGATION_SOURCE:-auto}
MAVLINK_TAKEOFF_URL=${MAVLINK_TAKEOFF_URL:-tcp:127.0.0.1:5763}

# rclpy preserves the CLI scalar type. Normalize integer-looking overrides so
# a user can write S_CURVE_SPAN=12 without turning a declared double into an
# invalid integer parameter; do the same for the boolean landing switch.
as_double() {
  case "$1" in
    ''|*[!0-9.+-]*) printf '%s' "$1" ;;
    *.*|*e*|*E*) printf '%s' "$1" ;;
    *) printf '%s.0' "$1" ;;
  esac
}
TAKEOFF_ALT_ARG=$(as_double "$TAKEOFF_ALT")
S_CURVE_SPAN_ARG=$(as_double "$S_CURVE_SPAN")
S_CURVE_AMPLITUDE_ARG=$(as_double "$S_CURVE_AMPLITUDE")
S_CURVE_VERTICAL_AMPLITUDE_ARG=$(as_double "$S_CURVE_VERTICAL_AMPLITUDE")
S_CURVE_SPEED_ARG=$(as_double "$S_CURVE_SPEED")
S_CURVE_HOLD_TIME_ARG=$(as_double "$S_CURVE_HOLD_TIME")
S_CURVE_WAYPOINT_SPACING_ARG=$(as_double "$S_CURVE_WAYPOINT_SPACING")
S_CURVE_WAYPOINT_HOLD_ARG=$(as_double "$S_CURVE_WAYPOINT_HOLD")
S_CURVE_WAYPOINT_TOLERANCE_ARG=$(as_double "$S_CURVE_WAYPOINT_TOLERANCE")
LOCALIZATION_HOLD_S_ARG=$(as_double "$LOCALIZATION_HOLD_S")
MINIMUM_CLEARANCE_ALT_ARG=$(as_double "$MINIMUM_CLEARANCE_ALT")
CALIBRATION_YAW_SWEEP_DEG_ARG=$(as_double "$CALIBRATION_YAW_SWEEP_DEG")
LOCKED_YAW_OFFSET_DEG_ARG=$(as_double "$LOCKED_YAW_OFFSET_DEG")
case "${LAND_AT_END,,}" in
  1|true|yes|on) LAND_AT_END_ARG=true ;;
  0|false|no|off) LAND_AT_END_ARG=false ;;
  *) printf 'LAND_AT_END must be true/false or 1/0, got %s.\n' "$LAND_AT_END" >&2; exit 2 ;;
esac
case "${LOCALIZATION_SAFETY_ENABLED,,}" in
  1|true|yes|on) LOCALIZATION_SAFETY_ENABLED_ARG=true ;;
  0|false|no|off) LOCALIZATION_SAFETY_ENABLED_ARG=false ;;
  *) printf 'LOCALIZATION_SAFETY_ENABLED must be true/false or 1/0, got %s.\n' "$LOCALIZATION_SAFETY_ENABLED" >&2; exit 2 ;;
esac

cat <<EOF
GUIDED 3D S-curve state machine starting.

Logs: $LOG_DIR
Route: span=$S_CURVE_SPAN m, lateral_amplitude=$S_CURVE_AMPLITUDE m,
       vertical_amplitude=$S_CURVE_VERTICAL_AMPLITUDE m,
       passes=$S_CURVE_PASSES, speed=$S_CURVE_SPEED m/s
Stops: every $S_CURVE_WAYPOINT_SPACING m for at least $S_CURVE_WAYPOINT_HOLD s,
       endpoint hold=$S_CURVE_HOLD_TIME s, tolerance=$S_CURVE_WAYPOINT_TOLERANCE m
Safety: enabled=$LOCALIZATION_SAFETY_ENABLED, minimum localization-loss hold=$LOCALIZATION_HOLD_S s
Yaw: calibration_sweep=$CALIBRATION_YAW_SWEEP_DEG deg, then locked at home+$LOCKED_YAW_OFFSET_DEG deg
Altitude: takeoff=$TAKEOFF_ALT m, minimum_clearance=$MINIMUM_CLEARANCE_ALT m
EOF

set +e
ros2 run multi_slam_uav_sim guided_s_curve_waypoints --ros-args \
  -p use_sim_time:=true \
  -p takeoff_alt:="$TAKEOFF_ALT_ARG" \
  -p longitudinal_span:="$S_CURVE_SPAN_ARG" \
  -p lateral_amplitude:="$S_CURVE_AMPLITUDE_ARG" \
  -p vertical_amplitude:="$S_CURVE_VERTICAL_AMPLITUDE_ARG" \
  -p vertical_cycles:="$S_CURVE_VERTICAL_CYCLES" \
  -p pass_count:="$S_CURVE_PASSES" \
  -p speed_mps:="$S_CURVE_SPEED_ARG" \
  -p hold_time:="$S_CURVE_HOLD_TIME_ARG" \
  -p waypoint_spacing_m:="$S_CURVE_WAYPOINT_SPACING_ARG" \
  -p waypoint_hold_s:="$S_CURVE_WAYPOINT_HOLD_ARG" \
  -p waypoint_position_tolerance_m:="$S_CURVE_WAYPOINT_TOLERANCE_ARG" \
  -p localization_safety_enabled:="$LOCALIZATION_SAFETY_ENABLED_ARG" \
  -p localization_hold_s:="$LOCALIZATION_HOLD_S_ARG" \
  -p minimum_clearance_alt:="$MINIMUM_CLEARANCE_ALT_ARG" \
  -p calibration_yaw_sweep_deg:="$CALIBRATION_YAW_SWEEP_DEG_ARG" \
  -p locked_yaw_offset_deg:="$LOCKED_YAW_OFFSET_DEG_ARG" \
  -p land_at_end:="$LAND_AT_END_ARG" \
  -p navigation_source:="$NAVIGATION_SOURCE" \
  -p mavlink_takeoff_url:="$MAVLINK_TAKEOFF_URL" \
  2>&1 | tee "$LOG_DIR/guided_s_curve_waypoints.log"
status="${PIPESTATUS[0]}"
exit "$status"
