#!/usr/bin/env bash
set -eo pipefail

# Match the restored simulation stack's validated ROS 2 transport. Callers can
# still override this explicitly for a controlled middleware comparison.
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)

source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"

ROUTE_FEEDBACK_SOURCE=${ROUTE_FEEDBACK_SOURCE:-unified_backend}
GAZEBO_TRUTH_ODOM_TOPIC=${GAZEBO_TRUTH_ODOM_TOPIC:-/sim/mid360/ground_truth_odom}
case "$ROUTE_FEEDBACK_SOURCE" in
  unified_backend|gazebo_truth) ;;
  *)
    printf 'ROUTE_FEEDBACK_SOURCE must be unified_backend or gazebo_truth.\n' >&2
    exit 2
    ;;
esac

mavros_ready=false
for _attempt in $(seq 1 10); do
  mavros_topics=$(timeout 5s ros2 topic list --no-daemon --spin-time 2.0 2>/dev/null || true)
  if grep -q '^/mavros/state$' <<<"$mavros_topics"; then
    mavros_ready=true
    break
  fi
  sleep 0.5
done
if [[ "$mavros_ready" != true ]]; then
  printf 'MAVROS topics are not visible yet; continuing to the node bounded FCU wait.\n' >&2
fi
if ! python3 "$PKG_SHARE/scripts/wait_for_ros_message.py" \
    --topic /clock --timeout 24 --reliability best_effort \
    >/dev/null 2>&1; then
  printf 'ROS simulation /clock is unavailable. Start the updated simulation stack first.\n' >&2
  exit 2
fi

if ! python3 "$PKG_SHARE/scripts/wait_for_ros_message.py" \
    --topic /fusion/unified/odom --timeout 45 --reliability best_effort \
    >/dev/null 2>&1; then
  cat >&2 <<'EOF'
The figure-eight observer mission did not receive /fusion/unified/odom.
Start the FAST-LIO frontend and unified backend stack before this command.
The mission is aborting before arming because there would be no SLAM trajectory
to diagnose. This readiness check does not make SLAM a control input when the
selected feedback source is gazebo_truth.
EOF
  exit 2
fi
if [[ "$ROUTE_FEEDBACK_SOURCE" == "gazebo_truth" ]] && \
   ! python3 "$PKG_SHARE/scripts/wait_for_ros_message.py" \
      --topic "$GAZEBO_TRUTH_ODOM_TOPIC" --timeout 45 \
      --reliability best_effort >/dev/null 2>&1; then
  printf 'Gazebo-truth route feedback is unavailable: %s\n' \
    "$GAZEBO_TRUTH_ODOM_TOPIC" >&2
  exit 2
fi

LOG_DIR=${LOG_DIR:-$WS_ROOT/logs/s_curve_state_machine_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOG_DIR"

TAKEOFF_ALT=${TAKEOFF_ALT:-2.2}
S_CURVE_SPAN=${S_CURVE_SPAN:-9.0}
S_CURVE_AMPLITUDE=${S_CURVE_AMPLITUDE:-1.5}
S_CURVE_VERTICAL_AMPLITUDE=${S_CURVE_VERTICAL_AMPLITUDE:-0.8}
S_CURVE_VERTICAL_CYCLES=${S_CURVE_VERTICAL_CYCLES:-2}
S_CURVE_PASSES=${S_CURVE_PASSES:-1}
FIGURE8_ROTATION_DEG=${FIGURE8_ROTATION_DEG:-158.0}
FIGURE8_ALTITUDE_POWER=${FIGURE8_ALTITUDE_POWER:-4}
S_CURVE_SPEED=${S_CURVE_SPEED:-0.35}
S_CURVE_HOLD_TIME=${S_CURVE_HOLD_TIME:-3.0}
S_CURVE_WAYPOINT_SPACING=${S_CURVE_WAYPOINT_SPACING:-2.0}
S_CURVE_WAYPOINT_HOLD=${S_CURVE_WAYPOINT_HOLD:-1.0}
S_CURVE_WAYPOINT_TOLERANCE=${S_CURVE_WAYPOINT_TOLERANCE:-0.45}
MAX_ROUTE_COMMAND_OFFSET=${MAX_ROUTE_COMMAND_OFFSET:-1.50}
MAX_ROUTE_VERTICAL_OFFSET=${MAX_ROUTE_VERTICAL_OFFSET:-0.75}
MAX_ROUTE_ALTITUDE_MARGIN=${MAX_ROUTE_ALTITUDE_MARGIN:-0.50}
POST_TAKEOFF_HOLD_TIME=${POST_TAKEOFF_HOLD_TIME:-3.0}
FINAL_HOLD_TIME=${FINAL_HOLD_TIME:-0.0}
LOCALIZATION_SAFETY_ENABLED=${LOCALIZATION_SAFETY_ENABLED:-auto}
LOCALIZATION_HOLD_S=${LOCALIZATION_HOLD_S:-1.0}
MINIMUM_CLEARANCE_ALT=${MINIMUM_CLEARANCE_ALT:-1.5}
CALIBRATION_YAW_SWEEP_DEG=${CALIBRATION_YAW_SWEEP_DEG:-12.0}
CALIBRATION_YAW_CYCLES=${CALIBRATION_YAW_CYCLES:-3.0}
CALIBRATION_MOTION_ENABLED=${CALIBRATION_MOTION_ENABLED:-true}
CALIBRATION_MOTION_RADIUS_M=${CALIBRATION_MOTION_RADIUS_M:-1.0}
CALIBRATION_MOTION_SPEED_MPS=${CALIBRATION_MOTION_SPEED_MPS:-0.60}
CALIBRATION_MOTION_SAMPLES=${CALIBRATION_MOTION_SAMPLES:-161}
CALIBRATION_ONLY=${CALIBRATION_ONLY:-false}
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
MAX_ROUTE_COMMAND_OFFSET_ARG=$(as_double "$MAX_ROUTE_COMMAND_OFFSET")
MAX_ROUTE_VERTICAL_OFFSET_ARG=$(as_double "$MAX_ROUTE_VERTICAL_OFFSET")
MAX_ROUTE_ALTITUDE_MARGIN_ARG=$(as_double "$MAX_ROUTE_ALTITUDE_MARGIN")
POST_TAKEOFF_HOLD_TIME_ARG=$(as_double "$POST_TAKEOFF_HOLD_TIME")
FINAL_HOLD_TIME_ARG=$(as_double "$FINAL_HOLD_TIME")
LOCALIZATION_HOLD_S_ARG=$(as_double "$LOCALIZATION_HOLD_S")
MINIMUM_CLEARANCE_ALT_ARG=$(as_double "$MINIMUM_CLEARANCE_ALT")
CALIBRATION_YAW_SWEEP_DEG_ARG=$(as_double "$CALIBRATION_YAW_SWEEP_DEG")
CALIBRATION_YAW_CYCLES_ARG=$(as_double "$CALIBRATION_YAW_CYCLES")
CALIBRATION_MOTION_RADIUS_M_ARG=$(as_double "$CALIBRATION_MOTION_RADIUS_M")
CALIBRATION_MOTION_SPEED_MPS_ARG=$(as_double "$CALIBRATION_MOTION_SPEED_MPS")
LOCKED_YAW_OFFSET_DEG_ARG=$(as_double "$LOCKED_YAW_OFFSET_DEG")
FIGURE8_ROTATION_DEG_ARG=$(as_double "$FIGURE8_ROTATION_DEG")
if [[ "$S_CURVE_PASSES" != "1" ]]; then
  printf 'The large figure-eight is a single closed traversal; S_CURVE_PASSES must be 1.\n' >&2
  exit 2
fi
if ! [[ "$FIGURE8_ALTITUDE_POWER" =~ ^[0-9]+$ ]] \
    || (( FIGURE8_ALTITUDE_POWER < 2 || FIGURE8_ALTITUDE_POWER % 2 != 0 )); then
  printf 'FIGURE8_ALTITUDE_POWER must be an even integer >= 2.\n' >&2
  exit 2
fi
case "${LAND_AT_END,,}" in
  1|true|yes|on) LAND_AT_END_ARG=true ;;
  0|false|no|off) LAND_AT_END_ARG=false ;;
  *) printf 'LAND_AT_END must be true/false or 1/0, got %s.\n' "$LAND_AT_END" >&2; exit 2 ;;
esac
case "${CALIBRATION_MOTION_ENABLED,,}" in
  1|true|yes|on) CALIBRATION_MOTION_ENABLED_ARG=true ;;
  0|false|no|off) CALIBRATION_MOTION_ENABLED_ARG=false ;;
  *) printf 'CALIBRATION_MOTION_ENABLED must be true/false or 1/0, got %s.\n' "$CALIBRATION_MOTION_ENABLED" >&2; exit 2 ;;
esac
case "${CALIBRATION_ONLY,,}" in
  1|true|yes|on) CALIBRATION_ONLY_ARG=true ;;
  0|false|no|off) CALIBRATION_ONLY_ARG=false ;;
  *) printf 'CALIBRATION_ONLY must be true/false or 1/0, got %s.\n' "$CALIBRATION_ONLY" >&2; exit 2 ;;
esac
if ! [[ "$CALIBRATION_MOTION_SAMPLES" =~ ^[0-9]+$ ]] \
    || (( CALIBRATION_MOTION_SAMPLES < 33 )); then
  printf 'CALIBRATION_MOTION_SAMPLES must be an integer >= 33.\n' >&2
  exit 2
fi
case "${LOCALIZATION_SAFETY_ENABLED,,}" in
  1|true|yes|on) LOCALIZATION_SAFETY_ENABLED_ARG=true ;;
  0|false|no|off) LOCALIZATION_SAFETY_ENABLED_ARG=false ;;
  auto)
    scheduler_publishers=0
    for _attempt in 1 2 3; do
      scheduler_info=$(timeout 5s ros2 topic info --no-daemon --spin-time 1.0 \
        /reliability/scheduler_state 2>/dev/null || true)
      scheduler_publishers=$(sed -n 's/^Publisher count: \([0-9][0-9]*\)$/\1/p' \
        <<<"$scheduler_info")
      scheduler_publishers=${scheduler_publishers:-0}
      (( scheduler_publishers > 0 )) && break
      sleep 0.5
    done
    if (( scheduler_publishers > 0 )); then
      LOCALIZATION_SAFETY_ENABLED_ARG=true
      printf 'Localization safety auto mode: scheduler publisher detected; strict supervision enabled.\n'
    else
      LOCALIZATION_SAFETY_ENABLED_ARG=false
      printf 'Localization safety auto mode: no scheduler publisher; strict unified-backend route control remains active, but scheduler-triggered hold is unavailable.\n' >&2
    fi
    ;;
  *) printf 'LOCALIZATION_SAFETY_ENABLED must be auto, true/false, or 1/0, got %s.\n' "$LOCALIZATION_SAFETY_ENABLED" >&2; exit 2 ;;
esac
if [[ "$ROUTE_FEEDBACK_SOURCE" == "gazebo_truth" ]] && \
   [[ "$LOCALIZATION_SAFETY_ENABLED_ARG" != "false" ]]; then
  printf '%s\n' \
    'Gazebo-truth observer mode disables SLAM-triggered mission holds.' \
    'The unified backend remains active only for mapping and error evidence.'
  LOCALIZATION_SAFETY_ENABLED_ARG=false
fi

cat <<EOF
GUIDED 3D large figure-eight state machine starting.

Logs: $LOG_DIR
Route: span=$S_CURVE_SPAN m, lateral_amplitude=$S_CURVE_AMPLITUDE m,
       peak_rise=$S_CURVE_VERTICAL_AMPLITUDE m, axis=$FIGURE8_ROTATION_DEG deg,
       altitude_power=$FIGURE8_ALTITUDE_POWER, one closed traversal,
       speed=$S_CURVE_SPEED m/s
Stops: every $S_CURVE_WAYPOINT_SPACING m for at least $S_CURVE_WAYPOINT_HOLD s,
       endpoint hold=$S_CURVE_HOLD_TIME s, tolerance=$S_CURVE_WAYPOINT_TOLERANCE m
Safety: requested=$LOCALIZATION_SAFETY_ENABLED, enabled=$LOCALIZATION_SAFETY_ENABLED_ARG,
        minimum localization-loss hold=$LOCALIZATION_HOLD_S s
Feedback: $ROUTE_FEEDBACK_SOURCE; unified SLAM is observer-only when gazebo_truth,
          command offset limits=$MAX_ROUTE_COMMAND_OFFSET m horizontal / $MAX_ROUTE_VERTICAL_OFFSET m vertical
Altitude guard: base FCU altitude +/- $MAX_ROUTE_ALTITUDE_MARGIN m, planned rise + margin
Yaw: calibration_sweep=$CALIBRATION_YAW_SWEEP_DEG deg x $CALIBRATION_YAW_CYCLES cycles;
     calibration_motion=$CALIBRATION_MOTION_ENABLED_ARG, radius=$CALIBRATION_MOTION_RADIUS_M m,
     speed=$CALIBRATION_MOTION_SPEED_MPS m/s, samples=$CALIBRATION_MOTION_SAMPLES,
     calibration_only=$CALIBRATION_ONLY_ARG;
     first lobe follows path heading, second lobe locks its center-exit heading;
     heading offset=$LOCKED_YAW_OFFSET_DEG deg
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
  -p figure_eight_rotation_deg:="$FIGURE8_ROTATION_DEG_ARG" \
  -p figure_eight_altitude_power:="$FIGURE8_ALTITUDE_POWER" \
  -p speed_mps:="$S_CURVE_SPEED_ARG" \
  -p hold_time:="$S_CURVE_HOLD_TIME_ARG" \
  -p waypoint_spacing_m:="$S_CURVE_WAYPOINT_SPACING_ARG" \
  -p waypoint_hold_s:="$S_CURVE_WAYPOINT_HOLD_ARG" \
  -p waypoint_position_tolerance_m:="$S_CURVE_WAYPOINT_TOLERANCE_ARG" \
  -p route_feedback_source:="$ROUTE_FEEDBACK_SOURCE" \
  -p gazebo_truth_odom_topic:="$GAZEBO_TRUTH_ODOM_TOPIC" \
  -p max_route_command_offset_m:="$MAX_ROUTE_COMMAND_OFFSET_ARG" \
  -p max_route_vertical_offset_m:="$MAX_ROUTE_VERTICAL_OFFSET_ARG" \
  -p route_altitude_margin_m:="$MAX_ROUTE_ALTITUDE_MARGIN_ARG" \
  -p post_takeoff_hold_time_s:="$POST_TAKEOFF_HOLD_TIME_ARG" \
  -p final_hold_time_s:="$FINAL_HOLD_TIME_ARG" \
  -p localization_safety_enabled:="$LOCALIZATION_SAFETY_ENABLED_ARG" \
  -p localization_hold_s:="$LOCALIZATION_HOLD_S_ARG" \
  -p minimum_clearance_alt:="$MINIMUM_CLEARANCE_ALT_ARG" \
  -p calibration_yaw_sweep_deg:="$CALIBRATION_YAW_SWEEP_DEG_ARG" \
  -p calibration_yaw_cycles:="$CALIBRATION_YAW_CYCLES_ARG" \
  -p calibration_motion_enabled:="$CALIBRATION_MOTION_ENABLED_ARG" \
  -p calibration_motion_radius_m:="$CALIBRATION_MOTION_RADIUS_M_ARG" \
  -p calibration_motion_speed_mps:="$CALIBRATION_MOTION_SPEED_MPS_ARG" \
  -p calibration_motion_samples:="$CALIBRATION_MOTION_SAMPLES" \
  -p calibration_only:="$CALIBRATION_ONLY_ARG" \
  -p locked_yaw_offset_deg:="$LOCKED_YAW_OFFSET_DEG_ARG" \
  -p land_at_end:="$LAND_AT_END_ARG" \
  -p navigation_source:="$NAVIGATION_SOURCE" \
  -p mavlink_takeoff_url:="$MAVLINK_TAKEOFF_URL" \
  2>&1 | tee "$LOG_DIR/guided_s_curve_waypoints.log"
status="${PIPESTATUS[0]}"
exit "$status"
