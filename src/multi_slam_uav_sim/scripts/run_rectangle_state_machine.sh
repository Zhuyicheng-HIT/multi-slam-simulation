#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)

source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"

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
  cat <<EOF
MAVROS / FCU topics are not visible after bounded ROS graph discovery.

Start the first-window stack first and wait for:
  MAVROS FCU connected.

First-window command:
  install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_sim_with_flow.sh

Continuing because the state-machine node has its own bounded wait for
/mavros/state.connected. This script will not start Gazebo, SITL, or MAVROS.
EOF
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

LOG_DIR=${LOG_DIR:-$WS_ROOT/logs/rectangle_state_machine_$(date +%Y%m%d_%H%M%S)}
mkdir -p "$LOG_DIR"

TAKEOFF_ALT=${TAKEOFF_ALT:-3.0}
RECTANGLE_LENGTH_X=${RECTANGLE_LENGTH_X:-2.0}
RECTANGLE_LENGTH_Y=${RECTANGLE_LENGTH_Y:-1.2}
RECTANGLE_SPEED=${RECTANGLE_SPEED:-0.20}
HOLD_TIME=${HOLD_TIME:-2.0}
POST_TAKEOFF_HOLD_TIME=${POST_TAKEOFF_HOLD_TIME:-3.0}
LAND_AT_END=${LAND_AT_END:-true}
FINAL_HOLD_TIME=${FINAL_HOLD_TIME:-0.0}
WORLD_NAME=${WORLD_NAME:-simple_apm_rgbd_mid360}
PREFLIGHT_WAIT_S=${PREFLIGHT_WAIT_S:-45.0}
NAVIGATION_STABLE_S=${NAVIGATION_STABLE_S:-1.0}
NAVIGATION_SOURCE=${NAVIGATION_SOURCE:-auto}
FLOW_MIN_QUALITY=${FLOW_MIN_QUALITY:-0}
ACCURACY_DURATION_S=${ACCURACY_DURATION_S:-150.0}
ENABLE_FLOW_ACCURACY=${ENABLE_FLOW_ACCURACY:-1}
MAVLINK_TAKEOFF_URL=${MAVLINK_TAKEOFF_URL:-tcp:127.0.0.1:5763}

float_param() {
  case "$1" in
    *.*) printf '%s\n' "$1" ;;
    *) printf '%s.0\n' "$1" ;;
  esac
}

TAKEOFF_ALT_PARAM=$(float_param "$TAKEOFF_ALT")
RECTANGLE_LENGTH_X_PARAM=$(float_param "$RECTANGLE_LENGTH_X")
RECTANGLE_LENGTH_Y_PARAM=$(float_param "$RECTANGLE_LENGTH_Y")
RECTANGLE_SPEED_PARAM=$(float_param "$RECTANGLE_SPEED")
HOLD_TIME_PARAM=$(float_param "$HOLD_TIME")
POST_TAKEOFF_HOLD_TIME_PARAM=$(float_param "$POST_TAKEOFF_HOLD_TIME")
FINAL_HOLD_TIME_PARAM=$(float_param "$FINAL_HOLD_TIME")
PREFLIGHT_WAIT_S_PARAM=$(float_param "$PREFLIGHT_WAIT_S")
NAVIGATION_STABLE_S_PARAM=$(float_param "$NAVIGATION_STABLE_S")
ACCURACY_DURATION_S_PARAM=$(float_param "$ACCURACY_DURATION_S")

cat <<EOF
GPS/GUIDED rectangle waypoint state machine starting.

Logs:
  $LOG_DIR/guided_rectangle_waypoints.log
  $LOG_DIR/flow_gazebo_accuracy.log
  $LOG_DIR/flow_gazebo_accuracy.csv

Parameters:
  takeoff_alt=$TAKEOFF_ALT
  length_x=$RECTANGLE_LENGTH_X
  length_y=$RECTANGLE_LENGTH_Y
  speed_mps=$RECTANGLE_SPEED
  hold_time=$HOLD_TIME
  post_takeoff_hold_time_s=$POST_TAKEOFF_HOLD_TIME
  final_hold_time_s=$FINAL_HOLD_TIME
  land_at_end=$LAND_AT_END
  preflight_timeout_s=$PREFLIGHT_WAIT_S
  navigation_stable_s=$NAVIGATION_STABLE_S
  navigation_source=$NAVIGATION_SOURCE
  flow_min_quality=$FLOW_MIN_QUALITY
  accuracy_duration_s=$ACCURACY_DURATION_S
  enable_flow_accuracy=$ENABLE_FLOW_ACCURACY
  mavlink_takeoff_url=$MAVLINK_TAKEOFF_URL

Required first-window stack:
  run_sim_with_flow.sh

EOF

accuracy_pid=""
if [[ "$ENABLE_FLOW_ACCURACY" == "1" ]]; then
  ros2 run multi_slam_uav_sim flow_gazebo_accuracy --ros-args \
    -p use_sim_time:=true \
    -p flow_topic:=/sim/optical_flow/rad \
    -p gazebo_world_name:="$WORLD_NAME" \
    -p gazebo_model:=apm_iris \
    -p duration_s:="$ACCURACY_DURATION_S_PARAM" \
    -p csv_path:="$LOG_DIR/flow_gazebo_accuracy.csv" \
    >"$LOG_DIR/flow_gazebo_accuracy.log" 2>&1 &
  accuracy_pid=$!
else
  printf 'Gazebo optical-flow accuracy diagnostic disabled.\\n' >"$LOG_DIR/flow_gazebo_accuracy.log"
fi

set +e
ros2 run multi_slam_uav_sim guided_rectangle_waypoints --ros-args \
  -p use_sim_time:=true \
  -p takeoff_alt:="$TAKEOFF_ALT_PARAM" \
  -p length_x:="$RECTANGLE_LENGTH_X_PARAM" \
  -p length_y:="$RECTANGLE_LENGTH_Y_PARAM" \
  -p speed_mps:="$RECTANGLE_SPEED_PARAM" \
  -p hold_time:="$HOLD_TIME_PARAM" \
  -p post_takeoff_hold_time_s:="$POST_TAKEOFF_HOLD_TIME_PARAM" \
  -p final_hold_time_s:="$FINAL_HOLD_TIME_PARAM" \
  -p land_at_end:="$LAND_AT_END" \
  -p preflight_wait_s:="$PREFLIGHT_WAIT_S_PARAM" \
  -p navigation_stable_s:="$NAVIGATION_STABLE_S_PARAM" \
  -p navigation_source:="$NAVIGATION_SOURCE" \
  -p flow_min_quality:="$FLOW_MIN_QUALITY" \
  -p mavlink_takeoff_url:="$MAVLINK_TAKEOFF_URL" \
  2>&1 | tee "$LOG_DIR/guided_rectangle_waypoints.log"
state_status="${PIPESTATUS[0]}"

if [[ -n "$accuracy_pid" ]] && kill -0 "$accuracy_pid" 2>/dev/null; then
  printf '\nWaiting for optical-flow accuracy summary...\n'
  wait "$accuracy_pid"
fi

if [[ "$ENABLE_FLOW_ACCURACY" == "1" ]] && grep -q 'FLOW_ACCURACY' "$LOG_DIR/flow_gazebo_accuracy.log" 2>/dev/null; then
  printf '\nOptical-flow accuracy summary:\n'
  grep 'FLOW_ACCURACY' "$LOG_DIR/flow_gazebo_accuracy.log" | tail -n 1
elif [[ "$ENABLE_FLOW_ACCURACY" == "1" ]]; then
  printf '\nNo FLOW_ACCURACY summary found yet. See:\n  %s\n' "$LOG_DIR/flow_gazebo_accuracy.log"
fi

exit "$state_status"
