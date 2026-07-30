#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
set -u

PROFILE=${ROBUSTNESS_PROFILE:-${1:-stationary}}
case "$PROFILE" in
  t0|stationary|hover|yaw_30|yaw_90|straight|l_shape|single_corner|small_rectangle|loop_return|ag) ;;
  *)
    printf 'Unknown profile: %s\n' "$PROFILE" >&2
    exit 2
    ;;
esac

ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_visual_slam.active}
if [[ ! -f "$ACTIVE_FILE" ]]; then
  printf 'Start run_d435i_visual_slam_headless.sh first.\n' >&2
  exit 2
fi
read -r wrapper_pid headless_run_dir <"$ACTIVE_FILE"
if ! kill -0 "$wrapper_pid" 2>/dev/null; then
  printf 'Recorded headless wrapper is not running: %s\n' "$wrapper_pid" >&2
  exit 2
fi

TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR=${OUTPUT_DIR:-$WS_ROOT/logs/d435i_visual_slam/robustness/${TIMESTAMP}_${PROFILE}}
mkdir -p "$OUTPUT_DIR"
PROFILER_LOG="$OUTPUT_DIR/profiler.log"
MOTION_LOG="$OUTPUT_DIR/motion.log"
profiler_pid=""

cleanup_profiler() {
  if [[ -n "$profiler_pid" ]] && kill -0 "$profiler_pid" 2>/dev/null; then
    kill -INT -- "-$profiler_pid" 2>/dev/null || kill -INT "$profiler_pid" 2>/dev/null || true
    for _ in {1..30}; do
      if ! kill -0 "$profiler_pid" 2>/dev/null; then
        wait "$profiler_pid" 2>/dev/null || true
        return 0
      fi
      sleep 0.2
    done
    kill -TERM -- "-$profiler_pid" 2>/dev/null || kill -TERM "$profiler_pid" 2>/dev/null || true
    wait "$profiler_pid" 2>/dev/null || true
  fi
  return 0
}
trap cleanup_profiler EXIT INT TERM

rtabmap_log="$headless_run_dir/rtabmap.log"
if [[ "$PROFILE" != "t0" && ! -f "$rtabmap_log" ]]; then
  printf 'Profile %s requires an active RTAB-Map log: %s\n' \
    "$PROFILE" "$rtabmap_log" >&2
  exit 2
fi

cat >"$OUTPUT_DIR/run_context.txt" <<EOF
profile=$PROFILE
output_dir=$OUTPUT_DIR
headless_run_dir=$headless_run_dir
git_commit=$(git -C "$WS_ROOT" rev-parse HEAD)
git_branch=$(git -C "$WS_ROOT" branch --show-current)
bridge_impl=cpp
depth_encoding=16UC1
resolution=640x480
exact_sync=true
rtab_parameters=unchanged
rtabmap_profile=${RTABMAP_PROFILE:-unknown}
navigation_source=gps_guided
rtab_controls_flight=false
EOF

if [[ "$PROFILE" != "t0" ]]; then
  node_list=$(ros2 node list 2>/dev/null || true)
  odom_node=$(printf '%s\n' "$node_list" | awk '/\/rgbd_odometry$/ {print; exit}')
  rtab_node=$(printf '%s\n' "$node_list" | awk '/\/rtabmap$/ {print; exit}')
  if [[ -z "$odom_node" || -z "$rtab_node" ]]; then
    printf 'Could not resolve RTAB-Map runtime node names.\n' >&2
    printf '%s\n' "$node_list" >&2
    exit 1
  fi
  {
    printf 'capture:\n'
    printf '  profile: %s\n' "${RTABMAP_PROFILE:-unknown}"
    printf '  odometry_node: %s\n' "$odom_node"
    printf '  mapping_node: %s\n' "$rtab_node"
    printf 'nodes:\n'
    printf '  odometry:\n'
    printf '    name: %s\n' "$odom_node"
    printf '    dump:\n'
    ros2 param dump "$odom_node" | sed 's/^/      /'
    printf '  mapping:\n'
    printf '    name: %s\n' "$rtab_node"
    printf '    dump:\n'
    ros2 param dump "$rtab_node" | sed 's/^/      /'
  } >"$OUTPUT_DIR/parameters.yaml"
  {
    printf 'node\tparameter\tros2_param_get\n'
    for spec in \
      "$odom_node|Kp/DetectorStrategy" \
      "$odom_node|Vis/FeatureType" \
      "$odom_node|Vis/MinInliers" \
      "$odom_node|Odom/ResetCountdown" \
      "$rtab_node|Kp/DetectorStrategy" \
      "$rtab_node|Vis/FeatureType" \
      "$rtab_node|Mem/UseOdomFeatures" \
      "$rtab_node|Rtabmap/LoopThr"; do
      node=${spec%%|*}
      parameter=${spec#*|}
      value=$(ros2 param get "$node" "$parameter" 2>&1 || true)
      printf '%s\t%s\t%s\n' "$node" "$parameter" "$value"
    done
  } >"$OUTPUT_DIR/parameter_checks.tsv"
  grep -E \
    'Odometry: (Update|Ignored) parameter|Update RTAB-Map parameter|Mem/UseOdomFeatures|Vis/FeatureType|Kp/DetectorStrategy' \
    "$rtabmap_log" >"$OUTPUT_DIR/internal_parameter_evidence.log" || true
fi

setsid ros2 run multi_slam_uav_sim rtabmap_robustness_profiler --ros-args \
  -p output_dir:="$OUTPUT_DIR" \
  -p profile:="$PROFILE" \
  -p duration_s:=0.0 \
  -p rtabmap_log_path:="$rtabmap_log" \
  >"$PROFILER_LOG" 2>&1 &
profiler_pid=$!

for _ in {1..40}; do
  if grep -q 'RTAB robustness profiler active' "$PROFILER_LOG" 2>/dev/null; then
    break
  fi
  if ! kill -0 "$profiler_pid" 2>/dev/null; then
    cat "$PROFILER_LOG" >&2
    exit 1
  fi
  sleep 0.25
done
if ! grep -q 'RTAB robustness profiler active' "$PROFILER_LOG" 2>/dev/null; then
  printf 'Profiler did not become ready.\n' >&2
  exit 1
fi

if ! timeout 10s ros2 topic echo /front/d435i/transport/frame_tracking --once \
    >/dev/null 2>&1; then
  printf 'Bridge tracking topic has no data.\n' >&2
  exit 1
fi

status=0
if [[ "$PROFILE" == "t0" ]]; then
  T0_DURATION_S=${T0_DURATION_S:-60}
  printf 'T0 Gazebo + bridge observation for %ss.\n' "$T0_DURATION_S" | tee "$MOTION_LOG"
  sleep "$T0_DURATION_S"
elif [[ "$PROFILE" == "ag" ]]; then
  set +e
  LOG_DIR="$OUTPUT_DIR/ag_flight" \
    bash "$PKG_SHARE/scripts/run_d435i_visual_slam_flight.sh" \
    >"$MOTION_LOG" 2>&1
  status=$?
  set -e
else
  PRE_OBSERVATION_S=${PRE_OBSERVATION_S:-5.0}
  POST_OBSERVATION_S=${POST_OBSERVATION_S:-5.0}
  STATIONARY_S=${STATIONARY_S:-30.0}
  HOVER_S=${HOVER_S:-30.0}
  FLIGHT_ALTITUDE_M=${FLIGHT_ALTITUDE_M:-0.5}
  MOTION_DISTANCE_M=${MOTION_DISTANCE_M:-1.0}
  RECTANGLE_X_M=${RECTANGLE_X_M:-0.75}
  RECTANGLE_Y_M=${RECTANGLE_Y_M:-0.50}
  MOTION_SPEED_MPS=${MOTION_SPEED_MPS:-0.10}
  YAW_SPEED_DEG_S=${YAW_SPEED_DEG_S:-8.0}
  MOTION_HOLD_S=${MOTION_HOLD_S:-3.0}
  INITIAL_TAKEOFF_ALT=${INITIAL_TAKEOFF_ALT:-0.25}
  normalize_double() {
    printf '%.6f' "$1"
  }
  PRE_OBSERVATION_S=$(normalize_double "$PRE_OBSERVATION_S")
  POST_OBSERVATION_S=$(normalize_double "$POST_OBSERVATION_S")
  STATIONARY_S=$(normalize_double "$STATIONARY_S")
  HOVER_S=$(normalize_double "$HOVER_S")
  FLIGHT_ALTITUDE_M=$(normalize_double "$FLIGHT_ALTITUDE_M")
  MOTION_DISTANCE_M=$(normalize_double "$MOTION_DISTANCE_M")
  RECTANGLE_X_M=$(normalize_double "$RECTANGLE_X_M")
  RECTANGLE_Y_M=$(normalize_double "$RECTANGLE_Y_M")
  MOTION_SPEED_MPS=$(normalize_double "$MOTION_SPEED_MPS")
  YAW_SPEED_DEG_S=$(normalize_double "$YAW_SPEED_DEG_S")
  MOTION_HOLD_S=$(normalize_double "$MOTION_HOLD_S")
  INITIAL_TAKEOFF_ALT=$(normalize_double "$INITIAL_TAKEOFF_ALT")
  cat >>"$OUTPUT_DIR/run_context.txt" <<EOF
pre_observation_s=$PRE_OBSERVATION_S
post_observation_s=$POST_OBSERVATION_S
stationary_s=$STATIONARY_S
hover_s=$HOVER_S
flight_altitude_m=$FLIGHT_ALTITUDE_M
distance_m=$MOTION_DISTANCE_M
rectangle_x_m=$RECTANGLE_X_M
rectangle_y_m=$RECTANGLE_Y_M
motion_speed_mps=$MOTION_SPEED_MPS
yaw_speed_deg_s=$YAW_SPEED_DEG_S
motion_hold_s=$MOTION_HOLD_S
initial_takeoff_alt=$INITIAL_TAKEOFF_ALT
EOF
  set +e
  ros2 run multi_slam_uav_sim progressive_visual_motion --ros-args \
    -p profile:="$PROFILE" \
    -p navigation_source:=gps \
    -p takeoff_alt:="$INITIAL_TAKEOFF_ALT" \
    -p takeoff_min_alt_m:=0.15 \
    -p takeoff_free_climb_s:=14.0 \
    -p pre_observation_s:="$PRE_OBSERVATION_S" \
    -p post_observation_s:="$POST_OBSERVATION_S" \
    -p stationary_s:="$STATIONARY_S" \
    -p hover_s:="$HOVER_S" \
    -p flight_altitude_m:="$FLIGHT_ALTITUDE_M" \
    -p distance_m:="$MOTION_DISTANCE_M" \
    -p rectangle_x_m:="$RECTANGLE_X_M" \
    -p rectangle_y_m:="$RECTANGLE_Y_M" \
    -p motion_speed_mps:="$MOTION_SPEED_MPS" \
    -p yaw_speed_deg_s:="$YAW_SPEED_DEG_S" \
    -p motion_hold_s:="$MOTION_HOLD_S" \
    >"$MOTION_LOG" 2>&1
  status=$?
  set -e
fi

sleep 2
cleanup_profiler
profiler_pid=""
trap - EXIT INT TERM

if [[ -f "$headless_run_dir/d435i_bridge_performance.csv" ]]; then
  cp "$headless_run_dir/d435i_bridge_performance.csv" \
    "$OUTPUT_DIR/bridge_performance.csv"
fi
printf 'motion_exit_code=%s\n' "$status" >>"$OUTPUT_DIR/run_context.txt"
if [[ ! -f "$OUTPUT_DIR/summary.md" ]]; then
  printf 'Profiler did not produce summary.md.\n' >&2
  cat "$PROFILER_LOG" >&2
  exit 1
fi
printf 'Robustness result: %s\n' "$OUTPUT_DIR"
cat "$OUTPUT_DIR/summary.md"
exit "$status"
