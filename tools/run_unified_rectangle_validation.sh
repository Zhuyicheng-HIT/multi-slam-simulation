#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_DIR=${LOG_DIR:-"$REPO_ROOT/logs/unified_rectangle_$(date +%Y%m%d_%H%M%S)"}
LIDAR_WS=${LIDAR_WS:-"$HOME/multi-slam-deps/mid360_ws"}
VALIDATION_ROUTE=${VALIDATION_ROUTE:-rectangle}
VALIDATION_CALIBRATION_ONLY=${VALIDATION_CALIBRATION_ONLY:-false}
VALIDATION_ROS_DOMAIN_ID=${VALIDATION_ROS_DOMAIN_ID:-41}
if [[ ! "$VALIDATION_ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] ||
  (( VALIDATION_ROS_DOMAIN_ID < 0 || VALIDATION_ROS_DOMAIN_ID > 232 ))
then
  printf 'VALIDATION_ROS_DOMAIN_ID must be an integer in [0, 232].\n' >&2
  exit 2
fi
export ROS_DOMAIN_ID="$VALIDATION_ROS_DOMAIN_ID"
export RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}
VALIDATION_WORLD_NAME=${VALIDATION_WORLD_NAME:-${WORLD_NAME:-low_indoor_apm_rgbd_mid360}}
VALIDATION_WORLD_PATH=${VALIDATION_WORLD_PATH:-"$REPO_ROOT/install/multi_slam_uav_sim/share/multi_slam_uav_sim/worlds/${VALIDATION_WORLD_NAME}.sdf"}
VALIDATION_GAZEBO_WORLD_NAME=${VALIDATION_GAZEBO_WORLD_NAME:-$VALIDATION_WORLD_NAME}
VALIDATION_DYNAMIC_AGENTS_ENABLED=${VALIDATION_DYNAMIC_AGENTS_ENABLED:-false}
VALIDATION_DYNAMIC_AGENTS_CONFIG=${VALIDATION_DYNAMIC_AGENTS_CONFIG:-}
VALIDATION_TAKEOFF_ALT=${VALIDATION_TAKEOFF_ALT:-2.2}
VALIDATION_RANGE_FACET_ENABLED=${VALIDATION_RANGE_FACET_ENABLED:-false}
if [[ "$VALIDATION_ROUTE" == "s_curve" &&
  "${VALIDATION_CALIBRATION_ONLY,,}" == "true" ]]; then
  METRICS_DURATION=${METRICS_DURATION:-105}
  DRIFT_DURATION=${DRIFT_DURATION:-95}
  VALIDATION_RELOCALIZATION_WALL_TIMEOUT=${VALIDATION_RELOCALIZATION_WALL_TIMEOUT:-300}
elif [[ "$VALIDATION_ROUTE" == "s_curve" ]]; then
  METRICS_DURATION=${METRICS_DURATION:-280}
  DRIFT_DURATION=${DRIFT_DURATION:-270}
  VALIDATION_RELOCALIZATION_WALL_TIMEOUT=${VALIDATION_RELOCALIZATION_WALL_TIMEOUT:-600}
else
  METRICS_DURATION=${METRICS_DURATION:-135}
  DRIFT_DURATION=${DRIFT_DURATION:-125}
  VALIDATION_RELOCALIZATION_WALL_TIMEOUT=${VALIDATION_RELOCALIZATION_WALL_TIMEOUT:-300}
fi
VALIDATION_ENABLE_EXTERNALNAV_EKF3=${VALIDATION_ENABLE_EXTERNALNAV_EKF3:-0}
case "$VALIDATION_ENABLE_EXTERNALNAV_EKF3" in
  1|true|TRUE|yes|YES)
    VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC=${VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC:-/mavros/odometry/out}
    ;;
  *)
    # Estimator-only validation must not feed MAVROS' ODOMETRY plugin. The
    # plugin validates its own NED/FRD TF contract even when EKF3 ignores the
    # measurement, which can otherwise flood logs and delay FCU services.
    VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC=${VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC:-/fusion/externalnav/odom}
    ;;
esac
VALIDATION_MID360_SIM_BRIDGE_MODE=${VALIDATION_MID360_SIM_BRIDGE_MODE:-direct_livox}
VALIDATION_PRESERVE_LIO_ANCHOR=${VALIDATION_PRESERVE_LIO_ANCHOR:-false}
VALIDATION_PERFORMANCE_PROFILING=${VALIDATION_PERFORMANCE_PROFILING:-false}
VALIDATION_ENABLE_LIDAR_CALIBRATION_MOTION=${VALIDATION_ENABLE_LIDAR_CALIBRATION_MOTION:-false}
VALIDATION_AXIS_INFORMATION_HANDOFF_ENABLED=${VALIDATION_AXIS_INFORMATION_HANDOFF_ENABLED:-false}
VALIDATION_AXIS_HANDOFF_ENABLE_X=${VALIDATION_AXIS_HANDOFF_ENABLE_X:-false}
VALIDATION_AXIS_HANDOFF_ENABLE_Y=${VALIDATION_AXIS_HANDOFF_ENABLE_Y:-false}
VALIDATION_AXIS_HANDOFF_ENABLE_Z=${VALIDATION_AXIS_HANDOFF_ENABLE_Z:-true}
VALIDATION_GNSS_Z_REANCHOR_ENABLED=${VALIDATION_GNSS_Z_REANCHOR_ENABLED:-false}
VALIDATION_GNSS_Z_RECOVERY_INFORMATION_SCALE=${VALIDATION_GNSS_Z_RECOVERY_INFORMATION_SCALE:-0.50}
VALIDATION_BAROMETER_FALLBACK_ENABLED=${VALIDATION_BAROMETER_FALLBACK_ENABLED:-false}
VALIDATION_RELIABILITY_MODE=${VALIDATION_RELIABILITY_MODE:-dynamic}
case "$VALIDATION_RELIABILITY_MODE" in
  dynamic|fixed) ;;
  *)
    printf 'VALIDATION_RELIABILITY_MODE must be dynamic or fixed.\n' >&2
    exit 2
    ;;
esac
VALIDATION_FACTOR_PROFILE=${VALIDATION_FACTOR_PROFILE:-all}
case "$VALIDATION_FACTOR_PROFILE" in
  all|lidar|gnss|optical_flow|vision) ;;
  *)
    printf 'VALIDATION_FACTOR_PROFILE must be all, lidar, gnss, optical_flow, or vision.\n' >&2
    exit 2
    ;;
esac
if [[ -z "${VALIDATION_ENABLE_FLOW_ACCURACY+x}" ]]; then
  [[ "$VALIDATION_FACTOR_PROFILE" == "optical_flow" ]] && \
    VALIDATION_ENABLE_FLOW_ACCURACY=1 || VALIDATION_ENABLE_FLOW_ACCURACY=0
fi
if [[ -z "${VALIDATION_ENABLE_GAZEBO_FLOW+x}" ]]; then
  [[ "$VALIDATION_FACTOR_PROFILE" == "all" || \
    "$VALIDATION_FACTOR_PROFILE" == "optical_flow" ]] && \
    VALIDATION_ENABLE_GAZEBO_FLOW=1 || VALIDATION_ENABLE_GAZEBO_FLOW=0
fi
VALIDATION_RECORD_FASTLIO_ACCURACY=${VALIDATION_RECORD_FASTLIO_ACCURACY:-true}
VALIDATION_RECORD_RELIABILITY_TIMELINE=${VALIDATION_RECORD_RELIABILITY_TIMELINE:-false}
VALIDATION_RECORD_SLAM_DRIFT=${VALIDATION_RECORD_SLAM_DRIFT:-true}
VALIDATION_RESOURCE_INTERVAL_S=${VALIDATION_RESOURCE_INTERVAL_S:-2.0}
VALIDATION_START_FASTLIO_CLOUD_MAPPER=${VALIDATION_START_FASTLIO_CLOUD_MAPPER:-0}
VALIDATION_START_FASTLIO_OCCUPANCY_GRID=${VALIDATION_START_FASTLIO_OCCUPANCY_GRID:-0}
VALIDATION_LOCALIZATION_SAFETY_ENABLED=${VALIDATION_LOCALIZATION_SAFETY_ENABLED:-true}
VALIDATION_RECORD_REPLAY_BAG=${VALIDATION_RECORD_REPLAY_BAG:-true}
VALIDATION_RECORD_RAW_LIDAR=${VALIDATION_RECORD_RAW_LIDAR:-false}
VALIDATION_MINIMUM_PREFLIGHT_RTF=${VALIDATION_MINIMUM_PREFLIGHT_RTF:-0}
VALIDATION_REQUIRE_FASTLIO_DRIFT=${VALIDATION_REQUIRE_FASTLIO_DRIFT:-true}
VALIDATION_STOP_OBSERVERS_ON_LANDING=${VALIDATION_STOP_OBSERVERS_ON_LANDING:-true}
VALIDATION_STOP_AFTER_LANDING=${VALIDATION_STOP_AFTER_LANDING:-$VALIDATION_STOP_OBSERVERS_ON_LANDING}
VALIDATION_LANDING_GRACE_S=${VALIDATION_LANDING_GRACE_S:-5}
case "${VALIDATION_REQUIRE_FASTLIO_DRIFT,,}" in
  1|true|yes|on) validation_require_fastlio_drift=true ;;
  0|false|no|off) validation_require_fastlio_drift=false ;;
  *)
    printf 'VALIDATION_REQUIRE_FASTLIO_DRIFT must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
case "${VALIDATION_STOP_OBSERVERS_ON_LANDING,,}" in
  1|true|yes|on) validation_stop_observers_on_landing=true ;;
  0|false|no|off) validation_stop_observers_on_landing=false ;;
  *)
    printf 'VALIDATION_STOP_OBSERVERS_ON_LANDING must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
if [[ -z "${VALIDATION_MINIMUM_SIM_DURATION:-}" ]]; then
  if [[ "$validation_stop_observers_on_landing" == "true" ]]; then
    VALIDATION_MINIMUM_SIM_DURATION=120
  else
    VALIDATION_MINIMUM_SIM_DURATION="$METRICS_DURATION"
  fi
fi
if [[ ! "$VALIDATION_MINIMUM_SIM_DURATION" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'VALIDATION_MINIMUM_SIM_DURATION must be non-negative.\n' >&2
  exit 2
fi
case "${VALIDATION_STOP_AFTER_LANDING,,}" in
  1|true|yes|on) validation_stop_after_landing=true ;;
  0|false|no|off) validation_stop_after_landing=false ;;
  *)
    printf 'VALIDATION_STOP_AFTER_LANDING must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
case "${VALIDATION_DYNAMIC_AGENTS_ENABLED,,}" in
  1|true|yes|on) validation_dynamic_agents_enabled=true ;;
  0|false|no|off) validation_dynamic_agents_enabled=false ;;
  *)
    printf 'VALIDATION_DYNAMIC_AGENTS_ENABLED must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
if [[ "$validation_dynamic_agents_enabled" == "true" &&
  ! -f "$VALIDATION_DYNAMIC_AGENTS_CONFIG" ]]
then
  printf 'Dynamic-agent configuration is unavailable: %s\n' \
    "$VALIDATION_DYNAMIC_AGENTS_CONFIG" >&2
  exit 2
fi
if ! [[ "$VALIDATION_LANDING_GRACE_S" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'VALIDATION_LANDING_GRACE_S must be a non-negative number.\n' >&2
  exit 2
fi
if [[ -z "${VALIDATION_ROUTE_FEEDBACK_SOURCE:-}" ]]; then
  if [[ "$VALIDATION_ROUTE" == "rectangle" ]]; then
    VALIDATION_ROUTE_FEEDBACK_SOURCE=fcu_local
  else
    VALIDATION_ROUTE_FEEDBACK_SOURCE=unified_backend
  fi
fi
case "$VALIDATION_ROUTE:$VALIDATION_ROUTE_FEEDBACK_SOURCE" in
  rectangle:fcu_local|rectangle:gazebo_truth|s_curve:unified_backend|s_curve:fcu_local|s_curve:gazebo_truth) ;;
  *)
    printf 'Unsupported route/feedback pair: %s/%s\n' \
      "$VALIDATION_ROUTE" "$VALIDATION_ROUTE_FEEDBACK_SOURCE" >&2
    exit 2
    ;;
esac
VALIDATION_REQUIRE_TIME_CALIBRATION_LOCK=${VALIDATION_REQUIRE_TIME_CALIBRATION_LOCK:-false}
VALIDATION_REQUIRE_VISUAL_TIME_CALIBRATION_LOCK=${VALIDATION_REQUIRE_VISUAL_TIME_CALIBRATION_LOCK:-false}
VALIDATION_REQUIRE_TIME_CALIBRATION_APPLIED=${VALIDATION_REQUIRE_TIME_CALIBRATION_APPLIED:-false}
VALIDATION_ENABLE_VISION=${VALIDATION_ENABLE_VISION:-0}
VALIDATION_VISUAL_KEYFRAME_PROFILE=${VALIDATION_VISUAL_KEYFRAME_PROFILE:-balanced}
VALIDATION_VISUAL_FACTOR_MODE=${VALIDATION_VISUAL_FACTOR_MODE:-paper_reprojection}
VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M=${VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M:-35.0}
VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS=${VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS:-19}
if [[ ! "$VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M must be non-negative.\n' >&2
  exit 2
fi
if [[ ! "$VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS" =~ ^[0-9]+$ ]]; then
  printf 'VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS must be a non-negative integer.\n' >&2
  exit 2
fi
case "$VALIDATION_VISUAL_FACTOR_MODE" in
  paper_reprojection|rgbd_direct) ;;
  *)
    printf 'VALIDATION_VISUAL_FACTOR_MODE must be paper_reprojection or rgbd_direct.\n' >&2
    exit 2
    ;;
esac
case "$VALIDATION_VISUAL_KEYFRAME_PROFILE" in
  conservative|balanced_light|balanced|balanced_plus|dense|custom) ;;
  *)
    printf 'VALIDATION_VISUAL_KEYFRAME_PROFILE is unsupported.\n' >&2
    exit 2
    ;;
esac
case "${VALIDATION_ENABLE_VISION,,}" in
  1|true|yes|on)
    validation_enable_vision_arg=true
    validation_visual_factor_mode="$VALIDATION_VISUAL_FACTOR_MODE"
    ;;
  0|false|no|off)
    validation_enable_vision_arg=false
    validation_visual_factor_mode=disabled
    ;;
  *)
    printf 'VALIDATION_ENABLE_VISION must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
VALIDATION_REQUIRE_VISUAL_FACTORS=${VALIDATION_REQUIRE_VISUAL_FACTORS:-$VALIDATION_ENABLE_VISION}
VALIDATION_REQUIRE_AUTOMATIC_LOOP_CLOSURE=${VALIDATION_REQUIRE_AUTOMATIC_LOOP_CLOSURE:-false}
case "${VALIDATION_REQUIRE_VISUAL_FACTORS,,}" in
  1|true|yes|on) validation_require_visual_factors=true ;;
  0|false|no|off) validation_require_visual_factors=false ;;
  *)
    printf 'VALIDATION_REQUIRE_VISUAL_FACTORS must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
VALIDATION_RGBD_MINIMUM_DEPTH_M=${VALIDATION_RGBD_MINIMUM_DEPTH_M:-0.30}
# Gazebo depth is idealized; the real-hardware profile remains capped at 6 m.
VALIDATION_RGBD_MAXIMUM_DEPTH_M=${VALIDATION_RGBD_MAXIMUM_DEPTH_M:-10.0}
VALIDATION_CALIBRATION_YAW_SWEEP_DEG=${VALIDATION_CALIBRATION_YAW_SWEEP_DEG:-12.0}
VALIDATION_CALIBRATION_YAW_CYCLES=${VALIDATION_CALIBRATION_YAW_CYCLES:-3.0}
VALIDATION_CALIBRATION_MOTION_RADIUS_M=${VALIDATION_CALIBRATION_MOTION_RADIUS_M:-1.0}
VALIDATION_CALIBRATION_MOTION_SPEED_MPS=${VALIDATION_CALIBRATION_MOTION_SPEED_MPS:-0.60}
case "${VALIDATION_CALIBRATION_ONLY,,}" in
  1|true|yes|on) validation_calibration_only_arg=true ;;
  0|false|no|off) validation_calibration_only_arg=false ;;
  *) printf 'VALIDATION_CALIBRATION_ONLY must be true/false or 1/0.\n' >&2; exit 2 ;;
esac
if [[ "$validation_calibration_only_arg" == "true" &&
  "$VALIDATION_ROUTE" != "s_curve" ]]
then
  printf 'Calibration-only validation requires VALIDATION_ROUTE=s_curve.\n' >&2
  exit 2
fi
VALIDATION_RELOCALIZATION_TRIGGER_SIM_S=${VALIDATION_RELOCALIZATION_TRIGGER_SIM_S:-}
VALIDATION_RELOCALIZATION_TRIGGER_PHASE=${VALIDATION_RELOCALIZATION_TRIGGER_PHASE:-}
VALIDATION_RELOCALIZATION_CHECKPOINTS=${VALIDATION_RELOCALIZATION_CHECKPOINTS:-}
VALIDATION_RELOCALIZATION_VELOCITY_POLICY=${VALIDATION_RELOCALIZATION_VELOCITY_POLICY:-rotate}
VALIDATION_RELOCALIZATION_BIAS_POLICY=${VALIDATION_RELOCALIZATION_BIAS_POLICY:-preserve}
VALIDATION_RELOCALIZATION_STATIONARY_MAXIMUM_SPEED_MPS=${VALIDATION_RELOCALIZATION_STATIONARY_MAXIMUM_SPEED_MPS:-0.35}
VALIDATION_RELOCALIZATION_MOTION_PROFILE=${VALIDATION_RELOCALIZATION_MOTION_PROFILE:-hold}
VALIDATION_RELOCALIZATION_MOTION_RADIUS_M=${VALIDATION_RELOCALIZATION_MOTION_RADIUS_M:-0.6}
VALIDATION_RELOCALIZATION_MOTION_SPEED_MPS=${VALIDATION_RELOCALIZATION_MOTION_SPEED_MPS:-0.25}
VALIDATION_RELOCALIZATION_MOTION_YAW_RATE_DEG_S=${VALIDATION_RELOCALIZATION_MOTION_YAW_RATE_DEG_S:-12.0}
VALIDATION_RELOCALIZATION_MOTION_YAW_STEP_DEG=${VALIDATION_RELOCALIZATION_MOTION_YAW_STEP_DEG:-45.0}
VALIDATION_RELOCALIZATION_MOTION_SETTLE_S=${VALIDATION_RELOCALIZATION_MOTION_SETTLE_S:-2.5}
case "$VALIDATION_RELOCALIZATION_MOTION_PROFILE" in
  hold) validation_relocalization_motion_enabled=false ;;
  yaw_scan|circle|figure8) validation_relocalization_motion_enabled=true ;;
  *)
    printf 'Unknown relocalization motion profile: %s\n' \
      "$VALIDATION_RELOCALIZATION_MOTION_PROFILE" >&2
    exit 2
    ;;
esac
if [[ -n "$VALIDATION_RELOCALIZATION_CHECKPOINTS" ]]; then
  VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S=${VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S:-15.0}
else
  VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S=${VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S:-6.0}
fi
relocalization_trigger_modes=0
[[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_SIM_S" ]] &&
  ((relocalization_trigger_modes += 1))
[[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" ]] &&
  ((relocalization_trigger_modes += 1))
[[ -n "$VALIDATION_RELOCALIZATION_CHECKPOINTS" ]] &&
  ((relocalization_trigger_modes += 1))
if (( relocalization_trigger_modes > 1 ))
then
  printf 'Choose one relocalization trigger: simulation time, mission phase, or checkpoints.\n' >&2
  exit 2
fi
if [[ -n "$VALIDATION_RELOCALIZATION_CHECKPOINTS" &&
  "$VALIDATION_ROUTE" != "s_curve" ]]
then
  printf 'Checkpoint relocalization requires VALIDATION_ROUTE=s_curve.\n' >&2
  exit 2
fi
if [[ "$validation_relocalization_motion_enabled" == "true" &&
  -z "$VALIDATION_RELOCALIZATION_CHECKPOINTS" ]]
then
  printf 'Active relocalization motion requires checkpoint triggering.\n' >&2
  exit 2
fi
if [[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" ]]; then
  VALIDATION_POST_TAKEOFF_HOLD_TIME=${VALIDATION_POST_TAKEOFF_HOLD_TIME:-10.0}
else
  VALIDATION_POST_TAKEOFF_HOLD_TIME=${VALIDATION_POST_TAKEOFF_HOLD_TIME:-3.0}
fi
if [[ "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" == "final_loop_hold" ]]; then
  # The rectangle mission skips this phase when FINAL_HOLD_TIME is zero. Keep
  # the vehicle stationary long enough for reciprocal registration and the
  # backend epoch acknowledgement to complete before landing.
  VALIDATION_FINAL_HOLD_TIME=${VALIDATION_FINAL_HOLD_TIME:-12.0}
else
  VALIDATION_FINAL_HOLD_TIME=${VALIDATION_FINAL_HOLD_TIME:-0.0}
fi
# The stable path keeps FAST-LIO's internal prediction for deskew/matching but
# exports native factors and gives the unified backend final state/map
# ownership. Set this to 1 only for the experimental backend-trajectory A/B;
# the 2026-08-07 urban run exposed a request/factor deadlock in that mode.
FASTLIO_BACKEND_TRAJECTORY_FRONTEND=${FASTLIO_BACKEND_TRAJECTORY_FRONTEND:-0}
case "$FASTLIO_BACKEND_TRAJECTORY_FRONTEND" in
  1|true|TRUE|yes|YES) frontend_scan_prediction_enabled=true ;;
  0|false|FALSE|no|NO) frontend_scan_prediction_enabled=false ;;
  *)
    printf 'FASTLIO_BACKEND_TRAJECTORY_FRONTEND must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
mkdir -p "$LOG_DIR"
printf 'Validation ROS domain: %s\n' "$ROS_DOMAIN_ID"
printf 'Validation reliability: mode=%s lidar=%s imu=%s gnss=%s flow=%s vision=%s\n' \
  "$VALIDATION_RELIABILITY_MODE" \
  "${FIXED_LIDAR_WEIGHT:-1.0}" \
  "${FIXED_IMU_WEIGHT:-1.0}" \
  "${FIXED_GNSS_WEIGHT:-1.0}" \
  "${FIXED_OPTICAL_FLOW_WEIGHT:-1.0}" \
  "${FIXED_VISION_WEIGHT:-1.0}"
printf 'Validation factor profile: %s\n' "$VALIDATION_FACTOR_PROFILE"
printf 'Validation lightweight observers: flow_bridge=%s flow_accuracy=%s fastlio_accuracy=%s reliability_timeline=%s slam_drift=%s replay_bag=%s resource_interval=%ss\n' \
  "$VALIDATION_ENABLE_GAZEBO_FLOW" "$VALIDATION_ENABLE_FLOW_ACCURACY" \
  "$VALIDATION_RECORD_FASTLIO_ACCURACY" \
  "$VALIDATION_RECORD_RELIABILITY_TIMELINE" "$VALIDATION_RECORD_SLAM_DRIFT" \
  "$VALIDATION_RECORD_REPLAY_BAG" "$VALIDATION_RESOURCE_INTERVAL_S"
printf 'Validation visual cadence: profile=%s\n' \
  "$VALIDATION_VISUAL_KEYFRAME_PROFILE"
printf 'Validation axis recovery: lidar_axis_handoff=%s mask=%s,%s,%s gnss_z_reanchor=%s barometer=%s\n' \
  "$VALIDATION_AXIS_INFORMATION_HANDOFF_ENABLED" \
  "$VALIDATION_AXIS_HANDOFF_ENABLE_X" \
  "$VALIDATION_AXIS_HANDOFF_ENABLE_Y" \
  "$VALIDATION_AXIS_HANDOFF_ENABLE_Z" \
  "$VALIDATION_GNSS_Z_REANCHOR_ENABLED" \
  "$VALIDATION_BAROMETER_FALLBACK_ENABLED"
printf 'Validation GNSS Z recovery information scale: %s\n' \
  "$VALIDATION_GNSS_Z_RECOVERY_INFORMATION_SCALE"
printf 'Validation FAST-LIO drift gate: required=%s\n' \
  "$validation_require_fastlio_drift"
printf 'Validation observer stop: on_landing=%s minimum_sim_duration=%s\n' \
  "$validation_stop_observers_on_landing" \
  "$VALIDATION_MINIMUM_SIM_DURATION"
printf 'Validation figure-eight gates: minimum_distance_m=%s minimum_checkpoints=%s\n' \
  "$VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M" \
  "$VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS"
printf 'Validation world: file_profile=%s gazebo_name=%s path=%s\n' \
  "$VALIDATION_WORLD_NAME" "$VALIDATION_GAZEBO_WORLD_NAME" \
  "$VALIDATION_WORLD_PATH"
printf 'Validation dynamic agents: enabled=%s config=%s\n' \
  "$validation_dynamic_agents_enabled" \
  "${VALIDATION_DYNAMIC_AGENTS_CONFIG:-none}"
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
if ! ros2 pkg prefix "$RMW_IMPLEMENTATION" >/dev/null 2>&1; then
  printf 'ROS middleware implementation is unavailable: %s\n' \
    "$RMW_IMPLEMENTATION" >&2
  exit 2
fi
if [[ ! -f "$LIDAR_WS/install/setup.bash" ]]; then
  printf 'missing LiDAR workspace overlay: %s\n' "$LIDAR_WS/install/setup.bash" >&2
  exit 2
fi
source "$LIDAR_WS/install/setup.bash"

# This launcher owns the complete validation graph. Duplicate estimator or
# scheduler nodes mix clock domains and can publish competing factor/recovery
# decisions while still looking like a valid ROS topic. Refuse to start until
# the previous graph has been shut down explicitly.
existing_validation_nodes=$(
  timeout 6 ros2 node list --no-daemon --spin-time 2.0 2>/dev/null |
    grep -E '^/(fastlio_mapping|gazebo_clock_bridge|guided_rectangle_waypoints|guided_s_curve_waypoints|reliability_monitor|reliability_scheduler|relocalization_node|unified_backend_fusion|unified_external_nav_gate)$' || true
)
if [[ -n "$existing_validation_nodes" ]]; then
  printf 'A previous validation graph is still active:\n%s\n' \
    "$existing_validation_nodes" >&2
  printf 'Stop that graph before starting an isolated quantitative run.\n' >&2
  exit 2
fi

wait_rate() {
  local topic=$1
  local minimum_hz=$2
  local timeout_s=$3
  python3 "$REPO_ROOT/tools/topic_rate_probe.py" \
    --topic "$topic" --minimum-hz "$minimum_hz" --timeout "$timeout_s"
}

wait_static_transform() {
  local target_frame=$1
  local source_frame=$2
  local output
  output=$(timeout 12 ros2 run tf2_ros tf2_echo \
    "$target_frame" "$source_frame" 2>&1 || true)
  if [[ "$output" != *"Translation:"* || "$output" != *"Rotation:"* ]]; then
    printf 'Required static transform is unavailable: %s -> %s\n%s\n' \
      "$source_frame" "$target_frame" "$output" >&2
    return 1
  fi
}

pids=()
cleanup_started=0
cleanup() {
  if [[ "$cleanup_started" == "1" ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  sitl_pid=""
  if [[ -f "$LOG_DIR/sim/arducopter.pid" ]]; then
    sitl_pid=$(cat "$LOG_DIR/sim/arducopter.pid" 2>/dev/null || true)
    if [[ "$sitl_pid" =~ ^[0-9]+$ ]]; then
      kill -INT "$sitl_pid" 2>/dev/null || true
    fi
  fi
  for pid in "${pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  if [[ "$sitl_pid" =~ ^[0-9]+$ ]]; then
    kill -TERM "$sitl_pid" 2>/dev/null || true
  fi
  for pid in "${pids[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 2
  if [[ "$sitl_pid" =~ ^[0-9]+$ ]]; then
    kill -KILL "$sitl_pid" 2>/dev/null || true
  fi
  for pid in "${pids[@]:-}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    kill -KILL "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

setsid env HEADLESS=1 REQUIRE_GAZEBO_GPU=1 ENABLE_D435_POINTCLOUD=false \
  USE_SIM_TIME=true \
  WORLD="$VALIDATION_WORLD_PATH" \
  WORLD_NAME="$VALIDATION_GAZEBO_WORLD_NAME" \
  ENABLE_EXTERNALNAV_EKF3="$VALIDATION_ENABLE_EXTERNALNAV_EKF3" \
  ENABLE_GAZEBO_FLOW="$VALIDATION_ENABLE_GAZEBO_FLOW" \
  MAVROS_PLUGINLISTS_FILE="$REPO_ROOT/install/multi_slam_uav_sim/share/multi_slam_uav_sim/config/mavros_validation_pluginlists.yaml" \
  ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV=0 \
  MID360_SIM_BRIDGE_MODE="$VALIDATION_MID360_SIM_BRIDGE_MODE" \
  LOG_DIR="$LOG_DIR/sim" bash "$REPO_ROOT/tools/run_sim_with_unified_externalnav.sh" \
  >"$LOG_DIR/sim_launcher.log" 2>&1 &
pids+=("$!")
case "$VALIDATION_MID360_SIM_BRIDGE_MODE" in
  direct_livox)
    fastlio_pointcloud_bridge=0
    # Gazebo + ArduPilot cold starts can spend over 80 s before the direct
    # adapter is created. Keep the rate gate strict but leave enough time to
    # collect multiple source stamps after DDS discovery.
    wait_rate /livox/lidar 2.0 150
    ;;
  pointcloud_python)
    fastlio_pointcloud_bridge=1
    wait_rate /sim/mid360/points_raw 2.0 90
    ;;
  *)
    printf 'Unsupported validation MID360 bridge mode: %s\n' \
      "$VALIDATION_MID360_SIM_BRIDGE_MODE" >&2
    exit 2
    ;;
esac
wait_rate /mavros/imu/data_raw 20.0 40
wait_rate /sim/barometer/pressure 5.0 40

if [[ "$validation_dynamic_agents_enabled" == "true" ]]; then
  setsid ros2 run multi_slam_uav_sim people_motion --ros-args \
    --params-file "$VALIDATION_DYNAMIC_AGENTS_CONFIG" \
    -p use_sim_time:=true \
    -p world_name:="$VALIDATION_GAZEBO_WORLD_NAME" \
    >"$LOG_DIR/dynamic_agents.log" 2>&1 &
  dynamic_agents_pid=$!
  pids+=("$dynamic_agents_pid")
  sleep 2
  if ! kill -0 "$dynamic_agents_pid" 2>/dev/null; then
    printf 'Dynamic-agent controller exited during startup.\n' >&2
    tail -n 80 "$LOG_DIR/dynamic_agents.log" >&2 || true
    exit 1
  fi
fi

if [[ "$validation_enable_vision_arg" == "true" ]]; then
  setsid ros2 run d435i_rgbd_bridge_cpp d435i_rgbd_bridge --ros-args \
    -p use_sim_time:=true \
    -p gz_prefix:=/front/d435i/gz \
    -p ros_prefix:=/front/d435i \
    -p camera_link_frame:=d435i_link \
    -p depth_encoding:=16UC1 \
    -p min_depth_m:="$VALIDATION_RGBD_MINIMUM_DEPTH_M" \
    -p max_depth_m:="$VALIDATION_RGBD_MAXIMUM_DEPTH_M" \
    -p sync_queue_depth:=2 \
    -p qos_depth:=1 \
    -p qos_reliability:=best_effort \
    -p enable_pointcloud:=false \
    >"$LOG_DIR/d435i_rgbd_bridge.log" 2>&1 &
  pids+=("$!")

  setsid ros2 launch uf_visual_frontend visual_tight_coupling.launch.py \
    use_sim_time:=true \
    enabled:=true \
    start_fusion_stack:=false \
    visual_factor_mode:="$validation_visual_factor_mode" \
    visual_keyframe_profile:="$VALIDATION_VISUAL_KEYFRAME_PROFILE" \
    rgbd_minimum_depth_m:="$VALIDATION_RGBD_MINIMUM_DEPTH_M" \
    rgbd_maximum_depth_m:="$VALIDATION_RGBD_MAXIMUM_DEPTH_M" \
    >"$LOG_DIR/visual_frontend.log" 2>&1 &
  pids+=("$!")
  wait_rate /front/d435i/color/image_raw 2.0 60
  wait_rate /front/d435i/aligned_depth_to_color/image_raw 2.0 60
fi

setsid env LIDAR_WS="$LIDAR_WS" RVIZ=0 FASTLIO_INPUT_MODE=livox \
  USE_SIM_TIME=true \
  FASTLIO_BACKEND_TRAJECTORY_FRONTEND="$FASTLIO_BACKEND_TRAJECTORY_FRONTEND" \
  FASTLIO_NATIVE_FACTOR_EXPORT=1 \
  FASTLIO_DOWNSTREAM_BACKEND=1 \
  FASTLIO_DIAGNOSTIC_ODOMETRY=1 \
  FASTLIO_DIAGNOSTIC_PATH=0 \
  FASTLIO_DIAGNOSTIC_TF=0 \
  FASTLIO_MAP_INSERTION_MODE="${FASTLIO_MAP_INSERTION_MODE:-backend_confirmed}" \
  FASTLIO_BACKEND_STATE_TOPIC=/fusion/unified/map_pose \
  FASTLIO_BACKEND_ACTIVATION_STATE_TOPIC=/fusion/unified/frontend_activation_odom \
  START_FASTLIO_CLOUD_MAPPER="$VALIDATION_START_FASTLIO_CLOUD_MAPPER" \
  START_FASTLIO_OCCUPANCY_GRID="$VALIDATION_START_FASTLIO_OCCUPANCY_GRID" \
  START_LIVOX_POINTCLOUD_BRIDGE="$fastlio_pointcloud_bridge" \
  LOG_DIR="$LOG_DIR/fastlio" bash "$REPO_ROOT/tools/run_fastlio_mapping.sh" \
  >"$LOG_DIR/fastlio_launcher.log" 2>&1 &
pids+=("$!")

# Start the backend immediately after the frontend. In backend-confirmed map
# mode the frontend intentionally stops advancing after its bounded startup
# queue until the backend acknowledges optimized states.
setsid env ENABLE_VISION="$validation_enable_vision_arg" \
  VISUAL_FACTOR_MODE="$validation_visual_factor_mode" \
  RGBD_MINIMUM_DEPTH_M="$VALIDATION_RGBD_MINIMUM_DEPTH_M" \
  RGBD_MAXIMUM_DEPTH_M="$VALIDATION_RGBD_MAXIMUM_DEPTH_M" \
  USE_SIM_TIME=true \
  PRESERVE_LIO_ANCHOR="$VALIDATION_PRESERVE_LIO_ANCHOR" \
  FRONTEND_SCAN_PREDICTION_ENABLED="$frontend_scan_prediction_enabled" \
  RELOCALIZATION_SEARCH_TIMEOUT_S="$VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S" \
  RELOCALIZATION_VELOCITY_POLICY="$VALIDATION_RELOCALIZATION_VELOCITY_POLICY" \
  RELOCALIZATION_BIAS_POLICY="$VALIDATION_RELOCALIZATION_BIAS_POLICY" \
  RELOCALIZATION_STATIONARY_MAXIMUM_SPEED_MPS="$VALIDATION_RELOCALIZATION_STATIONARY_MAXIMUM_SPEED_MPS" \
  EXTERNAL_NAV_OUTPUT_TOPIC="$VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC" \
  PERFORMANCE_PROFILING_ENABLED="$VALIDATION_PERFORMANCE_PROFILING" \
  ENABLE_LIDAR_CALIBRATION_MOTION="$VALIDATION_ENABLE_LIDAR_CALIBRATION_MOTION" \
  AXIS_INFORMATION_HANDOFF_ENABLED="$VALIDATION_AXIS_INFORMATION_HANDOFF_ENABLED" \
  AXIS_HANDOFF_ENABLE_X="$VALIDATION_AXIS_HANDOFF_ENABLE_X" \
  AXIS_HANDOFF_ENABLE_Y="$VALIDATION_AXIS_HANDOFF_ENABLE_Y" \
  AXIS_HANDOFF_ENABLE_Z="$VALIDATION_AXIS_HANDOFF_ENABLE_Z" \
  GNSS_Z_REANCHOR_ENABLED="$VALIDATION_GNSS_Z_REANCHOR_ENABLED" \
  GNSS_Z_RECOVERY_INFORMATION_SCALE="$VALIDATION_GNSS_Z_RECOVERY_INFORMATION_SCALE" \
  BAROMETER_FALLBACK_ENABLED="$VALIDATION_BAROMETER_FALLBACK_ENABLED" \
  RANGE_FACET_ENABLED="$VALIDATION_RANGE_FACET_ENABLED" \
  RELIABILITY_MODE="$VALIDATION_RELIABILITY_MODE" \
  VISUAL_TIME_CALIBRATION_APPLY_LOCKED="${VISUAL_TIME_CALIBRATION_APPLY_LOCKED:-false}" \
  LOG_DIR="$LOG_DIR/unified" \
  bash "$REPO_ROOT/tools/run_unified_backend_stack.sh" \
  >"$LOG_DIR/unified_launcher.log" 2>&1 &
pids+=("$!")
backend_stack_pid=$!

sleep 2
if ! kill -0 "$backend_stack_pid" 2>/dev/null; then
  printf 'Unified backend stack exited during startup.\n' >&2
  tail -n 80 "$LOG_DIR/unified/online_backend.log" 2>/dev/null || true
  exit 1
fi

case "${FASTLIO_BACKEND_TRAJECTORY_FRONTEND:-0}" in
  1|true|TRUE|yes|YES)
    printf 'Backend-owned trajectory mode: FAST-LIO diagnostic topics are not backend startup dependencies.\n'
    ;;
  *)
    # The native factor packet, not FAST-LIO's diagnostic pose, is the backend
    # keyframe clock. Requiring /Odometry here can deadlock backend-confirmed
    # map insertion before the unified backend has started.
    wait_rate /fast_lio/native_lidar_factor 1.0 60
    if ! wait_rate /Odometry 1.0 10; then
      printf 'FAST-LIO diagnostic /Odometry is not continuously ready yet; continuing on native factors.\n'
    fi
    ;;
esac
wait_rate /fusion/unified/odom 2.0 60
case "$VALIDATION_ENABLE_EXTERNALNAV_EKF3" in
  # The backend first accumulates a stationary IMU bias window and initializes
  # its first native LiDAR state. Preserve the 10 Hz continuity requirement,
  # but do not spend its entire readiness budget on cold-start initialization.
  1|true|TRUE|yes|YES)
    wait_static_transform camera_init_ned camera_init
    wait_static_transform body_frd body
    wait_rate /mavros/odometry/out 10.0 75
    if awk -v value="$VALIDATION_MINIMUM_PREFLIGHT_RTF" \
      'BEGIN {exit !(value > 0.0)}'
    then
      if ! python3 "$REPO_ROOT/tools/topic_rate_probe.py" \
        --topic /mavros/odometry/out --minimum-hz 10.0 --timeout 20 \
        --window 40 \
        --minimum-wall-source-ratio "$VALIDATION_MINIMUM_PREFLIGHT_RTF"
      then
        printf 'Preflight RTF gate failed: require wall/source >= %s.\n' \
          "$VALIDATION_MINIMUM_PREFLIGHT_RTF" >&2
        exit 5
      fi
    fi
    ;;
  *) printf 'ExternalNav FCU consumption disabled; output continuity is metrics-only.\n' ;;
esac

case "${VALIDATION_RECORD_FASTLIO_ACCURACY,,}" in
  1|true|yes|on)
    setsid ros2 run multi_slam_uav_sim external_nav_accuracy --ros-args \
      -p use_sim_time:=false \
      -p world_name:="$VALIDATION_GAZEBO_WORLD_NAME" \
      -p odom_topic:=/Odometry \
      -p output_path:="$LOG_DIR/fastlio_accuracy.json" \
      >"$LOG_DIR/fastlio_accuracy.log" 2>&1 &
    pids+=("$!")
    ;;
  0|false|no|off) ;;
  *)
    printf 'VALIDATION_RECORD_FASTLIO_ACCURACY must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac

setsid ros2 run multi_slam_uav_sim external_nav_accuracy --ros-args \
  -p use_sim_time:=false \
  -p world_name:="$VALIDATION_GAZEBO_WORLD_NAME" \
  -p odom_topic:=/fusion/unified/odom \
  -p output_path:="$LOG_DIR/unified_accuracy.json" \
  >"$LOG_DIR/unified_accuracy.log" 2>&1 &
pids+=("$!")

observer_stop_args=()
if [[ "$validation_stop_observers_on_landing" == "true" ]]; then
  observer_stop_args+=(--stop-on-mission-phase landed)
fi

python3 "$REPO_ROOT/tools/unified_runtime_metrics.py" --duration "$METRICS_DURATION" \
  --output "$LOG_DIR/unified_runtime_metrics.json" \
  "${observer_stop_args[@]}" \
  --ros-args -p use_sim_time:=true \
  -p external_nav_topic:="$VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC" \
  >"$LOG_DIR/unified_runtime_metrics.log" 2>&1 &
metrics_pid=$!
pids+=("$metrics_pid")

timeline_pid=""
case "${VALIDATION_RECORD_RELIABILITY_TIMELINE,,}" in
  1|true|yes|on)
    setsid python3 \
      "$REPO_ROOT/src/ultra_fusion_nav/scripts/record_reliability_timeline.py" \
      --duration "$METRICS_DURATION" \
      --output "$LOG_DIR/reliability_timeline.json" \
      >"$LOG_DIR/reliability_timeline.log" 2>&1 &
    timeline_pid=$!
    pids+=("$timeline_pid")
    ;;
  0|false|no|off) ;;
  *)
    printf 'VALIDATION_RECORD_RELIABILITY_TIMELINE must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac

relocalization_trigger_pid=""
if [[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_SIM_S" ||
  -n "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" ||
  -n "$VALIDATION_RELOCALIZATION_CHECKPOINTS" ]]
then
  if [[ -n "$VALIDATION_RELOCALIZATION_CHECKPOINTS" ]]; then
    python3 "$REPO_ROOT/tools/trigger_relocalization_checkpoints.py" \
      --indices "$VALIDATION_RELOCALIZATION_CHECKPOINTS" \
      --motion-profile "$VALIDATION_RELOCALIZATION_MOTION_PROFILE" \
      --wall-timeout "$VALIDATION_RELOCALIZATION_WALL_TIMEOUT" \
      --output "$LOG_DIR/relocalization_trigger.json" \
      --ros-args -p use_sim_time:=true \
      >"$LOG_DIR/relocalization_trigger.log" 2>&1 &
  else
    relocalization_trigger_args=()
    if [[ -n "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE" ]]; then
      relocalization_trigger_args+=(
        --wait-for-phase "$VALIDATION_RELOCALIZATION_TRIGGER_PHASE")
    else
      relocalization_trigger_args+=(
        --after "$VALIDATION_RELOCALIZATION_TRIGGER_SIM_S")
    fi
    python3 "$REPO_ROOT/tools/trigger_relocalization_once.py" \
      "${relocalization_trigger_args[@]}" \
      --wall-timeout "$VALIDATION_RELOCALIZATION_WALL_TIMEOUT" \
      --output "$LOG_DIR/relocalization_trigger.json" \
      --ros-args -p use_sim_time:=true \
      >"$LOG_DIR/relocalization_trigger.log" 2>&1 &
  fi
  relocalization_trigger_pid=$!
  pids+=("$relocalization_trigger_pid")
fi

reliability_pid=""
case "${ENABLE_RELIABILITY_RECORD:-0}" in
  1|true|TRUE|yes|YES)
    setsid bash "$REPO_ROOT/tools/run_reliability_score_recorder.sh" \
      --duration "$METRICS_DURATION" \
      --output "$LOG_DIR/reliability_scores.csv" \
      "${observer_stop_args[@]}" \
      --allow-missing \
      >"$LOG_DIR/reliability_scores.log" 2>&1 &
    reliability_pid=$!
    pids+=("$reliability_pid")
    ;;
esac

replay_bag_pid=""
case "$VALIDATION_RECORD_REPLAY_BAG" in
  1|true|TRUE|yes|YES)
    raw_lidar_record_topic=()
    case "$VALIDATION_RECORD_RAW_LIDAR" in
      1|true|TRUE|yes|YES) raw_lidar_record_topic+=(/livox/lidar) ;;
      0|false|FALSE|no|NO) ;;
      *)
        printf 'VALIDATION_RECORD_RAW_LIDAR must be true/false or 1/0.\n' >&2
        exit 2
        ;;
    esac
    setsid ros2 bag record --use-sim-time \
      --compression-mode file --compression-format zstd \
      --compression-threads 1 \
      --output "$LOG_DIR/replay_bag" \
      /clock \
      /fast_lio/frontend_scan_request \
      /fast_lio/native_lidar_factor \
      "${raw_lidar_record_topic[@]}" \
      /sensors/imu \
      /sensors/gnss/fix \
      /sensors/gnss/raw \
      /sensors/optical_flow/rad \
      /sim/barometer/pressure \
      /mavros/imu/static_pressure \
      /reliability/scheduler_state \
      /reliability/lidar_score \
      /reliability/imu_score \
      /reliability/gnss_score \
      /reliability/optical_flow_score \
      /reliability/vision_score \
      /reliability/vision_factor_score \
      /vision/feature_tracks \
      /vision/rgbd_geometry_tracks \
      /vision/rgbd_direct_tracks \
      /fusion/unified/visual_timing \
      /lio/diagnostics \
      /calibration/lidar_relative_motion \
      /fusion/unified/odom \
      /fusion/unified/map_pose \
      /lio/odom \
      /lio/local_map \
      /lidar/points_deskewed \
      /fusion/unified/diagnostics \
      /fusion/unified/epoch \
      /relocalization/result \
      /relocalization/ready \
      /relocalization/motion_command \
      /relocalization/motion_status \
      "$VALIDATION_EXTERNAL_NAV_OUTPUT_TOPIC" \
      /external_nav/diagnostics \
      /sim/mid360/ground_truth_odom \
      /mission/phase \
      /mission/checkpoint \
      >"$LOG_DIR/replay_bag_record.log" 2>&1 &
    replay_bag_pid=$!
    pids+=("$replay_bag_pid")
    ;;
  0|false|FALSE|no|NO) ;;
  *)
    printf 'VALIDATION_RECORD_REPLAY_BAG must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac

setsid python3 "$REPO_ROOT/tools/collect_validation_resources.py" \
  --root-pid "$$" \
  --output "$LOG_DIR/resource_metrics.json" \
  --samples-output "$LOG_DIR/resource_samples.csv" \
  --interval "$VALIDATION_RESOURCE_INTERVAL_S" \
  >"$LOG_DIR/resource_metrics.log" 2>&1 &
resource_pid=$!
pids+=("$resource_pid")

case "$VALIDATION_ROUTE" in
  rectangle)
    route_script="$REPO_ROOT/tools/run_rectangle_state_machine.sh"
    route_log="$LOG_DIR/rectangle.log"
    ;;
  s_curve)
    route_script="$REPO_ROOT/tools/run_s_curve_state_machine.sh"
    route_log="$LOG_DIR/s_curve.log"
    ;;
  *)
    printf 'Unknown VALIDATION_ROUTE=%s (expected rectangle or s_curve)\n' \
      "$VALIDATION_ROUTE" >&2
    exit 2
    ;;
esac
env ROUTE_FEEDBACK_SOURCE="$VALIDATION_ROUTE_FEEDBACK_SOURCE" \
  ENABLE_FLOW_ACCURACY="$VALIDATION_ENABLE_FLOW_ACCURACY" \
  TAKEOFF_ALT="$VALIDATION_TAKEOFF_ALT" \
  LOCALIZATION_SAFETY_ENABLED="$VALIDATION_LOCALIZATION_SAFETY_ENABLED" \
  POST_TAKEOFF_HOLD_TIME="$VALIDATION_POST_TAKEOFF_HOLD_TIME" \
  FINAL_HOLD_TIME="$VALIDATION_FINAL_HOLD_TIME" \
  CALIBRATION_ONLY="$validation_calibration_only_arg" \
  CALIBRATION_YAW_SWEEP_DEG="$VALIDATION_CALIBRATION_YAW_SWEEP_DEG" \
  CALIBRATION_YAW_CYCLES="$VALIDATION_CALIBRATION_YAW_CYCLES" \
  CALIBRATION_MOTION_RADIUS_M="$VALIDATION_CALIBRATION_MOTION_RADIUS_M" \
  CALIBRATION_MOTION_SPEED_MPS="$VALIDATION_CALIBRATION_MOTION_SPEED_MPS" \
  RELOCALIZATION_MOTION_ENABLED="$validation_relocalization_motion_enabled" \
  RELOCALIZATION_MOTION_RADIUS_M="$VALIDATION_RELOCALIZATION_MOTION_RADIUS_M" \
  RELOCALIZATION_MOTION_SPEED_MPS="$VALIDATION_RELOCALIZATION_MOTION_SPEED_MPS" \
  RELOCALIZATION_MOTION_YAW_RATE_DEG_S="$VALIDATION_RELOCALIZATION_MOTION_YAW_RATE_DEG_S" \
  RELOCALIZATION_MOTION_YAW_STEP_DEG="$VALIDATION_RELOCALIZATION_MOTION_YAW_STEP_DEG" \
  RELOCALIZATION_MOTION_SETTLE_S="$VALIDATION_RELOCALIZATION_MOTION_SETTLE_S" \
  bash "$route_script" >"$route_log" 2>&1 &
route_pid=$!
pids+=("$route_pid")

# Run the two quantitative collectors in parallel with the route. When the
# route confirms LAND and FCU disarm, stop them after a short grace period so
# their finally blocks write partial-but-valid reports instead of waiting for
# the nominal 280 s duration.
drift_pid=""
case "${VALIDATION_RECORD_SLAM_DRIFT,,}" in
  1|true|yes|on)
    setsid python3 "$REPO_ROOT/tools/analyze_slam_drift.py" --duration "$DRIFT_DURATION" \
      --output "$LOG_DIR/slam_drift.json" \
      "${observer_stop_args[@]}" \
      --ros-args -p use_sim_time:=true \
      >"$LOG_DIR/slam_drift.log" 2>&1 &
    drift_pid=$!
    pids+=("$drift_pid")
    ;;
  0|false|no|off) ;;
  *)
    printf 'VALIDATION_RECORD_SLAM_DRIFT must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac

landing_seen=false
while kill -0 "$route_pid" 2>/dev/null; do
  if [[ "$validation_stop_after_landing" == "true" && -f "$route_log" ]]; then
    if grep -q 'LAND completed and FCU disarm confirmed' "$route_log"; then
      landing_seen=true
      printf 'landing_detected: route collectors will close after %.1fs\n' \
        "$VALIDATION_LANDING_GRACE_S"
      break
    fi
  fi
  sleep 1
done
route_status=0
wait "$route_pid" || route_status=$?
if [[ "$landing_seen" != "true" &&
  "$validation_stop_after_landing" == "true" &&
  -f "$route_log" ]] &&
  grep -q 'LAND completed and FCU disarm confirmed' "$route_log"
then
  landing_seen=true
  printf 'landing_detected: route collectors will close after %.1fs\n' \
    "$VALIDATION_LANDING_GRACE_S"
fi

stop_collector() {
  local pid="$1"
  [[ -n "$pid" ]] || return 0
  kill -INT -- "-$pid" 2>/dev/null || true
  kill -INT "$pid" 2>/dev/null || true
}
collector_stop_reason=
if [[ "$landing_seen" == "true" ]]; then
  collector_stop_reason=early_landing
  sleep "$VALIDATION_LANDING_GRACE_S"
elif (( route_status != 0 )); then
  collector_stop_reason=route_failed
  printf 'route_terminated: stopping collectors after route failure status %d\n' \
    "$route_status" >&2
fi
if [[ -n "$collector_stop_reason" ]]; then
  stop_collector "$metrics_pid"
  stop_collector "$drift_pid"
  stop_collector "$timeline_pid"
  stop_collector "$reliability_pid"
  stop_collector "$replay_bag_pid"
  stop_collector "$resource_pid"
  if (( route_status != 0 )); then
    stop_collector "$relocalization_trigger_pid"
  fi
fi

metrics_status=0
wait "$metrics_pid" || metrics_status=$?
if [[ -n "$collector_stop_reason" &&
  -s "$LOG_DIR/unified_runtime_metrics.json" ]]; then
  # Preserve the collector's partial sample report while making the stopping
  # reason explicit for scoring and downstream evidence review.
  python3 - "$LOG_DIR/unified_runtime_metrics.json" \
    "$collector_stop_reason" "$route_status" <<'PY'
import json
import sys

path = sys.argv[1]
reason = sys.argv[2]
route_status = int(sys.argv[3])
with open(path, encoding="utf-8") as stream:
    report = json.load(stream)
report["termination_reason"] = reason
if reason == "early_landing":
    report["early_stop_reason"] = "LAND completed and FCU disarm confirmed"
else:
    report["early_stop_reason"] = (
        f"route process exited with status {route_status}"
    )
with open(path, "w", encoding="utf-8") as stream:
    json.dump(report, stream, indent=2, sort_keys=True)
    stream.write("\n")
PY
  metrics_status=0
fi
timeline_status=0
if [[ -n "$timeline_pid" ]]; then
  wait "$timeline_pid" || timeline_status=$?
fi
drift_status=0
if [[ -n "$drift_pid" ]]; then
  wait "$drift_pid" || drift_status=$?
  if [[ -n "$collector_stop_reason" && -s "$LOG_DIR/slam_drift.json" ]]; then
    drift_status=0
  fi
fi
if [[ -n "$relocalization_trigger_pid" ]]; then
  relocalization_status=0
  wait "$relocalization_trigger_pid" || relocalization_status=$?
fi
if [[ -n "$reliability_pid" ]]; then
  reliability_status=0
  wait "$reliability_pid" || reliability_status=$?
fi
resource_status=0
stop_collector "$resource_pid"
wait "$resource_pid" || resource_status=$?
bag_status=0
if [[ -n "$replay_bag_pid" ]]; then
  kill -INT -- "-$replay_bag_pid" 2>/dev/null || true
  kill -INT "$replay_bag_pid" 2>/dev/null || true
  for _ in {1..100}; do
    if ! kill -0 "$replay_bag_pid" 2>/dev/null; then
      break
    fi
    sleep 0.1
  done
  wait "$replay_bag_pid" || bag_status=$?
fi
if [[ -n "$relocalization_trigger_pid" ]] &&
  (( relocalization_status != 0 ))
then
  printf 'relocalization_transaction_failed: status %d\n' \
    "$relocalization_status" >&2
  exit "$relocalization_status"
fi
if (( route_status != 0 )); then
  printf 'route_failed: %s exited with status %d\n' \
    "$VALIDATION_ROUTE" "$route_status" >&2
  exit "$route_status"
fi
if (( metrics_status != 0 )); then
  printf 'metric_collection_failed: runtime=%d\n' \
    "$metrics_status" >&2
  exit 3
fi
if [[ -n "$timeline_pid" ]]; then
  if (( timeline_status != 0 )) ||
    [[ ! -s "$LOG_DIR/reliability_timeline.json" ]]
  then
    printf 'timeline_collection_failed: status=%d report=%s\n' \
      "$timeline_status" "$LOG_DIR/reliability_timeline.json" >&2
    exit 3
  fi
fi
if [[ -n "$drift_pid" ]] && (( drift_status != 0 )); then
  if [[ "$validation_require_fastlio_drift" == "true" ]]; then
    printf 'metric_collection_failed: drift=%d runtime=%d\n' \
      "$drift_status" "$metrics_status" >&2
    exit 3
  fi
  printf 'fastlio_drift_diagnostic_failed_nonfatal: status %d\n' \
    "$drift_status" >&2
fi
if [[ -n "$reliability_pid" ]] && (( reliability_status != 0 )); then
  printf 'reliability_collection_failed: status %d\n' \
    "$reliability_status" >&2
  exit "$reliability_status"
fi
if (( resource_status != 0 )) || [[ ! -s "$LOG_DIR/resource_metrics.json" ]]; then
  printf 'resource_collection_failed: status=%d report=%s\n' \
    "$resource_status" "$LOG_DIR/resource_metrics.json" >&2
  exit 3
fi
if [[ -n "$replay_bag_pid" ]]; then
  if (( bag_status != 0 )) || [[ ! -s "$LOG_DIR/replay_bag/metadata.yaml" ]]; then
    printf 'replay_bag_record_failed: status=%d metadata=%s\n' \
      "$bag_status" "$LOG_DIR/replay_bag/metadata.yaml" >&2
    exit 3
  fi
fi
validation_gate_args=(
  --accuracy "$LOG_DIR/unified_accuracy.json"
  --runtime "$LOG_DIR/unified_runtime_metrics.json"
  --route-log "$route_log"
  --mavros-log "$LOG_DIR/sim/mavros.log"
  --sitl-log "$LOG_DIR/sim/sitl.log"
  --output "$LOG_DIR/validation_acceptance.json"
  --minimum-sim-duration "$VALIDATION_MINIMUM_SIM_DURATION"
  --expected-route-feedback "$VALIDATION_ROUTE_FEEDBACK_SOURCE"
  --minimum-figure-eight-distance "$VALIDATION_MINIMUM_FIGURE_EIGHT_DISTANCE_M"
  --minimum-figure-eight-checkpoints "$VALIDATION_MINIMUM_FIGURE_EIGHT_CHECKPOINTS"
  --factor-profile "$VALIDATION_FACTOR_PROFILE"
)
if [[ "$landing_seen" == "true" ]]; then
  for ((i=0; i<${#validation_gate_args[@]}; i++)); do
    if [[ "${validation_gate_args[$i]}" == "--minimum-sim-duration" ]]; then
      validation_gate_args[$((i + 1))]=0
      break
    fi
  done
fi
case "$VALIDATION_ENABLE_EXTERNALNAV_EKF3" in
  1|true|TRUE|yes|YES) validation_gate_args+=(--require-external-nav) ;;
esac
case "$VALIDATION_REQUIRE_TIME_CALIBRATION_LOCK" in
  1|true|TRUE|yes|YES) validation_gate_args+=(--require-time-lock) ;;
esac
case "$VALIDATION_REQUIRE_VISUAL_TIME_CALIBRATION_LOCK" in
  1|true|TRUE|yes|YES) validation_gate_args+=(--require-visual-time-lock) ;;
esac
case "$VALIDATION_REQUIRE_TIME_CALIBRATION_APPLIED" in
  1|true|TRUE|yes|YES) validation_gate_args+=(--require-time-applied) ;;
esac
if [[ "$validation_require_visual_factors" == "true" ]]; then
  validation_gate_args+=(--require-visual-factors)
fi
case "${VALIDATION_REQUIRE_AUTOMATIC_LOOP_CLOSURE,,}" in
  1|true|yes|on) validation_gate_args+=(--require-automatic-loop-closure) ;;
  0|false|no|off) ;;
  *)
    printf 'VALIDATION_REQUIRE_AUTOMATIC_LOOP_CLOSURE must be true/false or 1/0.\n' >&2
    exit 2
    ;;
esac
if [[ "$VALIDATION_ROUTE" == "rectangle" ]]; then
  python3 "$REPO_ROOT/tools/check_unified_validation_result.py" \
    "${validation_gate_args[@]}"
elif [[ "$VALIDATION_ROUTE" == "s_curve" &&
  "$validation_calibration_only_arg" == "true" ]]
then
  python3 "$REPO_ROOT/tools/check_unified_validation_result.py" \
    "${validation_gate_args[@]}" \
    --mission-profile calibration \
    --expected-waypoints 0
elif [[ "$VALIDATION_ROUTE" == "s_curve" ]]; then
  python3 "$REPO_ROOT/tools/check_unified_validation_result.py" \
    "${validation_gate_args[@]}" \
    --mission-profile figure_eight \
    --expected-waypoints 0
else
  printf 'Strict validation gate has no profile for VALIDATION_ROUTE=%s.\n' \
    "$VALIDATION_ROUTE" >&2
  exit 2
fi
printf 'validation_complete: %s\n' "$LOG_DIR"
