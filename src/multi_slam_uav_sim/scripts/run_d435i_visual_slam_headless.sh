#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)

source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
source "$PKG_SHARE/scripts/env.sh"
set -u

export D435I_RTAB_QOS=${D435I_RTAB_QOS:-1}

RTABMAP_PROFILE=${RTABMAP_PROFILE:-feature_aligned}
if [[ -n "${RTABMAP_CONFIG:-}" ]]; then
  RTABMAP_PROFILE_SOURCE=explicit_rtabmap_config
else
  RTABMAP_PROFILE_SOURCE=profile_name
  case "$RTABMAP_PROFILE" in
    baseline_mismatch)
      RTABMAP_CONFIG="$PKG_SHARE/config/d435i_rtabmap_baseline.yaml"
      ;;
    feature_aligned)
      RTABMAP_CONFIG="$PKG_SHARE/config/d435i_rtabmap_feature_aligned.yaml"
      ;;
    *)
      printf 'Unknown RTABMAP_PROFILE=%s (use baseline_mismatch or feature_aligned)\n' \
        "$RTABMAP_PROFILE" >&2
      exit 2
      ;;
  esac
fi

D435I_RESOLUTION_PROFILE=${D435I_RESOLUTION_PROFILE:-standard}
if [[ "$D435I_RESOLUTION_PROFILE" != "standard" ]]; then
  printf 'Unsupported D435I_RESOLUTION_PROFILE=%s; only the locally validated standard (640x480@30 target) profile is available.\n' \
    "$D435I_RESOLUTION_PROFILE" >&2
  exit 2
fi

D435I_ENABLE_RTABMAP=${D435I_ENABLE_RTABMAP:-1}
D435I_START_FLIGHT_STACK=${D435I_START_FLIGHT_STACK:-1}
GAZEBO_GUI=${GAZEBO_GUI:-0}
RTABMAP_GUI=${RTABMAP_GUI:-0}
RVIZ=${RVIZ:-0}
ENABLE_FLOW=${ENABLE_FLOW:-0}
ENABLE_FLOW_VIEWER=${ENABLE_FLOW_VIEWER:-0}
ENABLE_MID360=${ENABLE_MID360:-0}
ENABLE_D435I_POINTCLOUD=${ENABLE_D435I_POINTCLOUD:-0}
for binary_value in \
  "$D435I_ENABLE_RTABMAP" "$D435I_START_FLIGHT_STACK" \
  "$GAZEBO_GUI" "$RTABMAP_GUI" "$RVIZ" "$ENABLE_FLOW" \
  "$ENABLE_FLOW_VIEWER" "$ENABLE_MID360" "$ENABLE_D435I_POINTCLOUD"; do
  if [[ "$binary_value" != "0" && "$binary_value" != "1" ]]; then
    printf 'D435i workflow switches must be 0 or 1.\n' >&2
    exit 2
  fi
done
HEADLESS_MODE=$((1 - GAZEBO_GUI))
export D435I_RTABMAP_VIZ=$RTABMAP_GUI
export D435I_RVIZ=$RVIZ

SIM_PROFILE=${SIM_PROFILE:-d435i_only}
if [[ "$SIM_PROFILE" != "d435i_only" ]]; then
  printf 'Unsupported profile: %s (expected d435i_only)\n' "$SIM_PROFILE" >&2
  exit 2
fi

RUN_ID=${RUN_ID:-$(date +%Y%m%d_%H%M%S)}
RUN_DIR=${RUN_DIR:-$WS_ROOT/logs/d435i_visual_slam/headless/$RUN_ID}
ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_visual_slam.active}
PID_MANIFEST="$RUN_DIR/pids.tsv"
STACK_PID_FILE="$RUN_DIR/stack_components.tsv"
mkdir -p "$RUN_DIR"

if [[ -f "$ACTIVE_FILE" ]]; then
  read -r active_pid active_run <"$ACTIVE_FILE" || true
  if [[ -n "${active_pid:-}" ]] && kill -0 "$active_pid" 2>/dev/null; then
    printf 'D435i headless profile is already active: pid=%s run=%s\n' \
      "$active_pid" "${active_run:-unknown}" >&2
    exit 2
  fi
  "$PKG_SHARE/scripts/stop_d435i_visual_slam_headless.sh" || true
fi

printf 'component\tpid\tprocess_group\n' >"$PID_MANIFEST"
printf '%s\t%s\n' "$$" "$RUN_DIR" >"$ACTIVE_FILE"

pids=()
components=()
record_pid() {
  local component=$1
  local pid=$2
  pids+=("$pid")
  components+=("$component")
  printf '%s\t%s\t%s\n' "$component" "$pid" "$pid" >>"$PID_MANIFEST"
}

cleaning=0
cleanup() {
  if [[ "$cleaning" == "1" ]]; then
    return
  fi
  cleaning=1
  trap - EXIT INT TERM
  printf '\nStopping D435i visual SLAM headless profile...\n'
  for ((index=${#pids[@]}-1; index>=0; index--)); do
    pid=${pids[$index]}
    component=${components[$index]}
    if kill -0 "$pid" 2>/dev/null; then
      printf '  stopping %-24s pid=%s\n' "$component" "$pid"
      kill -INT -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  for _ in {1..20}; do
    alive=0
    for pid in "${pids[@]:-}"; do
      if kill -0 "$pid" 2>/dev/null; then alive=1; fi
    done
    if [[ "$alive" == "0" ]]; then break; fi
    sleep 0.5
  done
  for pid in "${pids[@]:-}"; do
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
    fi
  done
  if [[ -f "$ACTIVE_FILE" ]]; then
    read -r active_pid _ <"$ACTIVE_FILE" || true
    if [[ "${active_pid:-}" == "$$" ]]; then rm -f "$ACTIVE_FILE"; fi
  fi
  printf 'Logs: %s\n' "$RUN_DIR"
}
trap cleanup EXIT INT TERM

wait_for_topic() {
  local topic=$1
  local timeout_s=${2:-60}
  local started=$SECONDS
  printf 'Waiting for %s ...\n' "$topic"
  while (( SECONDS - started < timeout_s )); do
    if timeout 3s ros2 topic echo "$topic" --once >/dev/null 2>&1; then
      printf '  ready: %s\n' "$topic"
      return 0
    fi
    sleep 1
  done
  printf 'Timed out waiting for topic data: %s\n' "$topic" >&2
  return 1
}

publisher_count() {
  local topic=$1
  { ros2 topic info "$topic" 2>/dev/null || true; } | \
    awk '/Publisher count:/ {print $3}' | tail -n 1
}

WORLD_VARIANT=${D435I_WORLD:-baseline}
case "$WORLD_VARIANT" in
  baseline)
    WORLD="$PKG_SHARE/worlds/simple_apm_d435i_only.sdf"
    WORLD_NAME=simple_apm_d435i_only
    ;;
  textured)
    WORLD="$PKG_SHARE/worlds/simple_apm_rgbd_visual_textured.sdf"
    WORLD_NAME=simple_apm_rgbd_visual_textured
    ;;
  *)
    printf 'Unknown D435I_WORLD=%s (use baseline or textured)\n' "$WORLD_VARIANT" >&2
    exit 2
    ;;
esac

cat >"$RUN_DIR/profile.env" <<EOF
SIM_PROFILE=d435i_only
WORLD=$WORLD
WORLD_NAME=$WORLD_NAME
GAZEBO_GUI=$GAZEBO_GUI
HEADLESS_RENDERING=${HEADLESS_RENDERING:-1}
ENABLE_FLOW=$ENABLE_FLOW
ENABLE_FLOW_VIEWER=$ENABLE_FLOW_VIEWER
ENABLE_MID360=$ENABLE_MID360
ENABLE_D435I_POINTCLOUD=$ENABLE_D435I_POINTCLOUD
D435I_BRIDGE_IMPL=${D435I_BRIDGE_IMPL:-cpp}
D435I_RESOLUTION_PROFILE=$D435I_RESOLUTION_PROFILE
D435I_ENABLE_RTABMAP=$D435I_ENABLE_RTABMAP
D435I_START_FLIGHT_STACK=$D435I_START_FLIGHT_STACK
D435I_DEPTH_ENCODING=${D435I_DEPTH_ENCODING:-16UC1}
D435I_QOS_RELIABILITY=${D435I_QOS_RELIABILITY:-reliable}
D435I_QOS_DEPTH=${D435I_QOS_DEPTH:-1}
D435I_SYNC_QUEUE_DEPTH=${D435I_SYNC_QUEUE_DEPTH:-2}
D435I_RTAB_QOS=$D435I_RTAB_QOS
RTABMAP_PROFILE=$RTABMAP_PROFILE
RTABMAP_PROFILE_SOURCE=$RTABMAP_PROFILE_SOURCE
RTABMAP_CONFIG=$RTABMAP_CONFIG
RTABMAP_DATABASE=$RUN_DIR/rtabmap.db
RTABMAP_GUI=${RTABMAP_GUI:-0}
RVIZ=${RVIZ:-0}
EOF

printf 'Starting D435i-only Gazebo/SITL/MAVROS stack. Logs: %s\n' "$RUN_DIR"
setsid env \
  HEADLESS="$HEADLESS_MODE" \
  GAZEBO_GUI="$GAZEBO_GUI" \
  HEADLESS_RENDERING="${HEADLESS_RENDERING:-1}" \
  WORLD="$WORLD" WORLD_NAME="$WORLD_NAME" \
  LOG_DIR="$RUN_DIR/stack" \
  LOCK_FILE=/tmp/multi_slam_d435i_only_stack.lock \
  PID_FILE="$STACK_PID_FILE" \
  ENABLE_GAZEBO_FLOW="$ENABLE_FLOW" ENABLE_FCU_FLOW=0 \
  ENABLE_FLOW_VIEWER="$ENABLE_FLOW_VIEWER" \
  ENABLE_MID360="$ENABLE_MID360" ENABLE_D435I_BRIDGE=1 \
  ENABLE_D435I_POINTCLOUD="$ENABLE_D435I_POINTCLOUD" \
  D435I_BRIDGE_IMPL="${D435I_BRIDGE_IMPL:-cpp}" \
  D435I_DEPTH_ENCODING="${D435I_DEPTH_ENCODING:-16UC1}" \
  D435I_QOS_RELIABILITY="${D435I_QOS_RELIABILITY:-reliable}" \
  D435I_QOS_DEPTH="${D435I_QOS_DEPTH:-1}" \
  D435I_SYNC_QUEUE_DEPTH="${D435I_SYNC_QUEUE_DEPTH:-2}" \
  D435I_PERFORMANCE_STATS=1 \
  D435I_STATS_PERIOD_S="${D435I_STATS_PERIOD_S:-5.0}" \
  D435I_PERFORMANCE_CSV="$RUN_DIR/d435i_bridge_performance.csv" \
  START_SITL="$D435I_START_FLIGHT_STACK" \
  START_MAVROS="$D435I_START_FLIGHT_STACK" \
  ENABLE_FLIGHT_STATE_BRIDGE="$D435I_START_FLIGHT_STACK" \
  RECTANGLE_FLOW_TEST=0 AUTO_FLIGHT=0 \
  bash "$PKG_SHARE/scripts/run_apm_sensor_stack.sh" \
  >"$RUN_DIR/stack_supervisor.log" 2>&1 &
record_pid stack_supervisor "$!"

# Establish simulation time before starting any use_sim_time consumers.
clock_count=$(publisher_count /clock)
clock_count=${clock_count:-0}
if [[ "$clock_count" == "0" ]]; then
  setsid ros2 run ros_gz_bridge parameter_bridge \
    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
    >"$RUN_DIR/clock_bridge.log" 2>&1 &
  record_pid clock_bridge "$!"
elif [[ "$clock_count" != "1" ]]; then
  printf '/clock already has %s publishers; refusing ambiguous sim time.\n' \
    "$clock_count" >&2
  exit 2
else
  printf 'Using the existing single /clock publisher.\n'
fi

wait_for_topic /clock 30
clock_count=$(publisher_count /clock)
if [[ "$clock_count" != "1" ]]; then
  printf 'Expected exactly one /clock publisher, found %s.\n' \
    "${clock_count:-0}" >&2
  exit 2
fi

wait_for_topic /front/d435i/color/image_raw 75
wait_for_topic /front/d435i/aligned_depth_to_color/image_raw 30
wait_for_topic /front/d435i/color/camera_info 30

setsid ros2 run multi_slam_uav_sim gazebo_ground_truth_bridge --ros-args \
  -p world_name:="$WORLD_NAME" -p model_name:=apm_iris \
  -p output_topic:=/d435i_visual_slam/ground_truth \
  -p use_sim_time:=true \
  >"$RUN_DIR/ground_truth_bridge.log" 2>&1 &
record_pid ground_truth_bridge "$!"

if [[ "$D435I_ENABLE_RTABMAP" == "1" ]]; then
  printf 'RTAB-Map profile=%s source=%s config=%s database=%s\n' \
    "$RTABMAP_PROFILE" "$RTABMAP_PROFILE_SOURCE" "$RTABMAP_CONFIG" \
    "$RUN_DIR/rtabmap.db"
  setsid ros2 launch multi_slam_uav_sim d435i_rtabmap.launch.py \
    config_file:="$RTABMAP_CONFIG" \
    database_path:="$RUN_DIR/rtabmap.db" \
    >"$RUN_DIR/rtabmap.log" 2>&1 &
  record_pid rtabmap "$!"
  wait_for_topic /rtabmap/odom 60
  rtab_status=/rtabmap/odom
else
  rtab_status=disabled
fi
if [[ "$D435I_START_FLIGHT_STACK" == "1" ]]; then
  wait_for_topic /mavros/state 90
  wait_for_topic /mavros/local_position/pose 90
  mavros_status=/mavros/local_position/pose
else
  mavros_status=disabled
fi

cat <<EOF

D435i-only visual SLAM baseline is ready.

  RGB:          /front/d435i/color/image_raw
  aligned depth:/front/d435i/aligned_depth_to_color/image_raw
  CameraInfo:   /front/d435i/color/camera_info
  clock:        /clock (publisher count=1)
  RTAB-Map odom:$rtab_status
  RTAB profile:  $RTABMAP_PROFILE
  RTAB config:   $RTABMAP_CONFIG
  RTAB database: $RUN_DIR/rtabmap.db
  MAVROS pose:  $mavros_status
  ground truth: /d435i_visual_slam/ground_truth
  logs:         $RUN_DIR

No Gazebo GUI, RTAB-Map GUI, RViz, image viewer, optical-flow stack,
MID360 bridge, FAST-LIO, or D435i point cloud is started by default.
Set the documented 0/1 switches to opt into the GUI and sensor extras.
Press Ctrl+C or run stop_d435i_visual_slam_headless.sh to stop it.
EOF

wait "${pids[0]}"
