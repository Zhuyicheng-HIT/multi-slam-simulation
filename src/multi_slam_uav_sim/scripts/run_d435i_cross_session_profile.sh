
#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
source "$PKG_SHARE/scripts/d435i_active_run_lifecycle.sh"
set -u

wait_for_publisher() {
  local topic=$1 timeout_s=${2:-60} started=$SECONDS info=
  while (( SECONDS - started < timeout_s )); do
    info=$(timeout 5s ros2 topic info "$topic" --no-daemon 2>/dev/null || true)
    if grep -Eq 'Publisher count: [1-9][0-9]*' <<<"$info"; then
      return 0
    fi
    sleep 1
  done
  return 1
}

wait_for_sample() {
  local topic=$1 timeout_s=${2:-60} started=$SECONDS
  while (( SECONDS - started < timeout_s )); do
    if timeout 7s ros2 topic echo "$topic" --no-daemon --spin-time 5.0 \
        --once >/dev/null 2>&1; then
      return 0
    fi
    sleep 1
  done
  return 1
}

ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_cross_session.active}
d435i_require_active_stack "$ACTIVE_FILE" "$WS_ROOT"
CONDITION=${CONDITION:-start_same}
CONDITIONS_CONFIG=${CONDITIONS_CONFIG:-$PKG_SHARE/config/d435i_relocalization_conditions.yaml}
LOCALIZATION_CONFIG=${LOCALIZATION_CONFIG:-$PKG_SHARE/config/d435i_rtabmap_localization.yaml}
REFERENCE_DB=${REFERENCE_DB:?REFERENCE_DB is required}
REFERENCE_METADATA=${REFERENCE_METADATA:?REFERENCE_METADATA is required}
OUTPUT_DIR=${OUTPUT_DIR:-$WS_ROOT/logs/d435i_visual_slam/cross_session/$(date +%Y%m%d_%H%M%S)_$CONDITION}
mkdir -p "$OUTPUT_DIR/monitor" "$OUTPUT_DIR/database_diagnostics"

if [[ ! -f "$REFERENCE_DB" || ! -f "$REFERENCE_METADATA" ]]; then
  printf 'Reference database or metadata is missing: %s %s\n' \
    "$REFERENCE_DB" "$REFERENCE_METADATA" >&2
  exit 2
fi
condition_values=$(python3 -c '
import sys,yaml
root=yaml.safe_load(open(sys.argv[1], encoding="utf-8"))
item=root["conditions"][sys.argv[2]]
print("\t".join(str(item[key]) for key in
      ("x_m","y_m","z_m","yaw_rad","sweep_deg")))
' "$CONDITIONS_CONFIG" "$CONDITION")
IFS=$'\t' read -r target_x target_y target_z target_yaw sweep_deg \
  <<<"$condition_values"
normalize_double() { printf '%.9f' "$1"; }
target_x=$(normalize_double "$target_x")
target_y=$(normalize_double "$target_y")
target_z=$(normalize_double "$target_z")
target_yaw=$(normalize_double "$target_yaw")
sweep_deg=$(normalize_double "$sweep_deg")

session_db="$OUTPUT_DIR/session2.db"
cp --reflink=auto --preserve=mode,timestamps -- "$REFERENCE_DB" "$session_db"
chmod u+w -- "$session_db"
reference_sha256_before=$(sha256sum "$REFERENCE_DB" | awk '{print $1}')
session_sha256_before=$(sha256sum "$session_db" | awk '{print $1}')
if [[ "$reference_sha256_before" != "$session_sha256_before" ]]; then
  printf 'Session database copy hash mismatch.\n' >&2
  exit 2
fi

cat >"$OUTPUT_DIR/run_context.env" <<EOF
condition=$CONDITION
target_x_m=$target_x
target_y_m=$target_y
target_z_m=$target_z
target_yaw_rad=$target_yaw
sweep_deg=$sweep_deg
reference_db=$REFERENCE_DB
reference_metadata=$REFERENCE_METADATA
reference_sha256_before=$reference_sha256_before
session_sha256_before=$session_sha256_before
git_commit=$(git -C "$WS_ROOT" rev-parse HEAD)
git_branch=$(git -C "$WS_ROOT" branch --show-current)
headless_run_dir=$D435I_ACTIVE_RUN_DIR
EOF

monitor_pid=""
motion_pid=""
rtab_pid=""
cleaning=0
stop_owned_group() {
  local pid=$1 needle=$2 label=$3 command
  [[ -n "$pid" && "$pid" =~ ^[0-9]+$ ]] || return 0
  kill -0 "$pid" 2>/dev/null || return 0
  command=$(tr '\0' ' ' <"/proc/$pid/cmdline" 2>/dev/null || true)
  if [[ "$command" != *"$needle"* ]]; then
    printf 'REFUSE cleanup label=%s pid=%s command=%s\n' \
      "$label" "$pid" "$command" >>"$OUTPUT_DIR/process_cleanup.log"
    return 1
  fi
  printf 'SIGNAL INT label=%s pid=%s command=%s\n' \
    "$label" "$pid" "$command" >>"$OUTPUT_DIR/process_cleanup.log"
  kill -INT -- "-$pid" 2>/dev/null || kill -INT "$pid" 2>/dev/null || true
  for _ in {1..50}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.2
  done
  printf 'SIGNAL TERM label=%s pid=%s\n' \
    "$label" "$pid" >>"$OUTPUT_DIR/process_cleanup.log"
  kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
  for _ in {1..25}; do
    kill -0 "$pid" 2>/dev/null || return 0
    sleep 0.2
  done
  printf 'SIGNAL KILL label=%s pid=%s\n' \
    "$label" "$pid" >>"$OUTPUT_DIR/process_cleanup.log"
  kill -KILL -- "-$pid" 2>/dev/null || kill -KILL "$pid" 2>/dev/null || true
}
cleanup() {
  local status=$?
  [[ "$cleaning" == "0" ]] || return
  cleaning=1
  trap - EXIT INT TERM
  if [[ -n "$motion_pid" ]] && kill -0 "$motion_pid" 2>/dev/null; then
    timeout 5s ros2 topic pub --once /d435i_cross_session/control \
      std_msgs/msg/String "{data: abort}" >>"$OUTPUT_DIR/process_cleanup.log" 2>&1 || true
    for _ in {1..150}; do
      kill -0 "$motion_pid" 2>/dev/null || break
      sleep 0.2
    done
  fi
  stop_owned_group "$motion_pid" d435i_relocalization_motion motion || true
  stop_owned_group "$rtab_pid" d435i_rtabmap.launch.py rtabmap || true
  stop_owned_group "$monitor_pid" d435i_cross_session_monitor monitor || true
  wait "$motion_pid" 2>/dev/null || true
  wait "$rtab_pid" 2>/dev/null || true
  wait "$monitor_pid" 2>/dev/null || true
  return "$status"
}
trap cleanup EXIT INT TERM

setsid ros2 run multi_slam_uav_sim d435i_cross_session_monitor --ros-args \
  -p use_sim_time:=true -p mode:=session -p condition:="$CONDITION" \
  -p ground_truth_topic:=/sim/mid360/ground_truth_odom \
  -p output_dir:="$OUTPUT_DIR/monitor" \
  >"$OUTPUT_DIR/monitor.log" 2>&1 &
monitor_pid=$!

setsid ros2 run multi_slam_uav_sim d435i_relocalization_motion --ros-args \
  -r __node:=d435i_relocalization_motion \
  -p use_sim_time:=true -p navigation_source:=gps \
  -p condition:="$CONDITION" \
  -p target_x_m:="$target_x" -p target_y_m:="$target_y" \
  -p target_z_m:="$target_z" -p target_yaw_rad:="$target_yaw" \
  -p observation_sweep_deg:="$sweep_deg" -p observation_hold_s:=8.0 \
  -p horizontal_speed_mps:=0.20 -p yaw_rate_deg_s:=12.0 \
  -p motion_hold_s:=2.0 -p takeoff_alt:=0.25 \
  -p takeoff_min_alt_m:=0.15 -p takeoff_free_climb_s:=14.0 \
  -p control_timeout_s:=240.0 \
  >"$OUTPUT_DIR/motion.log" 2>&1 &
motion_pid=$!

ready=0
for _ in {1..240}; do
  if grep -q 'RELOCALIZATION_STAGE.*position_ready' \
      "$OUTPUT_DIR/motion.log" 2>/dev/null; then
    ready=1
    break
  fi
  if ! kill -0 "$motion_pid" 2>/dev/null; then break; fi
  sleep 1
done
if [[ "$ready" != "1" ]]; then
  printf 'Positioning motion did not reach position_ready.\n' >&2
  exit 1
fi

setsid ros2 launch multi_slam_uav_sim d435i_rtabmap.launch.py \
  config_file:="$LOCALIZATION_CONFIG" database_path:="$session_db" \
  >"$OUTPUT_DIR/rtabmap.log" 2>&1 &
rtab_pid=$!
# Publisher discovery and sample reception are separate contracts. Requiring
# two sparse topics to be caught by consecutive short-lived CLI subscribers in
# the same polling iteration produced false startup failures under stage3 load.
# The combined 180 s budget remains below the motion node's 240 s hold timeout.
if ! wait_for_publisher /rtabmap/info 30 || \
   ! wait_for_publisher /rtabmap/odom 30 || \
   ! wait_for_sample /rtabmap/info 60 || \
   ! wait_for_sample /rtabmap/odom 60; then
  printf 'Localization RTAB-Map did not publish Info and odometry.\n' >&2
  exit 1
fi

timeout 10s ros2 topic pub --once /d435i_cross_session/control \
  std_msgs/msg/String "{data: observe}" >"$OUTPUT_DIR/control.log" 2>&1

# Freeze RTAB-Map while the aircraft is still holding the evaluated pose.
# The subsequent safety return must not contaminate localization stability,
# lost/reset, or jump statistics.
observation_complete=0
for _ in {1..180}; do
  if grep -q 'RELOCALIZATION_STAGE.*observation_complete' \
      "$OUTPUT_DIR/motion.log" 2>/dev/null; then
    observation_complete=1
    break
  fi
  if ! kill -0 "$motion_pid" 2>/dev/null; then break; fi
  sleep 1
done
if [[ "$observation_complete" != "1" ]]; then
  printf 'Relocalization observation did not complete.\n' >&2
  exit 1
fi
sleep 2
stop_owned_group "$rtab_pid" d435i_rtabmap.launch.py rtabmap
wait "$rtab_pid" 2>/dev/null || true
rtab_pid=""
timeout 10s ros2 topic pub --once /d435i_cross_session/control \
  std_msgs/msg/String "{data: return}" >>"$OUTPUT_DIR/control.log" 2>&1

motion_timed_out=1
for _ in {1..300}; do
  if ! kill -0 "$motion_pid" 2>/dev/null; then
    motion_timed_out=0
    break
  fi
  sleep 1
done
if [[ "$motion_timed_out" == "1" ]]; then
  printf 'Relocalization motion exceeded 300 seconds.\n' >&2
  exit 1
fi
set +e
wait "$motion_pid"
motion_exit=$?
set -e
motion_pid=""
printf 'motion_exit_code=%s\n' "$motion_exit" >>"$OUTPUT_DIR/run_context.env"
sleep 2
stop_owned_group "$monitor_pid" d435i_cross_session_monitor monitor
wait "$monitor_pid" 2>/dev/null || true
monitor_pid=""

reference_sha256_after=$(sha256sum "$REFERENCE_DB" | awk '{print $1}')
session_sha256_after=$(sha256sum "$session_db" | awk '{print $1}')
cat >>"$OUTPUT_DIR/run_context.env" <<EOF
reference_sha256_after=$reference_sha256_after
session_sha256_after=$session_sha256_after
EOF

ros2 run multi_slam_uav_sim rtabmap_database_diagnostics \
  "$session_db" "$OUTPUT_DIR/database_diagnostics" \
  --loop-csv "$OUTPUT_DIR/monitor/info_events.csv" \
  >"$OUTPUT_DIR/database_diagnostics.log" 2>&1
set +e
ros2 run multi_slam_uav_sim d435i_cross_session_analysis session \
  "$OUTPUT_DIR/monitor" "$session_db" "$OUTPUT_DIR/result" \
  --reference-metadata "$REFERENCE_METADATA" \
  --conditions-config "$CONDITIONS_CONFIG" --condition "$CONDITION" \
  --rtabmap-log "$OUTPUT_DIR/rtabmap.log" \
  --run-context "$OUTPUT_DIR/run_context.env"
analysis_exit=$?
set -e
printf 'analysis_exit_code=%s\n' "$analysis_exit" >>"$OUTPUT_DIR/run_context.env"
trap - EXIT INT TERM
cleanup
exit "$analysis_exit"
