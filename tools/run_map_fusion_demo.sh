#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
RUN_ID=${RUN_ID:-map_fusion_demo_$(date +%Y%m%d_%H%M%S)}
RUN_DIR=${RUN_DIR:-$REPO_ROOT/logs/map_fusion_demo/$RUN_ID}
OFFLINE_METHOD=${OFFLINE_METHOD:-gicp}
DEMO_ROUTE=${DEMO_ROUTE:-short_s}
DEMO_GUI=${DEMO_GUI:-1}
DEMO_RVIZ=${DEMO_RVIZ:-1}
KEEP_DEMO_OPEN=${KEEP_DEMO_OPEN:-1}

case "$OFFLINE_METHOD" in
  gicp|hybrid) ;;
  *) printf 'OFFLINE_METHOD must be gicp or hybrid.\n' >&2; exit 2 ;;
esac
case "$DEMO_ROUTE" in
  short_s|long_s) ;;
  *) printf 'DEMO_ROUTE must be short_s or long_s.\n' >&2; exit 2 ;;
esac
for value in "$DEMO_GUI" "$DEMO_RVIZ" "$KEEP_DEMO_OPEN"; do
  [[ "$value" == 0 || "$value" == 1 ]] || {
    printf 'DEMO_GUI, DEMO_RVIZ and KEEP_DEMO_OPEN must be 0 or 1.\n' >&2
    exit 2
  }
done

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
if [[ -f "$LIDAR_WS/install/setup.bash" ]]; then
  source "$LIDAR_WS/install/setup.bash"
fi

SIM_SHARE=$(ros2 pkg prefix multi_slam_uav_sim)/share/multi_slam_uav_sim
HYBRID_SHARE=$(ros2 pkg prefix hybridfusion_map_fusion)/share/hybridfusion_map_fusion
STACK_RUNNER="$SIM_SHARE/scripts/run_pr6_d435i_visual_headless.sh"
ROUTE_RUNNER="$SIM_SHARE/scripts/run_s_curve_state_machine.sh"
RVIZ_CONFIG="$SIM_SHARE/config/map_fusion_demo.rviz"
for required in "$STACK_RUNNER" "$ROUTE_RUNNER" "$RVIZ_CONFIG" \
  "$HYBRID_SHARE/config/hybridfusion.yaml"; do
  [[ -e "$required" ]] || {
    printf 'Missing installed demo artifact: %s\nBuild the workspace first.\n' \
      "$required" >&2
    exit 2
  }
done

mkdir -p "$RUN_DIR" "$RUN_DIR/stack" "$RUN_DIR/route" \
  "$RUN_DIR/export/visual" "$RUN_DIR/offline"
printf '%s\n' \
  'Online map: current source-aware voxel baseline; not probability fusion.' \
  'Offline map: cross-source registration plus concatenation/voxel downsampling.' \
  'Current RGB-D depth baseline remains idealized and uses legacy range gates.' \
  >"$RUN_DIR/CURRENT_BASELINE_LIMITS.txt"

declare -a owned_groups=()
cleanup_started=0
record_group() {
  owned_groups+=("$1")
}
stop_groups() {
  local pid
  for pid in "${owned_groups[@]}"; do
    [[ -n "$pid" ]] || continue
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${owned_groups[@]}"; do
    [[ -n "$pid" ]] || continue
    kill -KILL -- "-$pid" 2>/dev/null || true
  done
}
cleanup() {
  local status=$?
  [[ "$cleanup_started" == 0 ]] || return
  cleanup_started=1
  trap - EXIT INT TERM
  stop_groups
  printf '\nDemo stopped (status=%s). Results: %s\n' "$status" "$RUN_DIR"
  exit "$status"
}
trap cleanup EXIT INT TERM

wait_for_publisher() {
  local topic=$1 timeout_s=${2:-180} started=$SECONDS info= count=0
  while (( SECONDS - started < timeout_s )); do
    if [[ -n "${STACK_PID:-}" ]] && ! kill -0 "$STACK_PID" 2>/dev/null; then
      printf 'Simulation stack exited while waiting for publisher: %s\n' \
        "$topic" >&2
      return 1
    fi
    info=$(timeout 5s ros2 topic info "$topic" 2>/dev/null || true)
    count=$(sed -n 's/^Publisher count: \([0-9][0-9]*\)$/\1/p' <<<"$info")
    count=${count:-0}
    if (( count > 0 )); then
      printf 'ready publisher: %s\n' "$topic"
      return 0
    fi
    sleep 1
  done
  printf 'Timed out waiting for publisher: %s\n' "$topic" >&2
  return 1
}

wait_for_service() {
  local service=$1 timeout_s=${2:-180} started=$SECONDS
  while (( SECONDS - started < timeout_s )); do
    if [[ -n "${STACK_PID:-}" ]] && ! kill -0 "$STACK_PID" 2>/dev/null; then
      printf 'Simulation stack exited while waiting for service: %s\n' \
        "$service" >&2
      return 1
    fi
    if timeout 5s ros2 service type "$service" 2>/dev/null | \
        grep -q '^std_srvs/srv/Trigger$'; then
      printf 'ready service: %s\n' "$service"
      return 0
    fi
    sleep 1
  done
  printf 'Timed out waiting for service: %s\n' "$service" >&2
  return 1
}

require_pcd_points() {
  local path=$1 minimum=$2 label=$3 points
  [[ -s "$path" ]] || {
    printf '%s PCD is missing or empty: %s\n' "$label" "$path" >&2
    return 1
  }
  points=$(awk '$1 == "POINTS" {print $2; exit}' "$path")
  if [[ -z "$points" ]] || (( points < minimum )); then
    printf '%s PCD has too few points (%s, required >= %s): %s\n' \
      "$label" "${points:-unknown}" "$minimum" "$path" >&2
    return 1
  fi
  printf '%s PCD: %s points\n' "$label" "$points"
}

call_export() {
  local service=$1 output=$2 required=$3
  if ! timeout 180s ros2 service call "$service" std_srvs/srv/Trigger '{}' \
      >"$output" 2>&1; then
    printf 'Export service failed: %s\n' "$service" >&2
    return 1
  fi
  if ! grep -Eq 'success:[[:space:]]*true' "$output"; then
    printf 'Export service rejected the request: %s\n' "$service" >&2
    cat "$output" >&2
    return 1
  fi
  [[ -s "$required" ]] || {
    printf 'Export file is missing or empty: %s\n' "$required" >&2
    return 1
  }
}

printf 'Map-fusion demonstration starting.\n  Run: %s\n  Route: %s\n' \
  "$RUN_DIR" "$DEMO_ROUTE"
setsid env \
  RUN_ID="$RUN_ID" RUN_DIR="$RUN_DIR/stack" \
  PR6_HEADLESS=$((1 - DEMO_GUI)) PR6_START_RTABMAP=1 \
  VISUAL_BRIDGE_ENABLED=1 VISUAL_FRONTEND_ENABLED=1 \
  VISUAL_FACTOR_MODE=paper_reprojection VISUAL_KEYFRAME_PROFILE=balanced \
  ONLINE_MAPPING_MODE=joint EXPECT_EXTERNAL_VISUAL_MOTION=1 \
  SIM_RGBD_MIN_DEPTH_M=0.30 SIM_RGBD_MAX_DEPTH_M=10.0 \
  RUN_SMALL_RECTANGLE=0 EXIT_AFTER_RECTANGLE=0 \
  PERFORMANCE_PROFILING_ENABLED=0 REQUIRE_GAZEBO_GPU=1 \
  BACKEND_NUMERIC_THREADS=1 LIDAR_WS="$LIDAR_WS" \
  bash "$STACK_RUNNER" >"$RUN_DIR/stack_supervisor.log" 2>&1 &
STACK_PID=$!
record_group "$STACK_PID"

# Exporters are intentionally launched explicitly. hybridfusion.yaml contains
# both algorithm and ROS-node roots and is not a valid global --params-file.
setsid ros2 run hybridfusion_map_fusion rgbd_map_exporter --ros-args \
  -p use_sim_time:=true -p enabled:=true \
  -p rgb_topic:=/sensors/rgbd/color \
  -p depth_topic:=/sensors/rgbd/depth \
  -p camera_info_topic:=/front/d435i/color/camera_info \
  -p global_frame:=odom \
  -p camera_frame:=front_d435i_color_optical_frame \
  -p output_dir:="$RUN_DIR/export/visual" \
  -p depth_min_m:=0.30 -p depth_max_m:=10.0 \
  -p pixel_stride:=3 -p map_voxel_leaf_m:=0.06 \
  -p save_keyframes:=true \
  >"$RUN_DIR/rgbd_exporter.log" 2>&1 &
RGBD_EXPORT_PID=$!
record_group "$RGBD_EXPORT_PID"

wait_for_publisher /fusion/unified/odom 300
wait_for_publisher /fusion/runtime_external_nav 300
wait_for_publisher /mapping/shared/points 180
wait_for_service /mapping/shared/export 90
wait_for_service /hybridfusion_rgbd_map_exporter/save 90

if [[ "$DEMO_RVIZ" == 1 ]]; then
  setsid rviz2 -d "$RVIZ_CONFIG" --ros-args -p use_sim_time:=true \
    >"$RUN_DIR/rviz.log" 2>&1 &
  RVIZ_PID=$!
  record_group "$RVIZ_PID"
fi

route_environment=(
  LOG_DIR="$RUN_DIR/route"
  LOCALIZATION_SAFETY_ENABLED=true
  LAND_AT_END=true
)
if [[ "$DEMO_ROUTE" == short_s ]]; then
  route_environment+=(
    S_CURVE_SPAN=6.0 S_CURVE_AMPLITUDE=2.0
    S_CURVE_VERTICAL_AMPLITUDE=0.5 S_CURVE_VERTICAL_CYCLES=1
    S_CURVE_PASSES=1 S_CURVE_SPEED=0.35
    S_CURVE_WAYPOINT_SPACING=2.0 S_CURVE_WAYPOINT_HOLD=0.5
    S_CURVE_HOLD_TIME=1.0 TAKEOFF_ALT=4.0 MINIMUM_CLEARANCE_ALT=3.5
  )
fi

set +e
env "${route_environment[@]}" bash "$ROUTE_RUNNER" \
  > >(tee "$RUN_DIR/route_console.log") 2>&1
ROUTE_STATUS=$?
set -e
if (( ROUTE_STATUS != 0 )); then
  printf 'S-curve route failed with status %s; refusing offline success.\n' \
    "$ROUTE_STATUS" >&2
  exit "$ROUTE_STATUS"
fi

call_export /mapping/shared/export "$RUN_DIR/shared_map_export.log" \
  "$RUN_DIR/stack/shared_map/joint_map.pcd"
call_export /hybridfusion_rgbd_map_exporter/save "$RUN_DIR/rgbd_save.log" \
  "$RUN_DIR/export/visual/visual_map.pcd"
require_pcd_points "$RUN_DIR/stack/shared_map/lidar_map.pcd" 100 \
  'Online LiDAR source'
require_pcd_points "$RUN_DIR/stack/shared_map/joint_map.pcd" 100 \
  'Online joint map'
require_pcd_points "$RUN_DIR/export/visual/visual_map.pcd" 100 \
  'Offline visual source'

visual_frame=$(sed -n 's/^global_frame:[[:space:]]*//p' \
  "$RUN_DIR/export/visual/visual_map_metadata.yaml" | tail -1)
lidar_frame=camera_init
[[ -n "$visual_frame" && -n "$lidar_frame" ]] || {
  printf 'Export metadata did not declare both source frames.\n' >&2
  exit 3
}
cat >"$RUN_DIR/export/dataset.yaml" <<EOF
dataset:
  id: map_fusion_demo_${RUN_ID}
  generated_not_measured: false
  visual_map: visual/visual_map.pcd
  lidar_map: ../stack/shared_map/lidar_map.pcd
  visual_frame: ${visual_frame}
  lidar_frame: ${lidar_frame}
  initial_lidar_to_visual: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  initial_transform_provenance: same-start simulation engineering assumption
EOF

set +e
/usr/bin/time -v ros2 run hybridfusion_map_fusion hybridfusion_offline \
  --dataset "$RUN_DIR/export/dataset.yaml" \
  --config "$HYBRID_SHARE/config/hybridfusion.yaml" \
  --method "$OFFLINE_METHOD" --output "$RUN_DIR/offline" \
  >"$RUN_DIR/offline/console.log" 2>"$RUN_DIR/offline/time.log"
OFFLINE_STATUS=$?
set -e
if (( OFFLINE_STATUS != 0 )) || \
   [[ ! -s "$RUN_DIR/offline/fused_map.pcd" ]] || \
   ! grep -Eq '"converged":[[:space:]]*true' "$RUN_DIR/offline/result.json"; then
  printf 'Offline %s did not converge; result is retained but not displayed.\n' \
    "$OFFLINE_METHOD" >&2
  exit 4
fi
require_pcd_points "$RUN_DIR/offline/fused_map.pcd" 100 \
  'Offline fused map'

mapfile -t VISUAL_TF_ARGUMENTS < <(
  python3 "$SCRIPT_DIR/hybridfusion_visual_tf.py" \
    "$RUN_DIR/offline/transform.yaml"
)
setsid ros2 run tf2_ros static_transform_publisher \
  "${VISUAL_TF_ARGUMENTS[@]}" \
  >"$RUN_DIR/offline/static_tf.log" 2>&1 &
TF_PID=$!
record_group "$TF_PID"

setsid ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p use_sim_time:=true \
  -p file_name:="$RUN_DIR/offline/fused_map.pcd" \
  -p tf_frame:=hybridfusion_visual_map \
  -p publishing_period_ms:=1000 \
  -r cloud_pcd:=/mapping/offline/fused \
  >"$RUN_DIR/offline/pcd_publisher.log" 2>&1 &
PCD_PID=$!
record_group "$PCD_PID"
wait_for_publisher /mapping/offline/fused 30

cat <<EOF

Map-fusion demonstration completed.
  Online shared map: $RUN_DIR/stack/shared_map/joint_map.pcd
  Offline fused map: $RUN_DIR/offline/fused_map.pcd
  Offline method:    $OFFLINE_METHOD
  RViz topics:       /mapping/shared/points and /mapping/offline/fused
  Results:           $RUN_DIR

The online result is the current source-aware voxel baseline, not probability
fusion. The offline result is registration + concatenation + voxel filtering.
EOF

if [[ "$KEEP_DEMO_OPEN" == 1 ]]; then
  printf 'Press Ctrl-C to close Gazebo, RViz and all demo-owned processes.\n'
  while kill -0 "$STACK_PID" 2>/dev/null; do sleep 2; done
else
  exit 0
fi
