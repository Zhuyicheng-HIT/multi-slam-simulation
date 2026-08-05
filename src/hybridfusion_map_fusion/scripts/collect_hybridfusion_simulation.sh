#!/usr/bin/env bash
set -Eeo pipefail

if (( $# != 1 )); then
  printf 'Usage: %s /absolute/output/directory\n' "$0" >&2
  exit 2
fi
case "$1" in
  /*) ;;
  *) printf 'Output directory must be absolute: %s\n' "$1" >&2; exit 2 ;;
esac

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WS_INSTALL=$(cd "$SCRIPT_DIR/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
OUTPUT_ROOT=$(realpath -m "$1")
ACTIVE_FILE=${ACTIVE_FILE:-$WS_ROOT/logs/d435i_visual_slam/.active_headless}
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}

if [[ "$OUTPUT_ROOT" == / ]]; then
  printf 'Unsafe output path: %s\n' "$OUTPUT_ROOT" >&2
  exit 2
fi
if [[ -e "$ACTIVE_FILE" ]]; then
  printf 'Refusing to overlap an existing headless run: %s\n' "$ACTIVE_FILE" >&2
  exit 2
fi

source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
if [[ -f "$LIDAR_WS/install/setup.bash" ]]; then
  source "$LIDAR_WS/install/setup.bash"
fi
SIM_PREFIX=$(ros2 pkg prefix multi_slam_uav_sim)
SIM_SHARE="$SIM_PREFIX/share/multi_slam_uav_sim"
if [[ ! -x "$SIM_SHARE/scripts/run_pr6_d435i_visual_headless.sh" ]]; then
  printf 'Missing installed PR #6/PR #8 headless runner: %s\n' "$SIM_SHARE" >&2
  exit 2
fi

mkdir -p "$OUTPUT_ROOT" "$OUTPUT_ROOT/stack"
EXPORT_LOG="$OUTPUT_ROOT/exporters.log"
RUN_ID="hybridfusion_collection_$(date +%Y%m%d_%H%M%S)"
exporter_pid=
stack_pid=
cleanup_started=0

stop_group() {
  local pid=${1:-}
  [[ -n "$pid" ]] || return 0
  if kill -0 "$pid" 2>/dev/null; then
    kill -INT -- "-$pid" 2>/dev/null || true
    timeout 15s tail --pid="$pid" -f /dev/null 2>/dev/null || true
  fi
}

cleanup() {
  local status=$?
  [[ "$cleanup_started" == 0 ]] || return
  cleanup_started=1
  trap - EXIT INT TERM
  stop_group "$stack_pid"
  stop_group "$exporter_pid"
  exit "$status"
}
trap cleanup EXIT INT TERM

# Start the opt-in exporters first so they cannot miss the beginning of the
# existing route. They own no simulator, flight, backend, TF or map publisher.
setsid ros2 launch hybridfusion_map_fusion hybridfusion_export.launch.py \
  enabled:=true output_root:="$OUTPUT_ROOT" >"$EXPORT_LOG" 2>&1 &
exporter_pid=$!

setsid env \
  RUN_ID="$RUN_ID" RUN_DIR="$OUTPUT_ROOT/stack" ACTIVE_FILE="$ACTIVE_FILE" \
  LIDAR_WS="$LIDAR_WS" PR6_START_RTABMAP=1 RUN_SMALL_RECTANGLE=1 \
  EXIT_AFTER_RECTANGLE=1 REQUIRE_GAZEBO_GPU=${REQUIRE_GAZEBO_GPU:-0} \
  bash "$SIM_SHARE/scripts/run_pr6_d435i_visual_headless.sh" \
  >"$OUTPUT_ROOT/stack_supervisor.log" 2>&1 &
stack_pid=$!

set +e
wait "$stack_pid"
stack_status=$?
set -e
stack_pid=

rgbd_status=1
lidar_status=1
if timeout 20s ros2 service call /hybridfusion_rgbd_map_exporter/save \
    std_srvs/srv/Trigger '{}' >"$OUTPUT_ROOT/rgbd_save.log" 2>&1 && \
    [[ -s "$OUTPUT_ROOT/visual/visual_map.pcd" ]]; then
  rgbd_status=0
fi
if timeout 20s ros2 service call /hybridfusion_lidar_map_exporter/save \
    std_srvs/srv/Trigger '{}' >"$OUTPUT_ROOT/lidar_save.log" 2>&1 && \
    [[ -s "$OUTPUT_ROOT/lidar/lidar_map.pcd" ]]; then
  lidar_status=0
fi

if [[ -s "$OUTPUT_ROOT/visual/visual_map.pcd" && \
      -s "$OUTPUT_ROOT/lidar/lidar_map.pcd" ]]; then
  printf '%s\n' \
    'dataset:' \
    '  id: hybridfusion_live_building_route' \
    '  generated_not_measured: false' \
    '  visual_map: visual/visual_map.pcd' \
    '  lidar_map: lidar/lidar_map.pcd' \
    '  visual_frame: odom' \
    '  lidar_frame: camera_init' \
    '  initial_lidar_to_visual: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]' \
    '  initial_transform_provenance: same-start simulation engineering assumption; verify before use' \
    >"$OUTPUT_ROOT/dataset.yaml"
fi

printf 'stack_exit=%s\nrgbd_save_exit=%s\nlidar_save_exit=%s\n' \
  "$stack_status" "$rgbd_status" "$lidar_status" \
  >"$OUTPUT_ROOT/collection_result.env"

if (( stack_status != 0 || rgbd_status != 0 || lidar_status != 0 )); then
  printf 'HybridFusion collection failed; inspect %s\n' "$OUTPUT_ROOT" >&2
  exit 1
fi
printf 'HybridFusion source maps saved under %s\n' "$OUTPUT_ROOT"
