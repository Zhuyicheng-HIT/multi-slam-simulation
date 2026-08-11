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
read -r wrapper_pid headless_run_dir <"$ACTIVE_FILE"
if ! kill -0 "$wrapper_pid" 2>/dev/null; then
  printf 'Recorded headless wrapper is not running: %s\n' "$wrapper_pid" >&2
  exit 2
fi

DURATION_S=${DURATION_S:-60}
if [[ ! "$DURATION_S" =~ ^[0-9]+([.][0-9]+)?$ ]]; then
  printf 'DURATION_S must be a positive number: %s\n' "$DURATION_S" >&2
  exit 2
fi
DURATION_PARAM=$DURATION_S
if [[ "$DURATION_PARAM" != *.* ]]; then
  DURATION_PARAM="${DURATION_PARAM}.0"
fi
PROFILE_LABEL=${PROFILE_LABEL:-d435i_only_headless}
TIMESTAMP=$(date +%Y%m%d_%H%M%S)
OUTPUT_DIR=${OUTPUT_DIR:-$WS_ROOT/logs/d435i_visual_slam/performance/${TIMESTAMP}_${PROFILE_LABEL}}
mkdir -p "$OUTPUT_DIR"

{
  printf 'profile_label=%s\n' "$PROFILE_LABEL"
  printf 'duration_s=%s\n' "$DURATION_S"
  printf 'headless_run_dir=%s\n' "$headless_run_dir"
  printf 'git_commit=%s\n' "$(git -C "$WS_ROOT" rev-parse HEAD)"
  printf 'git_branch=%s\n' "$(git -C "$WS_ROOT" branch --show-current)"
  printf 'git_status_begin\n'
  git -C "$WS_ROOT" status --short
  printf 'git_status_end\n'
} >"$OUTPUT_DIR/run_context.txt"

{
  printf '=== glxinfo -B ===\n'
  if command -v glxinfo >/dev/null 2>&1; then glxinfo -B 2>&1 || true; else printf 'glxinfo unavailable\n'; fi
  printf '\n=== /dev/dxg ===\n'
  ls -l /dev/dxg 2>&1 || true
  printf '\n=== lspci display adapters ===\n'
  lspci 2>&1 | grep -Ei 'vga|3d|display' || true
  printf '\n=== nvidia-smi ===\n'
  if command -v nvidia-smi >/dev/null 2>&1; then nvidia-smi 2>&1 || true; else printf 'nvidia-smi unavailable\n'; fi
  printf '\n=== Gazebo renderer log ===\n'
  grep -Ei 'ogre|render|egl|glx|vulkan|llvmpipe|software' \
    "$headless_run_dir/stack/gazebo.log" 2>/dev/null | tail -n 120 || true
} >"$OUTPUT_DIR/gpu_rendering_audit.txt"

printf 'Profiling %s for %ss -> %s\n' "$PROFILE_LABEL" "$DURATION_S" "$OUTPUT_DIR"
ros2 run multi_slam_uav_sim d435i_pipeline_profiler --ros-args \
  -p duration_s:="$DURATION_PARAM" \
  -p output_dir:="$OUTPUT_DIR" \
  -p image_qos_reliability:="${D435I_PROFILER_QOS_RELIABILITY:-best_effort}" \
  -p image_qos_depth:="${D435I_PROFILER_QOS_DEPTH:-1}" \
  >"$OUTPUT_DIR/profiler.log" 2>&1

rtabmap_log="$headless_run_dir/rtabmap.log"
lost_count=$(grep -Eic 'Odometry lost|lost=true' "$rtabmap_log" 2>/dev/null || true)
reset_count=$(grep -Eic 'Odometry automatically reset|resetting odometry|Odometry reset' "$rtabmap_log" 2>/dev/null || true)
accepted_count=$(grep -Eic 'accepted loop closure|Loop closure.*accepted' "$rtabmap_log" 2>/dev/null || true)
rejected_count=$(grep -Eic 'rejected loop closure|Loop closure.*rejected' "$rtabmap_log" 2>/dev/null || true)
delay_lines=$(grep -Ei 'update time|delay' "$rtabmap_log" 2>/dev/null | tail -n 50 || true)
{
  printf 'odometry_lost_log_count=%s\n' "$lost_count"
  printf 'odometry_reset_log_count=%s\n' "$reset_count"
  printf 'accepted_loop_closure_log_count=%s\n' "$accepted_count"
  printf 'rejected_loop_closure_log_count=%s\n' "$rejected_count"
  printf '%s\n' "$delay_lines"
} >"$OUTPUT_DIR/rtabmap_log_events.txt"

cat >>"$OUTPUT_DIR/summary.md" <<EOF

## RTAB-Map log events

- Odometry lost matches: $lost_count
- Odometry reset matches: $reset_count
- Accepted loop closure matches: $accepted_count
- Rejected loop closure matches: $rejected_count

Raw system, GPU, image, RTAB-Map and trajectory data are stored beside this report.
EOF

printf 'Profile complete: %s\n' "$OUTPUT_DIR"
