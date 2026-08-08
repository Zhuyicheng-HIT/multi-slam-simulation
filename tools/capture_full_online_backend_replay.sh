#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
RUN_ID=${RUN_ID:-full_online_backend_capture_$(date +%Y%m%d_%H%M%S)}
RUN_DIR=${RUN_DIR:-$REPO_ROOT/logs/tmp/$RUN_ID}
BAG_DIR=${BAG_DIR:-$RUN_DIR/full_online_backend_replay}

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
mkdir -p "$RUN_DIR"

pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${pids[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

# Store only estimator inputs and the scan-trajectory handshake. Camera images,
# maps, simulation truth, and diagnostics are intentionally excluded. The bag
# remains under logs/tmp and must never be committed.
setsid ros2 bag record --storage sqlite3 --output "$BAG_DIR" \
  /clock \
  /fast_lio/frontend_scan_request \
  /fast_lio/native_lidar_factor \
  /sensors/imu \
  /sensors/gnss/fix \
  /sensors/optical_flow/rad \
  /vision/feature_tracks \
  /reliability/scheduler_state \
  /reliability/lidar_score \
  /reliability/imu_score \
  /reliability/gnss_score \
  /reliability/optical_flow_score \
  /reliability/vision_score \
  /calibration/lidar_relative_motion \
  >"$RUN_DIR/rosbag_record.log" 2>&1 &
pids+=("$!")
sleep 3

set +e
RUN_ID="$RUN_ID" \
RUN_DIR="$RUN_DIR/online" \
RUN_SMALL_RECTANGLE=1 \
EXIT_AFTER_RECTANGLE=1 \
PR6_START_RTABMAP=0 \
VISUAL_FACTOR_MODE=paper_reprojection \
VISUAL_KEYFRAME_PROFILE=balanced \
VISUAL_CANDIDATE_QUALITY_ENABLED=1 \
VISUAL_PENDING_ENABLED=1 \
PERFORMANCE_PROFILING_ENABLED=1 \
ONLINE_MAPPING_MODE=joint \
EVIDENCE_ROS_DURATION_S=240 \
EVIDENCE_WALL_TIMEOUT_S=1200 \
TRAJECTORY_ROS_DURATION_S=240 \
TRAJECTORY_WALL_TIMEOUT_S=1200 \
bash "$REPO_ROOT/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_pr6_d435i_visual_headless.sh" \
  >"$RUN_DIR/headless_capture.log" 2>&1
headless_status=$?
set -e

cleanup
trap - EXIT INT TERM
sleep 3
ros2 bag info "$BAG_DIR" >"$RUN_DIR/bag_info.txt"
printf 'capture_status=%s\nbag=%s\n' "$headless_status" "$BAG_DIR" \
  >"$RUN_DIR/capture_result.env"
cat "$RUN_DIR/bag_info.txt"
exit "$headless_status"
