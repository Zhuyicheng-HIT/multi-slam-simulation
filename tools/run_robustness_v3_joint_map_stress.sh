#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
  source "$LIDAR_WS/install/local_setup.bash"
RUN_DIR=${RUN_DIR:-$REPO_ROOT/logs/tmp/robustness_v3_joint_map_stress_$(date +%Y%m%d_%H%M%S)}
# The sensor-stack launcher changes into the ArduPilot checkout while starting
# SITL. Keep every PID, active marker and result path anchored to the project
# instead of letting a caller-supplied relative RUN_DIR change meaning.
mkdir -p "$RUN_DIR"
RUN_DIR=$(realpath "$RUN_DIR")
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-211}
export ROS_DOMAIN_ID
MAP_MODE=${MAP_MODE:-joint}
case "$MAP_MODE" in
  disabled|lidar_only|joint) ;;
  *) printf 'MAP_MODE must be disabled, lidar_only, or joint\n' >&2; exit 2 ;;
esac

set +e
RUN_ID=$(basename "$RUN_DIR") RUN_DIR="$RUN_DIR" \
  RUN_SMALL_RECTANGLE=1 EXIT_AFTER_RECTANGLE=1 \
  RECTANGLE_LENGTH_X=${RECTANGLE_LENGTH_X:-20.0} \
  RECTANGLE_LENGTH_Y=${RECTANGLE_LENGTH_Y:-12.0} \
  RECTANGLE_SPEED_MPS=${RECTANGLE_SPEED_MPS:-0.7} \
  ONLINE_MAPPING_MODE="$MAP_MODE" VISUAL_FACTOR_MODE=paper_reprojection \
  VISUAL_KEYFRAME_PROFILE=balanced \
  EVIDENCE_ROS_DURATION_S=${EVIDENCE_ROS_DURATION_S:-180} \
  EVIDENCE_WALL_TIMEOUT_S=${EVIDENCE_WALL_TIMEOUT_S:-1200} \
  TRAJECTORY_ROS_DURATION_S=${TRAJECTORY_ROS_DURATION_S:-160} \
  TRAJECTORY_WALL_TIMEOUT_S=${TRAJECTORY_WALL_TIMEOUT_S:-1500} \
  PERFORMANCE_PROFILING_ENABLED=${PERFORMANCE_PROFILING_ENABLED:-1} \
  bash "$REPO_ROOT/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_pr6_d435i_visual_headless.sh" \
  >"${RUN_DIR}_driver.log" 2>&1
status=$?
set -e

python3 "$SCRIPT_DIR/summarize_robustness_v3_joint_map.py" \
  --run-dir "$RUN_DIR" --headless-status "$status" \
  --route-x-m "${RECTANGLE_LENGTH_X:-20.0}" \
  --route-y-m "${RECTANGLE_LENGTH_Y:-12.0}" \
  --route-speed-mps "${RECTANGLE_SPEED_MPS:-0.7}" \
  --map-mode "$MAP_MODE" \
  --output "$RUN_DIR/robustness_joint_map_report.json"
cat "$RUN_DIR/robustness_joint_map_report.json"
exit "$status"
