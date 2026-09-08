#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
mode=${1:-}
output=${2:-}
case "$mode" in
  metric1|metric2-full|metric2-no-camera|metric3|metric3-four-source|metric3-five-source) ;;
  *) printf 'Unknown competition mode: %s\n' "$mode" >&2; exit 2 ;;
esac
[[ -n "$output" ]] || {
  output="$HOME/multi-slam-competition-runs/${mode}_$(date +%Y%m%d_%H%M%S)/bag"
}
[[ ! -e "$output" ]] || {
  printf 'Bag output already exists; refusing to overwrite: %s\n' "$output" >&2
  exit 2
}
mkdir -p "$(dirname "$output")"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
  source "$REPO_ROOT/install/setup.bash"
fi
if [[ -n "${LIDAR_WS:-}" && -f "$LIDAR_WS/install/setup.bash" ]]; then
  source "$LIDAR_WS/install/setup.bash"
fi
set -u

qos_file="$REPO_ROOT/src/ultra_fusion_nav/uf_sensor_pipeline/config/rosbag_qos_overrides.yaml"
topics=(
  /clock /tf /tf_static /competition/event
  /livox/lidar /livox/imu
  /sensors/lidar/points_raw /sensors/lidar/points_body_filtered
  /sensors/lidar/points /sensors/lidar/body_removed_ratio
  /sensors/imu /sensors/gnss/fix /sensors/gnss/fix_unthrottled
  /sensors/optical_flow/rad
  /camera/camera/color/image_raw
  /camera/camera/color/camera_info
  /camera/camera/depth/image_rect_raw
  /camera/camera/aligned_depth_to_color/image_raw
  /sensors/rgbd/color /sensors/rgbd/depth
  /mtf01p/micolink /mtf01p/range
  /mavros/state /mavros/local_position/pose /mavros/local_position/odom
  /mavros/global_position/raw/fix /mavros/imu/data_raw
  /fast_lio/native_lidar_factor /Odometry /cloud_registered
  /lio/odom /lio/path /lio/local_map /lio/diagnostics /lidar/points_deskewed
  /fusion/unified/odom /fusion/unified/map_pose /fusion/unified/diagnostics
  /fusion/unified/epoch /mavros/odometry/out
  /reliability/lidar_score /reliability/imu_score /reliability/gnss_score
  /reliability/optical_flow_score /reliability/vision_score
  /reliability/vision_factor_score /reliability/scheduler_state
  /dynamic_observer/static_candidates /dynamic_observer/dynamic_candidates
  /dynamic_observer/unknown_candidates /dynamic_observer/scored_cloud
  /dynamic_observer/statistics /dynamic_observer/latency_diagnostics
  /relocalization/ready /relocalization/request_intent /relocalization/request
  /relocalization/result /safety/active_relocalization_status
  /safety/raw_obstacle_state /autonomy/command_decision /mission/phase
  /fault/state /sensor_contract/diagnostics
)

printf 'Recording competition mode=%s to %s\n' "$mode" "$output"
ros2 bag record --storage sqlite3 --output "$output" \
  --compression-mode file --compression-format zstd \
  --qos-profile-overrides-path "$qos_file" "${topics[@]}" &
recorder_pid=$!
recorder_status=0
stopping=false

stop_recorder() {
  if [[ "$stopping" == true ]]; then
    return
  fi
  stopping=true
  if kill -0 "$recorder_pid" 2>/dev/null; then
    "$SCRIPT_DIR/competition_event.py" --mode "$mode" --event RECORDING_STOP \
      --detail operator_stop --repeat 10 --period 0.03 2>/dev/null || true
    kill -INT "$recorder_pid" 2>/dev/null || true
  fi
  set +e
  wait "$recorder_pid"
  recorder_status=$?
  set -e
}
trap stop_recorder INT TERM

deadline=$((SECONDS + 15))
while (( SECONDS < deadline )); do
  if ros2 node list --no-daemon 2>/dev/null | grep -q '/rosbag2_recorder'; then
    break
  fi
  kill -0 "$recorder_pid" 2>/dev/null || break
  sleep 0.5
done
kill -0 "$recorder_pid" 2>/dev/null || {
  wait "$recorder_pid" || true
  printf 'rosbag2 recorder exited before becoming ready.\n' >&2
  exit 1
}
"$SCRIPT_DIR/competition_event.py" --mode "$mode" --event RECORDING_START \
  --detail "$(basename "$(dirname "$output")")" --repeat 20 --period 0.03

set +e
wait "$recorder_pid"
recorder_status=$?
set -e
trap - INT TERM
if [[ "$stopping" == false ]]; then
  stopping=true
fi

metadata="$output/metadata.yaml"
[[ -s "$metadata" ]] || {
  printf 'Recording is incomplete: metadata.yaml is missing or empty.\n' >&2
  exit 1
}
ros2 bag info "$output" >"$(dirname "$output")/bag_info.txt"
python3 "$SCRIPT_DIR/verify_competition_bag.py" "$mode" "$metadata" \
  --output "$(dirname "$output")/bag_validation.json"
(
  cd "$(dirname "$output")"
  find "$(basename "$output")" -maxdepth 1 -type f -print0 | sort -z | \
    xargs -0 sha256sum >SHA256SUMS.txt
)
printf 'Bag closed and verified: %s\n' "$output"
# SIGINT is the documented operator stop path. Treat it as success only after
# rosbag metadata, topic evidence and checksums have all been verified above.
if [[ "$recorder_status" == 130 ]]; then
  recorder_status=0
fi
exit "$recorder_status"
