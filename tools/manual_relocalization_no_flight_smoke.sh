#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
[[ -n "${LIDAR_WS:-}" && -f "$LIDAR_WS/install/setup.bash" ]] && \
  source "$LIDAR_WS/install/setup.bash"
set -u

# A private domain and localhost-only discovery ensure this smoke cannot reach
# a running MAVROS/FCU graph. It validates state ownership, not flight success.
export ROS_DOMAIN_ID=${MANUAL_RELOC_SMOKE_DOMAIN_ID:-$((100 + $$ % 100))}
export ROS_LOCALHOST_ONLY=1
run_dir=${1:-/tmp/manual-relocalization-no-flight-$ROS_DOMAIN_ID-$$}
mkdir -p "$run_dir"
pids=()

cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "${pids[@]}"; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  for pid in "${pids[@]}"; do wait "$pid" 2>/dev/null || true; done
  exit "$status"
}
trap cleanup EXIT INT TERM

start() {
  local name=$1
  shift
  setsid "$@" >"$run_dir/$name.log" 2>&1 &
  pids+=("$!")
}

start safety ros2 launch uf_safety_supervisor safety_slice.launch.py \
  use_sim_time:=false raw_lidar_topic:=/livox/lidar
start request_arbiter ros2 run uf_reliability relocalization_request_arbiter
start manual_control ros2 run uf_reliability manual_relocalization_control
start active_controller ros2 run uf_relocalization active_relocalization_controller

deadline=$((SECONDS + 20))
while (( SECONDS < deadline )); do
  if ros2 service type /relocalization/manual_control 2>/dev/null | \
      grep -qx 'uf_interfaces/srv/ManualRelocalization'; then
    break
  fi
  sleep 0.5
done
ros2 service type /relocalization/manual_control 2>/dev/null | \
  grep -qx 'uf_interfaces/srv/ManualRelocalization' || {
    printf 'Manual service did not become ready. Logs: %s\n' "$run_dir" >&2
    exit 1
  }

ros2 run uf_relocalization active_relocalization_ros_smoke.py \
  --mode success --request-source manual_service | tee "$run_dir/result.json"
printf 'No-flight control-path smoke passed in isolated ROS domain %s.\n' "$ROS_DOMAIN_ID"
printf 'This does not prove real relocalization accuracy or flight recovery.\n'
