#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
action=${1:-status}
set +u
source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"
[[ -n "${LIDAR_WS:-}" && -f "$LIDAR_WS/install/setup.bash" ]] && \
  source "$LIDAR_WS/install/setup.bash"
set -u

service=/relocalization/manual_control
service_type=uf_interfaces/srv/ManualRelocalization
episode_id=${MANUAL_RELOCALIZATION_EPISODE_ID:-$(date +%s%N)}
lease_s=${MANUAL_RELOCALIZATION_LEASE_S:-1.0}

call_service() {
  local command=$1 event=$2
  ros2 service type "$service" 2>/dev/null | grep -qx "$service_type" || {
    printf 'Manual relocalization service is not ready: %s\n' "$service" >&2
    return 1
  }
  "$SCRIPT_DIR/competition_event.py" --mode metric3 --event "$event" \
    --detail "episode=$episode_id" --repeat 10 --period 0.03 || true
  ros2 service call "$service" "$service_type" \
    "{command: $command, source: manual_control, episode_id: $episode_id, timestamp: {sec: 0, nanosec: 0}, lease_duration_s: $lease_s}"
}

case "$action" in
  start)
    ready=$(timeout 5s ros2 topic echo /relocalization/ready std_msgs/msg/Bool \
      --once --no-daemon 2>/dev/null || true)
    grep -Eq 'data: true' <<<"$ready" || {
      printf 'Database is not ready; START refused.\n' >&2
      exit 1
    }
    call_service 1 MANUAL_RELOCALIZATION_START
    ;;
  cancel)
    call_service 2 MANUAL_RELOCALIZATION_CANCEL
    ;;
  status)
    printf '%s\n' '--- database ready ---'
    timeout 3s ros2 topic echo /relocalization/ready std_msgs/msg/Bool \
      --once --no-daemon || true
    printf '%s\n' '--- active controller ---'
    timeout 3s ros2 topic echo /safety/active_relocalization_status \
      uf_interfaces/msg/ActiveRelocalizationStatus --once --no-daemon || true
    printf '%s\n' '--- latest result ---'
    timeout 3s ros2 topic echo /relocalization/result \
      uf_interfaces/msg/RelocalizationResult --once --no-daemon || true
    ;;
  *)
    printf 'Usage: %s {start|cancel|status}\n' "$0" >&2
    exit 2
    ;;
esac
