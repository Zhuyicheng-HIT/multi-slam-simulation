#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
set +u
source /opt/ros/humble/setup.bash
[[ -f "$REPO_ROOT/install/setup.bash" ]] && source "$REPO_ROOT/install/setup.bash"
[[ -n "${LIDAR_WS:-}" && -f "$LIDAR_WS/install/setup.bash" ]] && \
  source "$LIDAR_WS/install/setup.bash"
set -u

mode=${1:-}
output_dir=${2:-frequency_$(date +%Y%m%d_%H%M%S)}
case "$mode" in
  metric1|metric2-full|metric2-no-camera|metric3|metric3-four-source|metric3-five-source) ;;
  *) printf 'Unknown mode: %s\n' "$mode" >&2; exit 2 ;;
esac
mkdir -p "$output_dir"
duration=${FREQUENCY_SAMPLE_SECONDS:-12}

topics=(
  /livox/lidar
  /livox/imu
  /sensors/optical_flow/rad
  /fusion/unified/odom
)
if [[ "$mode" != metric1 ]]; then
  topics+=(/sensors/gnss/fix)
fi
if [[ "$mode" == metric2-full || "$mode" == metric3 || "$mode" == metric3-five-source ]]; then
  topics+=(/sensors/rgbd/color /sensors/rgbd/depth)
fi

pids=()
for topic in "${topics[@]}"; do
  safe=${topic//\//_}
  timeout --signal=INT "${duration}s" ros2 topic hz "$topic" --window 100 \
    >"$output_dir/${safe}.log" 2>&1 &
  pids+=("$!")
done
for pid in "${pids[@]}"; do
  wait "$pid" 2>/dev/null || true
done

printf 'topic\taverage_hz\tstatus\n' >"$output_dir/summary.tsv"
missing=0
for topic in "${topics[@]}"; do
  safe=${topic//\//_}
  log="$output_dir/${safe}.log"
  rate=$(sed -n 's/^average rate: \([0-9.][0-9.]*\).*/\1/p' "$log" | tail -n 1)
  if [[ -n "$rate" ]]; then
    status=OK
  else
    rate=NA
    status=NO_SAMPLES
    missing=$((missing + 1))
  fi
  printf '%s\t%s\t%s\n' "$topic" "$rate" "$status" >>"$output_dir/summary.tsv"
done
cat "$output_dir/summary.tsv"
if (( missing > 0 )); then
  printf '%s required topics produced no frequency sample.\n' "$missing" >&2
  exit 1
fi
