#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
BACKEND_CPU=${BACKEND_CPU:-30}
SIMULATION_CPUSET=${SIMULATION_CPUSET:-0-27}
RUN_ID=${RUN_ID:-performance_v2_affinity_$(date +%Y%m%d_%H%M%S)}
RUN_DIR=${RUN_DIR:-$REPO_ROOT/logs/tmp/$RUN_ID}
HEADLESS_SCRIPT="$REPO_ROOT/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_pr6_d435i_visual_headless.sh"
BACKEND_PATTERN="$REPO_ROOT/install/uf_backend_fusion/lib/uf_backend_fusion/online_backend_fusion"
AFFINITY_GUARD_INTERVAL_S=${AFFINITY_GUARD_INTERVAL_S:-1}

source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
mkdir -p "$RUN_DIR"

if pgrep -f "$BACKEND_PATTERN" >/dev/null 2>&1; then
  printf 'Refusing affinity diagnostic while another backend is active.\n' >&2
  exit 2
fi
available=$(nproc)
if (( BACKEND_CPU < 0 || BACKEND_CPU >= available )); then
  printf 'BACKEND_CPU=%s is outside available CPUs 0-%s.\n' \
    "$BACKEND_CPU" "$((available - 1))" >&2
  exit 2
fi

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
bash "$HEADLESS_SCRIPT" >"$RUN_DIR/headless.log" 2>&1 &
headless_pid=$!
set -e

backend_pid=
for _ in {1..180}; do
  backend_pid=$(pgrep -n -f "$BACKEND_PATTERN" 2>/dev/null || true)
  [[ -n "$backend_pid" ]] && break
  if ! kill -0 "$headless_pid" 2>/dev/null; then
    break
  fi
  sleep 1
done
if [[ -z "$backend_pid" ]]; then
  printf 'Backend did not start; see %s/headless.log.\n' "$RUN_DIR" >&2
  wait "$headless_pid"
  exit $?
fi

printf 'component\tpid\tcpuset\n' >"$RUN_DIR/affinity.tsv"
taskset --all-tasks --pid --cpu-list "$BACKEND_CPU" "$backend_pid" \
  >>"$RUN_DIR/taskset.log" 2>&1
printf 'backend\t%s\t%s\n' "$backend_pid" "$BACKEND_CPU" \
  >>"$RUN_DIR/affinity.tsv"

# This is a diagnostic-only isolation experiment. Restrict only known child
# processes belonging to this just-started stack; never change system policy.
# Several heavy processes start after the backend, so keep a bounded guard
# active for exactly the lifetime of the headless parent instead of sampling
# the process list only once.
is_descendant_of_headless() {
  local pid=$1
  local parent
  while [[ "$pid" =~ ^[0-9]+$ ]] && (( pid > 1 )); do
    [[ "$pid" == "$headless_pid" ]] && return 0
    parent=$(ps -o ppid= -p "$pid" 2>/dev/null | tr -d ' ')
    [[ -n "$parent" && "$parent" != "$pid" ]] || break
    pid=$parent
  done
  return 1
}

declare -A assigned=()
apply_stack_affinity() {
  local pattern pid key
  for pattern in \
    'gz sim' \
    arducopter \
    fastlio_mapping \
    gz_rgbd_latest_bridge \
    rgbd_feature_frontend \
    shared_source_map_node \
    simulation_performance_monitor; do
    while read -r pid; do
      [[ -n "$pid" && "$pid" != "$backend_pid" ]] || continue
      is_descendant_of_headless "$pid" || continue
      key="$pattern:$pid"
      [[ -z "${assigned[$key]:-}" ]] || continue
      taskset --all-tasks --pid --cpu-list "$SIMULATION_CPUSET" "$pid" \
        >>"$RUN_DIR/taskset.log" 2>&1 || continue
      printf '%s\t%s\t%s\n' "$pattern" "$pid" "$SIMULATION_CPUSET" \
        >>"$RUN_DIR/affinity.tsv"
      assigned[$key]=1
    done < <(pgrep -f "$pattern" 2>/dev/null || true)
  done
}

while kill -0 "$headless_pid" 2>/dev/null; do
  apply_stack_affinity
  sleep "$AFFINITY_GUARD_INTERVAL_S"
done &
guard_pid=$!

set +e
wait "$headless_pid"
status=$?
set -e
kill "$guard_pid" 2>/dev/null || true
wait "$guard_pid" 2>/dev/null || true
printf 'status=%s\nbackend_cpu=%s\nsimulation_cpuset=%s\n' \
  "$status" "$BACKEND_CPU" "$SIMULATION_CPUSET" \
  >"$RUN_DIR/affinity_result.env"
exit "$status"
