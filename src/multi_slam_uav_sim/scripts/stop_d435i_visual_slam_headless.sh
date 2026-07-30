#!/usr/bin/env bash
set -eo pipefail

set -u

ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_visual_slam.active}
if [[ ! -f "$ACTIVE_FILE" ]]; then
  printf 'No active D435i visual SLAM headless run is recorded.\n'
  exit 0
fi

read -r wrapper_pid run_dir <"$ACTIVE_FILE" || true
if [[ -z "${wrapper_pid:-}" || -z "${run_dir:-}" ]]; then
  printf 'Invalid active-run record: %s\n' "$ACTIVE_FILE" >&2
  exit 2
fi

case "$run_dir" in
  */logs/d435i_visual_slam/headless/*) ;;
  *)
    printf 'Refusing unexpected run directory: %s\n' "$run_dir" >&2
    exit 2
    ;;
esac

if kill -0 "$wrapper_pid" 2>/dev/null; then
  printf 'Stopping D435i headless wrapper pid=%s\n' "$wrapper_pid"
  kill -TERM "$wrapper_pid"
  for _ in {1..30}; do
    if ! kill -0 "$wrapper_pid" 2>/dev/null; then break; fi
    sleep 0.5
  done
fi

recorded_components=()
recorded_pids=()
recorded_groups=()
load_manifest() {
  local manifest=$1
  local has_group=$2
  [[ -f "$manifest" ]] || return
  mapfile -t records < <(tail -n +2 "$manifest")
  for ((index=${#records[@]}-1; index>=0; index--)); do
    IFS=$'\t' read -r component pid process_group <<<"${records[$index]}"
    [[ -n "${pid:-}" ]] || continue
    if [[ "$has_group" != "1" || -z "${process_group:-}" ]]; then
      process_group=$pid
    fi
    recorded_components+=("$component")
    recorded_pids+=("$pid")
    recorded_groups+=("$process_group")
  done
}

load_manifest "$run_dir/pids.tsv" 1
load_manifest "$run_dir/stack_components.tsv" 0

signal_recorded() {
  local signal=$1
  for index in "${!recorded_pids[@]}"; do
    pid=${recorded_pids[$index]}
    process_group=${recorded_groups[$index]}
    component=${recorded_components[$index]}
    if ! kill -0 "$pid" 2>/dev/null; then continue; fi
    actual_group=$(ps -o pgid= -p "$pid" 2>/dev/null | tr -d ' ')
    if [[ "$actual_group" != "$process_group" ]]; then
      printf 'Skipping reused/unexpected PID %s for %s (pgid=%s expected=%s)\n' \
        "$pid" "$component" "${actual_group:-none}" "$process_group" >&2
      continue
    fi
    printf 'Sending %-4s to recorded %-24s pid=%s\n' \
      "$signal" "$component" "$pid"
    kill -"$signal" -- "-$process_group" 2>/dev/null || \
      kill -"$signal" "$pid" 2>/dev/null || true
  done
}

signal_recorded INT
sleep 2
signal_recorded TERM
sleep 2
signal_recorded KILL

rm -f "$ACTIVE_FILE"
printf 'D435i visual SLAM headless profile stopped. Logs: %s\n' "$run_dir"
