#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
APPLY=0
KEEP_RUNS=8

usage() {
  cat <<EOF
Usage: bash tools/prune_runtime_cache.sh [--apply] [--keep-runs N]

Without --apply, only prints removable generated data. The script never touches
rosbags, milestone directories, JSON reports, or directories containing .keep.
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --apply)
      APPLY=1
      shift
      ;;
    --keep-runs)
      KEEP_RUNS=${2:?missing value for --keep-runs}
      shift 2
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown argument: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

if [[ ! "$KEEP_RUNS" =~ ^[0-9]+$ ]]; then
  printf '--keep-runs must be a non-negative integer\n' >&2
  exit 2
fi
if [[ ! -d "$REPO_ROOT/.git" || "$REPO_ROOT" == "/" ]]; then
  printf 'Refusing cleanup outside a repository: %s\n' "$REPO_ROOT" >&2
  exit 2
fi

remove_path() {
  local target=$1
  case "$target" in
    "$REPO_ROOT"/log|"$REPO_ROOT"/logs/apm_sensor_stack_*|"$REPO_ROOT"/logs/rectangle_state_machine_*) ;;
    *)
      printf 'Refusing unexpected cleanup target: %s\n' "$target" >&2
      exit 2
      ;;
  esac
  printf '%s  %s\n' "$(du -sh "$target" 2>/dev/null | cut -f1)" "$target"
  if [[ "$APPLY" == "1" ]]; then
    rm -rf -- "$target"
  fi
}

printf '%s colcon logs:\n' "$([[ "$APPLY" == "1" ]] && printf Removing || printf Previewing)"
if [[ -d "$REPO_ROOT/log" ]]; then
  remove_path "$REPO_ROOT/log"
fi

for prefix in apm_sensor_stack_ rectangle_state_machine_; do
  mapfile -t runs < <(
    find "$REPO_ROOT/logs" -mindepth 1 -maxdepth 1 -type d \
      -name "${prefix}*" -printf '%p\n' | sort -r
  )
  for ((index=KEEP_RUNS; index<${#runs[@]}; index++)); do
    run=${runs[$index]}
    if [[ -e "$run/.keep" ]] \
        || find "$run" -type f \( -name '*.db3' -o -name '*.mcap' \
          -o -name 'report.json' -o -name 'simulation_performance.json' \
          -o -name 'externalnav_accuracy.json' \) -print -quit | grep -q .; then
      printf 'KEEP  %s\n' "$run"
      continue
    fi
    remove_path "$run"
  done
done

if [[ "$APPLY" != "1" ]]; then
  printf '\nDry run only. Re-run with --apply to remove the listed paths.\n'
fi
