
#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
source "$PKG_SHARE/scripts/d435i_active_run_lifecycle.sh"
set -u

ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_visual_slam.active}
if [[ ! -f "$ACTIVE_FILE" ]]; then
  printf 'No active D435i visual SLAM headless run is recorded.\n'
  exit 0
fi

if ! d435i_active_read "$ACTIVE_FILE"; then
  printf 'Invalid active-run record; refusing unverified process cleanup: %s\n' \
    "$ACTIVE_FILE" >&2
  exit 2
fi
wrapper_pid=$D435I_ACTIVE_PID
run_dir=$D435I_ACTIVE_RUN_DIR
run_token=$D435I_ACTIVE_RUN_TOKEN
project_root=${D435I_ACTIVE_PROJECT_ROOT:-$WS_ROOT}

if [[ "$(realpath -m "$project_root")" != "$(realpath -m "$WS_ROOT")" ]]; then
  printf 'Marker project root does not match this checkout; refusing cleanup: %s\n' \
    "$project_root" >&2
  exit 3
fi
if ! d435i_run_dir_owned "$run_dir" "$project_root"; then
  printf 'Refusing unexpected run directory: %s\n' "$run_dir" >&2
  exit 3
fi

lifecycle_dir="$run_dir/lifecycle"
mkdir -p "$lifecycle_dir"
wrapper_was_running=0
if kill -0 "$wrapper_pid" 2>/dev/null; then
  wrapper_was_running=1
  if ! d435i_active_pid_owned "$wrapper_pid" "$project_root" \
      "$D435I_ACTIVE_PID_START_TICKS"; then
    printf 'Marker PID exists but does not belong to this project wrapper; it will not be killed and the marker will be retained: %s\n' \
      "$wrapper_pid" >&2
    exit 3
  fi
  printf 'Stopping verified D435i headless wrapper pid=%s experiment=%s\n' \
    "$wrapper_pid" "${D435I_ACTIVE_EXPERIMENT_ID:-unknown}"
  kill -TERM "$wrapper_pid"
  for _ in {1..30}; do
    if ! kill -0 "$wrapper_pid" 2>/dev/null; then break; fi
    sleep 0.5
  done
  if kill -0 "$wrapper_pid" 2>/dev/null; then
    if d435i_active_pid_owned "$wrapper_pid" "$project_root" \
        "$D435I_ACTIVE_PID_START_TICKS"; then
      printf 'Verified wrapper did not exit after TERM; sending KILL to exact PID %s\n' \
        "$wrapper_pid" | tee -a "$lifecycle_dir/external_stop.log"
      kill -KILL "$wrapper_pid"
    else
      printf 'Wrapper identity changed while stopping; refusing KILL.\n' >&2
      exit 3
    fi
  fi
else
  d435i_active_archive "$ACTIVE_FILE" "$lifecycle_dir/stale_markers" \
    external_stop_wrapper_missing
  printf 'Wrapper PID is absent; archived stale marker to %s\n' \
    "$D435I_ACTIVE_ARCHIVE_PATH"
fi

d435i_cleanup_run_manifests \
  "$run_dir" "$project_root" "$lifecycle_dir/external_stop_cleanup.log"

if [[ -f "$ACTIVE_FILE" ]]; then
  if [[ -n "$run_token" ]]; then
    if ! d435i_active_remove_owned "$ACTIVE_FILE" "$wrapper_pid" "$run_token"; then
      printf 'Active marker ownership changed; refusing removal: %s\n' \
        "$ACTIVE_FILE" >&2
      exit 3
    fi
  elif [[ "$wrapper_was_running" == "0" || ! -e "/proc/$wrapper_pid" ]]; then
    rm -f -- "$ACTIVE_FILE"
  fi
fi
printf 'D435i visual SLAM headless profile stopped. Logs: %s\n' "$run_dir"
