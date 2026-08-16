
#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
source "$SCRIPT_DIR/d435i_active_run_lifecycle.sh"

TEST_ROOT=$(mktemp -d /tmp/d435i_lifecycle_test.XXXXXX)
owned_pid=""
wrong_pid=""
manifest_pid=""
cleanup_test() {
  for pid in "$owned_pid" "$wrong_pid" "$manifest_pid"; do
    if [[ "$pid" =~ ^[0-9]+$ ]] && kill -0 "$pid" 2>/dev/null; then
      kill -KILL "$pid" 2>/dev/null || true
    fi
  done
  if [[ "$TEST_ROOT" == /tmp/d435i_lifecycle_test.* ]]; then
    rm -rf -- "$TEST_ROOT"
  fi
}
trap cleanup_test EXIT

project_root="$TEST_ROOT/project"
run_dir="$project_root/logs/d435i_visual_slam/lifecycle_test"
marker="$TEST_ROOT/active.env"
mkdir -p "$run_dir"

d435i_run_dir_owned "$run_dir" "$project_root"
d435i_run_dir_owned \
  "$project_root/artifacts/runtime-regression-test/reports/headless" \
  "$project_root"
d435i_run_dir_owned \
  "$project_root/logs/tmp/full_online_capture/online" \
  "$project_root"
if d435i_run_dir_owned "$project_root/tmp/headless" "$project_root"; then
  printf 'run directory outside owned logs/artifacts roots was accepted\n' >&2
  exit 1
fi

token="$$-test"
d435i_active_write "$marker" "$$" "$run_dir" "$project_root" \
  test_branch test_experiment "$token" \
  "$project_root/install/run_d435i_visual_slam_headless.sh"
d435i_active_read "$marker"
[[ "$D435I_ACTIVE_FORMAT" == "2" ]]
[[ "$D435I_ACTIVE_PID" == "$$" ]]
[[ "$D435I_ACTIVE_BRANCH" == "test_branch" ]]
[[ "$D435I_ACTIVE_EXPERIMENT_ID" == "test_experiment" ]]
[[ -n "$D435I_ACTIVE_STARTED_AT_UTC" ]]
[[ -n "$D435I_ACTIVE_PID_START_TICKS" ]]
d435i_active_remove_owned "$marker" "$$" "$token"
[[ ! -e "$marker" ]]

owned_name="$project_root/install/run_d435i_visual_slam_headless.sh"
setsid python3 -c 'import time; time.sleep(30)' "$owned_name" &
owned_pid=$!
owned_ticks=$(d435i_process_start_ticks "$owned_pid")
d435i_active_pid_owned "$owned_pid" "$project_root" "$owned_ticks"
kill -TERM "$owned_pid"
wait "$owned_pid" 2>/dev/null || true
owned_pid=""

setsid sleep 30 &
wrong_pid=$!
wrong_ticks=$(d435i_process_start_ticks "$wrong_pid")
if d435i_active_pid_owned "$wrong_pid" "$project_root" "$wrong_ticks"; then
  printf 'wrong PID was accepted as project-owned\n' >&2
  exit 1
fi
kill -0 "$wrong_pid"

manifest_dir="$project_root/logs/d435i_visual_slam/manifest_cleanup"
mkdir -p "$manifest_dir"
manifest_name="$project_root/install/flight_state_bridge"
setsid python3 -c 'import time; time.sleep(30)' "$manifest_name" &
manifest_pid=$!
manifest_ticks=$(d435i_process_start_ticks "$manifest_pid")
printf 'component\tpid\tprocess_group\tstart_ticks\nflight_state_bridge\t%s\t%s\t%s\n' \
  "$manifest_pid" "$manifest_pid" "$manifest_ticks" \
  >"$manifest_dir/pids.tsv"
d435i_cleanup_run_manifests "$manifest_dir" "$project_root" \
  "$manifest_dir/cleanup.log"
for _ in {1..20}; do
  if ! kill -0 "$manifest_pid" 2>/dev/null; then break; fi
  sleep 0.1
done
if kill -0 "$manifest_pid" 2>/dev/null; then
  printf 'verified manifest process survived cleanup\n' >&2
  exit 1
fi
manifest_pid=""

refusal_dir="$project_root/logs/d435i_visual_slam/refusal"
mkdir -p "$refusal_dir"
printf 'component\tpid\tprocess_group\tstart_ticks\nflight_state_bridge\t%s\t%s\t%s\n' \
  "$wrong_pid" "$wrong_pid" "$wrong_ticks" >"$refusal_dir/pids.tsv"
d435i_cleanup_run_manifests "$refusal_dir" "$project_root" \
  "$refusal_dir/cleanup.log"
kill -0 "$wrong_pid"
grep -q '^REFUSE ' "$refusal_dir/cleanup.log"
kill -TERM "$wrong_pid"
wait "$wrong_pid" 2>/dev/null || true
wrong_pid=""

printf 'D435i active-run lifecycle short test: PASS\n'
