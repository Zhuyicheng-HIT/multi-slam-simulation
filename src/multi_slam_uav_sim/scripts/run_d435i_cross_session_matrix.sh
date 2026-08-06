#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
source "$PKG_SHARE/scripts/d435i_active_run_lifecycle.sh"
set -u

MATRIX_ID=${MATRIX_ID:-cross_session_pr6_v2_$(date +%Y%m%d_%H%M%S)}
CROSS_SESSION_ROOT=${CROSS_SESSION_ROOT:-$WS_ROOT/logs/d435i_visual_slam/cross_session}
MATRIX_DIR=${MATRIX_DIR:-$CROSS_SESSION_ROOT/matrix_$MATRIX_ID}
CONDITIONS_CONFIG=${CONDITIONS_CONFIG:-$PKG_SHARE/config/d435i_relocalization_conditions.yaml}
CROSS_SESSION_CONDITIONS=${CROSS_SESSION_CONDITIONS:-"start_same start_offset start_yaw_offset start_reverse"}
VALID_RUNS_PER_CONDITION=${VALID_RUNS_PER_CONDITION:-3}
MAX_ATTEMPTS_PER_CONDITION=${MAX_ATTEMPTS_PER_CONDITION:-5}
REQUIRE_SUCCESS=${REQUIRE_SUCCESS:-0}
ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_cross_session.active}
NATIVE_LIDAR_WS=${NATIVE_LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
if [[ ! -f "$NATIVE_LIDAR_WS/install/setup.bash" ]]; then
  printf 'Patched isolated FAST-LIO overlay is missing: %s\n' \
    "$NATIVE_LIDAR_WS/install/setup.bash" >&2
  exit 2
fi
mkdir -p "$MATRIX_DIR"
printf 'condition\tattempt\tstatus\texit_code\trelocalization_success\toutput_dir\n' \
  >"$MATRIX_DIR/matrix.tsv"
printf 'run\tactive_absent\tmanifest_clear\tports_clear\n' \
  >"$MATRIX_DIR/lifecycle_audit.tsv"

wrapper_pid=""
monitor_pid=""
cleanup_stack() {
  if [[ -f "$ACTIVE_FILE" ]]; then
    ACTIVE_FILE="$ACTIVE_FILE" \
      bash "$PKG_SHARE/scripts/stop_d435i_visual_slam_headless.sh" \
      >>"$MATRIX_DIR/stack_cleanup.log" 2>&1 || true
  elif [[ -n "$wrapper_pid" ]] && kill -0 "$wrapper_pid" 2>/dev/null; then
    if d435i_active_pid_owned "$wrapper_pid" "$WS_ROOT"; then
      kill -TERM "$wrapper_pid" 2>/dev/null || true
      for _ in {1..60}; do
        kill -0 "$wrapper_pid" 2>/dev/null || break
        sleep 0.5
      done
    else
      printf 'Refusing unverified wrapper PID: %s\n' "$wrapper_pid" \
        >>"$MATRIX_DIR/stack_cleanup.log"
    fi
  fi
  wait "$wrapper_pid" 2>/dev/null || true
  wrapper_pid=""
  sleep 2
}
stop_monitor() {
  [[ -n "$monitor_pid" ]] || return 0
  if kill -0 "$monitor_pid" 2>/dev/null; then
    command=$(tr '\0' ' ' <"/proc/$monitor_pid/cmdline" 2>/dev/null || true)
    if [[ "$command" == *"d435i_cross_session_monitor"* ]]; then
      kill -INT -- "-$monitor_pid" 2>/dev/null || \
        kill -INT "$monitor_pid" 2>/dev/null || true
      for _ in {1..50}; do
        kill -0 "$monitor_pid" 2>/dev/null || break
        sleep 0.2
      done
    else
      printf 'Refusing unverified monitor PID: %s %s\n' \
        "$monitor_pid" "$command" >>"$MATRIX_DIR/stack_cleanup.log"
    fi
  fi
  wait "$monitor_pid" 2>/dev/null || true
  monitor_pid=""
}
trap 'stop_monitor; cleanup_stack' EXIT INT TERM

wait_ready() {
  local log=$1 pid=$2
  for _ in {1..180}; do
    if grep -q 'PR #6 + D435i visual integration is ready' "$log" 2>/dev/null; then
      return 0
    fi
    kill -0 "$pid" 2>/dev/null || return 1
    sleep 1
  done
  return 1
}

audit_cleanup() {
  local label=$1 run_dir=$2 active_absent=1 manifest_clear=1 ports_clear=1
  [[ ! -f "$ACTIVE_FILE" ]] || active_absent=0
  for manifest in "$run_dir/pids.tsv" "$run_dir/stack_components.tsv"; do
    [[ -f "$manifest" ]] || continue
    while IFS=$'\t' read -r _component pid _rest; do
      [[ "$pid" =~ ^[0-9]+$ ]] || continue
      if kill -0 "$pid" 2>/dev/null; then manifest_clear=0; fi
    done < <(tail -n +2 "$manifest")
  done
  if ss -ltnp 2>/dev/null | grep -Eq ':(5760|5762|14550|14551)\b'; then
    ports_clear=0
  fi
  printf '%s\t%s\t%s\t%s\n' \
    "$label" "$active_absent" "$manifest_clear" "$ports_clear" \
    >>"$MATRIX_DIR/lifecycle_audit.tsv"
  [[ "$active_absent" == "1" && "$manifest_clear" == "1" \
     && "$ports_clear" == "1" ]]
}

start_headless() {
  local run_id=$1 run_dir=$2 enable_rtab=$3 log=$4
  setsid env RUN_ID="$run_id" RUN_DIR="$run_dir" ACTIVE_FILE="$ACTIVE_FILE" \
    LIDAR_WS="$NATIVE_LIDAR_WS" PR6_START_RTABMAP="$enable_rtab" \
    RUN_SMALL_RECTANGLE=0 REQUIRE_GAZEBO_GPU=0 \
    bash "$PKG_SHARE/scripts/run_pr6_d435i_visual_headless.sh" \
    >"$log" 2>&1 &
  wrapper_pid=$!
}

if [[ -z "${REFERENCE_DB:-}" || -z "${REFERENCE_METADATA:-}" ]]; then
  reference_root="$MATRIX_DIR/session1_reference"
  reference_run="$reference_root/headless"
  mkdir -p "$reference_run" "$reference_root/monitor" \
    "$reference_root/flight" "$reference_root/database_diagnostics" \
    "$reference_root/reference"
  start_headless "${MATRIX_ID}_session1" "$reference_run" 1 \
    "$reference_root/wrapper.log"
  if ! wait_ready "$reference_root/wrapper.log" "$wrapper_pid"; then
    tail -n 200 "$reference_root/wrapper.log" \
      >"$reference_root/start_failure_tail.log" || true
    cleanup_stack
    exit 125
  fi
  setsid ros2 run multi_slam_uav_sim d435i_cross_session_monitor --ros-args \
    -p use_sim_time:=true -p mode:=reference -p condition:=session1 \
    -p ground_truth_topic:=/sim/mid360/ground_truth_odom \
    -p output_dir:="$reference_root/monitor" \
    >"$reference_root/monitor.log" 2>&1 &
  monitor_pid=$!
  set +e
  ros2 run multi_slam_uav_sim d435i_speed_envelope_motion --ros-args \
    -p use_sim_time:=true -p speed_test_profile:=long_loop_return \
    -p navigation_source:=gps -p horizontal_speed_mps:=0.20 \
    -p long_route_distance_m:=4.50 -p long_route_design_speed_mps:=0.20 \
    -p long_route_acceleration_s:=2.0 -p long_route_min_steady_s:=3.0 \
    -p takeoff_alt:=0.25 -p takeoff_min_alt_m:=0.15 \
    -p flight_altitude_m:=0.50 \
    -p pre_observation_s:=10.0 -p post_observation_s:=10.0 \
    -p motion_hold_s:=3.0 \
    >"$reference_root/flight_console.log" 2>&1
  reference_flight_exit=$?
  set -e
  stop_monitor
  cleanup_stack
  audit_cleanup session1 "$reference_run" || exit 125
  reference_original="$reference_run/rtabmap.db"
  if [[ "$reference_flight_exit" != "0" || ! -s "$reference_original" ]]; then
    printf 'Session 1 mapping flight or database failed.\n' >&2
    exit 1
  fi
  ros2 run multi_slam_uav_sim rtabmap_database_diagnostics \
    "$reference_original" "$reference_root/database_diagnostics" \
    --loop-csv "$reference_root/monitor/info_events.csv" \
    >"$reference_root/database_diagnostics.log" 2>&1
  ros2 run multi_slam_uav_sim d435i_cross_session_analysis reference \
    "$reference_root/monitor" "$reference_original" \
    "$reference_root/reference" \
    --rtabmap-log "$reference_run/integration_overlay.log" \
    >"$reference_root/reference_analysis.log" 2>&1
  REFERENCE_DB="$reference_root/reference/rtabmap_reference.db"
  cp --reflink=auto -- "$reference_original" "$REFERENCE_DB"
  chmod 0444 -- "$REFERENCE_DB"
  REFERENCE_METADATA="$reference_root/reference/reference_metadata.json"
else
  REFERENCE_DB=$(realpath "$REFERENCE_DB")
  REFERENCE_METADATA=$(realpath "$REFERENCE_METADATA")
fi
reference_hash_initial=$(sha256sum "$REFERENCE_DB" | awk '{print $1}')
metadata_hash=$(python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1]))["database_sha256"])' \
  "$REFERENCE_METADATA")
if [[ "$reference_hash_initial" != "$metadata_hash" ]]; then
  printf 'Reference mother hash does not match its metadata.\n' >&2
  exit 2
fi
cat >"$MATRIX_DIR/reference.env" <<EOF
reference_db=$REFERENCE_DB
reference_metadata=$REFERENCE_METADATA
reference_sha256=$reference_hash_initial
conditions_config=$CONDITIONS_CONFIG
EOF

last_infra_signature=""
consecutive_same_infra=0
for condition in $CROSS_SESSION_CONDITIONS; do
  python3 -c 'import sys,yaml; assert sys.argv[2] in yaml.safe_load(open(sys.argv[1]))["conditions"]' \
    "$CONDITIONS_CONFIG" "$condition"
  valid=0
  attempt=0
  while (( valid < VALID_RUNS_PER_CONDITION )); do
    attempt=$((attempt + 1))
    if (( attempt > MAX_ATTEMPTS_PER_CONDITION )); then
      printf '%s did not produce %s valid runs in %s attempts.\n' \
        "$condition" "$VALID_RUNS_PER_CONDITION" \
        "$MAX_ATTEMPTS_PER_CONDITION" >&2
      exit 1
    fi
    case_dir="$MATRIX_DIR/${condition}_attempt$(printf '%02d' "$attempt")"
    run_dir="$case_dir/headless"
    result_dir="$case_dir/result"
    mkdir -p "$run_dir" "$result_dir"
    printf '\n=== %s attempt=%s valid=%s/%s ===\n' \
      "$condition" "$attempt" "$valid" "$VALID_RUNS_PER_CONDITION"
    start_headless "${MATRIX_ID}_${condition}_a${attempt}" "$run_dir" 0 \
      "$case_dir/wrapper.log"
    if ! wait_ready "$case_dir/wrapper.log" "$wrapper_pid"; then
      reason=stack_not_ready
      tail -n 200 "$case_dir/wrapper.log" \
        >"$case_dir/start_failure_tail.log" || true
      printf '%s\t%s\tINFRA_INVALID\t125\t\t%s\n' \
        "$condition" "$attempt" "$result_dir" >>"$MATRIX_DIR/matrix.tsv"
      cleanup_stack
      audit_cleanup "${condition}_a${attempt}" "$run_dir" || true
      if [[ "$reason" == "$last_infra_signature" ]]; then
        consecutive_same_infra=$((consecutive_same_infra + 1))
      else
        last_infra_signature=$reason
        consecutive_same_infra=1
      fi
      if (( consecutive_same_infra >= 3 )); then
        printf 'Three consecutive identical infrastructure failures: %s\n' \
          "$reason" >&2
        exit 125
      fi
      continue
    fi
    set +e
    env ACTIVE_FILE="$ACTIVE_FILE" CONDITION="$condition" \
      CONDITIONS_CONFIG="$CONDITIONS_CONFIG" REFERENCE_DB="$REFERENCE_DB" \
      REFERENCE_METADATA="$REFERENCE_METADATA" OUTPUT_DIR="$result_dir" \
      bash "$PKG_SHARE/scripts/run_d435i_cross_session_profile.sh" \
      >"$case_dir/profile_console.log" 2>&1
    profile_exit=$?
    set -e
    cleanup_stack
    if ! audit_cleanup "${condition}_a${attempt}" "$run_dir"; then
      status=INFRA_INVALID
      reason=cleanup_incomplete
      profile_exit=125
    else
      status=RUN_INVALID
      reason=analysis_incomplete
    fi
    complete=false
    success=false
    if [[ -f "$result_dir/result/relocalization_summary.json" ]]; then
      complete=$(python3 -c \
        'import json,sys; print(str(bool(json.load(open(sys.argv[1]))["validation_complete"])).lower())' \
        "$result_dir/result/relocalization_summary.json")
      success=$(python3 -c \
        'import json,sys; print(str(bool(json.load(open(sys.argv[1]))["relocalization_success"])).lower())' \
        "$result_dir/result/relocalization_summary.json")
    fi
    if [[ "$profile_exit" == "0" && "$complete" == "true" ]]; then
      if [[ "$REQUIRE_SUCCESS" == "1" && "$success" != "true" ]]; then
        status=ALGORITHM_FAIL
        reason=smoke_relocalization_failed
      else
        status=VALID
        reason=complete
        valid=$((valid + 1))
        consecutive_same_infra=0
        last_infra_signature=""
      fi
    fi
    printf '%s\t%s\t%s\t%s\t%s\t%s\n' \
      "$condition" "$attempt" "$status" "$profile_exit" "$success" \
      "$result_dir" >>"$MATRIX_DIR/matrix.tsv"
  done
done

reference_hash_final=$(sha256sum "$REFERENCE_DB" | awk '{print $1}')
if [[ "$reference_hash_final" != "$reference_hash_initial" ]]; then
  printf 'Read-only reference mother changed during Session 2 matrix.\n' >&2
  exit 2
fi
ros2 run multi_slam_uav_sim d435i_cross_session_comparison "$MATRIX_DIR" \
  --conditions-config "$CONDITIONS_CONFIG"
mkdir -p "$CROSS_SESSION_ROOT"
cp "$MATRIX_DIR/cross_session_validation.csv" "$CROSS_SESSION_ROOT/"
cp "$MATRIX_DIR/cross_session_validation.json" "$CROSS_SESSION_ROOT/"
cp "$MATRIX_DIR/cross_session_validation.md" "$CROSS_SESSION_ROOT/"
trap - EXIT INT TERM
printf 'Cross-session matrix complete: %s\n' "$MATRIX_DIR"
cat "$MATRIX_DIR/matrix.tsv"
