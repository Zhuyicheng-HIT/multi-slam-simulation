#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)
source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
set -u

MATRIX_ID=${MATRIX_ID:-$(date +%Y%m%d_%H%M%S)}
MATRIX_DIR=${MATRIX_DIR:-$WS_ROOT/logs/d435i_visual_slam/global_loop_closure/matrix_$MATRIX_ID}
FEATURE_ALIGNMENT_CASES=${FEATURE_ALIGNMENT_CASES:-"A0 A1 B0 B1 C0 C1-run1 C1-run2 C1-run3 R-yaw90 R-small-rectangle"}
ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_feature_alignment.active}
mkdir -p "$MATRIX_DIR"
printf 'case\trtabmap_profile\tmotion_profile\texit_code\toutput_dir\n' >"$MATRIX_DIR/matrix.tsv"

wrapper_pid=""
cleanup_stack() {
  if [[ -n "$wrapper_pid" ]] && kill -0 "$wrapper_pid" 2>/dev/null; then
    kill -INT -- "-$wrapper_pid" 2>/dev/null || kill -INT "$wrapper_pid" 2>/dev/null || true
    for _ in {1..60}; do
      if ! kill -0 "$wrapper_pid" 2>/dev/null; then
        wait "$wrapper_pid" 2>/dev/null || true
        sleep 2
        return 0
      fi
      sleep 0.25
    done
    kill -TERM -- "-$wrapper_pid" 2>/dev/null || kill -TERM "$wrapper_pid" 2>/dev/null || true
    wait "$wrapper_pid" 2>/dev/null || true
    sleep 2
  fi
  return 0
}
trap cleanup_stack EXIT INT TERM

case_parameters() {
  case "$1" in
    A0) printf 'baseline_mismatch stationary\n' ;;
    A1) printf 'feature_aligned stationary\n' ;;
    B0) printf 'baseline_mismatch ag\n' ;;
    B1) printf 'feature_aligned ag\n' ;;
    C0) printf 'baseline_mismatch loop_return\n' ;;
    C1-run1|C1-run2|C1-run3) printf 'feature_aligned loop_return\n' ;;
    R-yaw90) printf 'feature_aligned yaw_90\n' ;;
    R-small-rectangle) printf 'feature_aligned small_rectangle\n' ;;
    *) return 1 ;;
  esac
}

for case_id in $FEATURE_ALIGNMENT_CASES; do
  if ! read -r rtabmap_profile motion_profile < <(case_parameters "$case_id"); then
    printf 'Unknown feature-alignment case: %s\n' "$case_id" >&2
    exit 2
  fi
  case_dir="$MATRIX_DIR/$case_id"
  run_dir="$case_dir/headless"
  output_dir="$case_dir/result"
  mkdir -p "$run_dir" "$output_dir"
  printf '\n=== Starting %s (%s, %s) ===\n' \
    "$case_id" "$rtabmap_profile" "$motion_profile"
  setsid env \
    RUN_ID="feature_${MATRIX_ID}_${case_id}" RUN_DIR="$run_dir" \
    ACTIVE_FILE="$ACTIVE_FILE" RTABMAP_PROFILE="$rtabmap_profile" \
    D435I_WORLD=textured D435I_BRIDGE_IMPL=cpp D435I_DEPTH_ENCODING=16UC1 \
    D435I_QOS_RELIABILITY=reliable D435I_QOS_DEPTH=1 D435I_RTAB_QOS=1 \
    D435I_ENABLE_RTABMAP=1 D435I_START_FLIGHT_STACK=1 \
    GAZEBO_GUI=0 HEADLESS_RENDERING=1 \
    bash "$PKG_SHARE/scripts/run_d435i_visual_slam_headless.sh" \
    >"$case_dir/wrapper.log" 2>&1 &
  wrapper_pid=$!
  ready=0
  for _ in {1..180}; do
    if grep -q 'D435i-only visual SLAM baseline is ready' \
        "$case_dir/wrapper.log" 2>/dev/null; then
      ready=1
      break
    fi
    if ! kill -0 "$wrapper_pid" 2>/dev/null; then break; fi
    sleep 1
  done
  if [[ "$ready" != "1" ]]; then
    printf '%s headless stack did not become ready.\n' "$case_id" >&2
    tail -n 160 "$case_dir/wrapper.log" >&2 || true
    printf '%s\t%s\t%s\t125\t%s\n' \
      "$case_id" "$rtabmap_profile" "$motion_profile" "$output_dir" \
      >>"$MATRIX_DIR/matrix.tsv"
    cleanup_stack
    wrapper_pid=""
    continue
  fi

  set +e
  env ROBUSTNESS_PROFILE="$motion_profile" OUTPUT_DIR="$output_dir" \
    ACTIVE_FILE="$ACTIVE_FILE" RTABMAP_PROFILE="$rtabmap_profile" \
    STATIONARY_S=60.0 \
    bash "$PKG_SHARE/scripts/run_d435i_robustness_profile.sh" \
    >"$case_dir/profile_console.log" 2>&1
  status=$?
  set -e
  cleanup_stack
  wrapper_pid=""

  cp "$run_dir/rtabmap.log" "$output_dir/rtabmap.log"
  if [[ -f "$run_dir/rtabmap.db" ]]; then
    cp "$run_dir/rtabmap.db" "$output_dir/database.db"
    set +e
    ros2 run multi_slam_uav_sim rtabmap_database_diagnostics \
      "$output_dir/database.db" "$output_dir" \
      --loop-csv "$output_dir/loop_closure.csv" \
      >"$output_dir/database_diagnostics.log" 2>&1
    diagnostic_status=$?
    set -e
    if [[ "$diagnostic_status" != "0" ]]; then
      status=$diagnostic_status
    fi
  else
    printf 'Missing per-run database: %s\n' "$run_dir/rtabmap.db" >&2
    status=1
  fi
  printf '%s\t%s\t%s\t%s\t%s\n' \
    "$case_id" "$rtabmap_profile" "$motion_profile" "$status" "$output_dir" \
    >>"$MATRIX_DIR/matrix.tsv"
  sleep 2
done

trap - EXIT INT TERM
printf 'Feature-alignment matrix complete: %s\n' "$MATRIX_DIR"
cat "$MATRIX_DIR/matrix.tsv"
