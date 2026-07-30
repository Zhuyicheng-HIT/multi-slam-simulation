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
MATRIX_DIR=${MATRIX_DIR:-$WS_ROOT/logs/d435i_visual_slam/robustness/matrix_$MATRIX_ID}
ROBUSTNESS_PROFILES=${ROBUSTNESS_PROFILES:-"t0 stationary hover ag yaw_30 yaw_90 straight l_shape single_corner small_rectangle loop_return"}
ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_visual_slam.active}
mkdir -p "$MATRIX_DIR"
printf 'profile\texit_code\toutput_dir\n' >"$MATRIX_DIR/matrix.tsv"

wrapper_pid=""
cleanup_stack() {
  if [[ -n "$wrapper_pid" ]] && kill -0 "$wrapper_pid" 2>/dev/null; then
    kill -INT -- "-$wrapper_pid" 2>/dev/null || kill -INT "$wrapper_pid" 2>/dev/null || true
    for _ in {1..40}; do
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

for profile in $ROBUSTNESS_PROFILES; do
  run_dir="$MATRIX_DIR/${profile}/headless"
  output_dir="$MATRIX_DIR/${profile}/result"
  mkdir -p "$run_dir" "$output_dir"
  if [[ "$profile" == "t0" ]]; then
    enable_rtab=0
    start_flight=0
  else
    enable_rtab=1
    start_flight=1
  fi
  printf '\n=== Starting profile %s ===\n' "$profile"
  setsid env \
    RUN_ID="matrix_${MATRIX_ID}_${profile}" \
    RUN_DIR="$run_dir" \
    ACTIVE_FILE="$ACTIVE_FILE" \
    D435I_WORLD=textured \
    D435I_BRIDGE_IMPL=cpp \
    D435I_DEPTH_ENCODING=16UC1 \
    D435I_QOS_RELIABILITY=reliable \
    D435I_QOS_DEPTH=1 \
    D435I_RTAB_QOS=1 \
    D435I_ENABLE_RTABMAP="$enable_rtab" \
    D435I_START_FLIGHT_STACK="$start_flight" \
    GAZEBO_GUI=0 HEADLESS_RENDERING=1 \
    bash "$PKG_SHARE/scripts/run_d435i_visual_slam_headless.sh" \
    >"$MATRIX_DIR/${profile}/wrapper.log" 2>&1 &
  wrapper_pid=$!
  ready=0
  for _ in {1..150}; do
    if grep -q 'D435i-only visual SLAM baseline is ready' \
        "$MATRIX_DIR/${profile}/wrapper.log" 2>/dev/null; then
      ready=1
      break
    fi
    if ! kill -0 "$wrapper_pid" 2>/dev/null; then
      break
    fi
    sleep 1
  done
  if [[ "$ready" != "1" ]]; then
    printf 'Profile %s headless stack did not become ready.\n' "$profile" >&2
    tail -n 120 "$MATRIX_DIR/${profile}/wrapper.log" >&2 || true
    printf '%s\t%s\t%s\n' "$profile" 125 "$output_dir" >>"$MATRIX_DIR/matrix.tsv"
    cleanup_stack
    wrapper_pid=""
    continue
  fi

  set +e
  robustness_env=(
    ROBUSTNESS_PROFILE="$profile"
    OUTPUT_DIR="$output_dir"
    ACTIVE_FILE="$ACTIVE_FILE"
  )
  if [[ "$profile" == "hover" ]]; then
    robustness_env+=(HOVER_S="${T2_HOVER_S:-60.0}")
  fi
  env "${robustness_env[@]}" \
    bash "$PKG_SHARE/scripts/run_d435i_robustness_profile.sh" \
    >"$MATRIX_DIR/${profile}/profile_console.log" 2>&1
  status=$?
  set -e
  printf '%s\t%s\t%s\n' "$profile" "$status" "$output_dir" \
    >>"$MATRIX_DIR/matrix.tsv"
  cleanup_stack
  wrapper_pid=""
  sleep 2
done

trap - EXIT INT TERM
printf 'Matrix complete: %s\n' "$MATRIX_DIR"
cat "$MATRIX_DIR/matrix.tsv"
