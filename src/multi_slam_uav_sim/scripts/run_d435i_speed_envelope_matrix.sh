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
SPEED_ROOT=${SPEED_ROOT:-$WS_ROOT/logs/d435i_visual_slam/speed_envelope}
MATRIX_DIR=${MATRIX_DIR:-$SPEED_ROOT/matrix_$MATRIX_ID}
SPEED_ENVELOPE_CASES=${SPEED_ENVELOPE_CASES:-"H0 H1 H2 H3 H4 Y0 Y1 Y2 Y3 Y4 V0 V1 V2 V3 C0 C1 C2 C3 C4 R0 R1 R2 R3 L0 L1 L2 L3"}
ACTIVE_FILE=${ACTIVE_FILE:-/tmp/multi_slam_d435i_speed_envelope.active}
mkdir -p "$MATRIX_DIR"
printf 'case\tfamily\tprofile\thorizontal_mps\tvertical_mps\tyaw_dps\texit_code\tclassification\toutput_dir\n' >"$MATRIX_DIR/matrix.tsv"

wrapper_pid=""
cleanup_stack() {
  if [[ -n "$wrapper_pid" ]] && kill -0 "$wrapper_pid" 2>/dev/null; then
    kill -INT -- "-$wrapper_pid" 2>/dev/null || kill -INT "$wrapper_pid" 2>/dev/null || true
    for _ in {1..80}; do
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
    H0) printf 'H horizontal 0.10 0.10 8\n' ;;
    H1) printf 'H horizontal 0.20 0.10 8\n' ;;
    H2) printf 'H horizontal 0.35 0.10 8\n' ;;
    H3) printf 'H horizontal 0.50 0.10 8\n' ;;
    H4) printf 'H horizontal 0.75 0.10 8\n' ;;
    Y0) printf 'Y yaw 0.10 0.10 8\n' ;;
    Y1) printf 'Y yaw 0.10 0.10 15\n' ;;
    Y2) printf 'Y yaw 0.10 0.10 25\n' ;;
    Y3) printf 'Y yaw 0.10 0.10 40\n' ;;
    Y4) printf 'Y yaw 0.10 0.10 60\n' ;;
    V0) printf 'V vertical 0.10 0.10 8\n' ;;
    V1) printf 'V vertical 0.10 0.20 8\n' ;;
    V2) printf 'V vertical 0.10 0.35 8\n' ;;
    V3) printf 'V vertical 0.10 0.50 8\n' ;;
    C0) printf 'C combined 0.20 0.10 15\n' ;;
    C1) printf 'C combined 0.35 0.10 15\n' ;;
    C2) printf 'C combined 0.35 0.10 25\n' ;;
    C3) printf 'C combined 0.50 0.10 25\n' ;;
    C4) printf 'C combined 0.50 0.10 40\n' ;;
    R0) printf 'R small_rectangle 0.10 0.10 15\n' ;;
    R1) printf 'R small_rectangle 0.20 0.10 15\n' ;;
    R2) printf 'R small_rectangle 0.35 0.10 15\n' ;;
    R3) printf 'R small_rectangle 0.50 0.10 15\n' ;;
    L0) printf 'L loop_return 0.10 0.10 15\n' ;;
    L1) printf 'L loop_return 0.20 0.10 15\n' ;;
    L2) printf 'L loop_return 0.35 0.10 15\n' ;;
    L3) printf 'L loop_return 0.50 0.10 15\n' ;;
    *) return 1 ;;
  esac
}

declare -A stopped_family
for case_id in $SPEED_ENVELOPE_CASES; do
  if ! read -r family profile horizontal vertical yaw < <(case_parameters "$case_id"); then
    printf 'Unknown speed-envelope case: %s\n' "$case_id" >&2
    exit 2
  fi
  if [[ "${stopped_family[$family]:-0}" == "1" ]]; then
    case_dir="$MATRIX_DIR/${case_id}_${profile}_not_exercised"
    mkdir -p "$case_dir"
    printf 'Skipped because an earlier %s case failed.\n' "$family" >"$case_dir/NOT_EXERCISED.txt"
    printf '%s\t%s\t%s\t%s\t%s\t%s\t0\tNOT_EXERCISED\t%s\n' \
      "$case_id" "$family" "$profile" "$horizontal" "$vertical" "$yaw" "$case_dir" \
      >>"$MATRIX_DIR/matrix.tsv"
    continue
  fi

  target=${horizontal}mps
  if [[ "$profile" == "yaw" ]]; then
    target=${yaw}dps
  elif [[ "$profile" == "vertical" ]]; then
    target=${vertical}mps
  elif [[ "$profile" == "combined" ]]; then
    target=${horizontal}mps_${yaw}dps
  fi
  case_dir="$MATRIX_DIR/${case_id}_${profile}_${target}"
  run_dir="$case_dir/headless"
  output_dir="$case_dir/result"
  mkdir -p "$run_dir" "$output_dir"
  printf '\n=== %s: %s horizontal=%s vertical=%s yaw=%s ===\n' \
    "$case_id" "$profile" "$horizontal" "$vertical" "$yaw"

  setsid env \
    RUN_ID="speed_${MATRIX_ID}_${case_id}" RUN_DIR="$run_dir" \
    ACTIVE_FILE="$ACTIVE_FILE" RTABMAP_PROFILE=feature_aligned \
    D435I_WORLD=textured D435I_BRIDGE_IMPL=cpp D435I_DEPTH_ENCODING=16UC1 \
    D435I_QOS_RELIABILITY=reliable D435I_QOS_DEPTH=1 D435I_RTAB_QOS=1 \
    D435I_ENABLE_RTABMAP=1 D435I_START_FLIGHT_STACK=1 \
    GAZEBO_GUI=0 HEADLESS_RENDERING=1 \
    bash "$PKG_SHARE/scripts/run_d435i_visual_slam_headless.sh" \
    >"$case_dir/wrapper.log" 2>&1 &
  wrapper_pid=$!
  ready=0
  for _ in {1..180}; do
    if grep -q 'D435i-only visual SLAM baseline is ready' "$case_dir/wrapper.log" 2>/dev/null; then
      ready=1
      break
    fi
    if ! kill -0 "$wrapper_pid" 2>/dev/null; then break; fi
    sleep 1
  done
  if [[ "$ready" != "1" ]]; then
    printf '%s stack did not become ready.\n' "$case_id" >&2
    tail -n 160 "$case_dir/wrapper.log" >&2 || true
    printf '%s\t%s\t%s\t%s\t%s\t%s\t125\tFAIL\t%s\n' \
      "$case_id" "$family" "$profile" "$horizontal" "$vertical" "$yaw" "$output_dir" \
      >>"$MATRIX_DIR/matrix.tsv"
    stopped_family[$family]=1
    cleanup_stack
    wrapper_pid=""
    continue
  fi

  set +e
  env OUTPUT_DIR="$output_dir" ACTIVE_FILE="$ACTIVE_FILE" \
    RTABMAP_PROFILE=feature_aligned SPEED_TEST_PROFILE="$profile" \
    HORIZONTAL_SPEED_MPS="$horizontal" VERTICAL_SPEED_MPS="$vertical" \
    YAW_RATE_DEG_S="$yaw" \
    bash "$PKG_SHARE/scripts/run_d435i_speed_envelope_profile.sh" \
    >"$case_dir/profile_console.log" 2>&1
  status=$?
  set -e
  cleanup_stack
  wrapper_pid=""

  cp "$run_dir/rtabmap.log" "$output_dir/rtabmap.log" 2>/dev/null || true
  if [[ -f "$run_dir/rtabmap.db" ]]; then
    cp "$run_dir/rtabmap.db" "$output_dir/database.db"
    set +e
    ros2 run multi_slam_uav_sim rtabmap_database_diagnostics \
      "$output_dir/database.db" "$output_dir" \
      --loop-csv "$output_dir/loop_closure.csv" \
      >"$output_dir/database_diagnostics.log" 2>&1
    diagnostic_status=$?
    set -e
    if [[ "$diagnostic_status" != "0" ]]; then status=$diagnostic_status; fi
  else
    printf 'Missing per-run database: %s\n' "$run_dir/rtabmap.db" >&2
    status=1
  fi

  classification=FAIL
  if [[ -f "$output_dir/summary.json" ]]; then
    classification=$(python3 -c \
      'import json,sys; print(json.load(open(sys.argv[1]))["classification"])' \
      "$output_dir/summary.json")
  fi
  if [[ "$status" != "0" ]]; then classification=FAIL; fi
  printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
    "$case_id" "$family" "$profile" "$horizontal" "$vertical" "$yaw" \
    "$status" "$classification" "$output_dir" >>"$MATRIX_DIR/matrix.tsv"
  if [[ "$classification" == "FAIL" ]]; then
    stopped_family[$family]=1
  fi
  sleep 2
done

trap - EXIT INT TERM
ros2 run multi_slam_uav_sim d435i_speed_envelope_comparison "$MATRIX_DIR"
cp "$MATRIX_DIR/speed_envelope.csv" "$SPEED_ROOT/speed_envelope.csv"
cp "$MATRIX_DIR/speed_envelope_comparison.md" "$SPEED_ROOT/speed_envelope_comparison.md"
printf 'Speed-envelope matrix complete: %s\n' "$MATRIX_DIR"
cat "$MATRIX_DIR/matrix.tsv"
