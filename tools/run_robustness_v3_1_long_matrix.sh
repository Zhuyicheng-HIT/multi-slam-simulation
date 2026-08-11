#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
MATRIX_DIR=${MATRIX_DIR:-$REPO_ROOT/logs/tmp/robustness_v3_1_long_matrix}
MODES=${MODES:-"disabled lidar_only joint"}
REPEATS=${REPEATS:-"1 2 3"}
mkdir -p "$MATRIX_DIR"

index=0
for mode in $MODES; do
  for repeat in $REPEATS; do
    index=$((index + 1))
    run_dir="$MATRIX_DIR/${mode}_r${repeat}"
    if [[ -s "$run_dir/robustness_joint_map_report.json" && ${RESUME_EXISTING:-0} == 1 ]]; then
      continue
    fi
    printf 'long-run mode=%s repeat=%s\n' "$mode" "$repeat"
    set +e
    # Keep DDS discovery ports outside Linux's default ephemeral range.
    # High domain IDs (for example 221) can make Fast DDS discovery stall in
    # WSL even while Gazebo's transport clock is healthy.
    RUN_DIR="$run_dir" MAP_MODE="$mode" \
      ROS_DOMAIN_ID=$((30 + index)) \
      RECTANGLE_LENGTH_X=10.0 RECTANGLE_LENGTH_Y=6.0 \
      RECTANGLE_SPEED_MPS=0.5 PERFORMANCE_PROFILING_ENABLED=1 \
      EVIDENCE_ROS_DURATION_S=300 EVIDENCE_WALL_TIMEOUT_S=1800 \
      TRAJECTORY_ROS_DURATION_S=300 TRAJECTORY_WALL_TIMEOUT_S=1800 \
      "$SCRIPT_DIR/run_robustness_v3_joint_map_stress.sh" \
      >"$MATRIX_DIR/${mode}_r${repeat}_wrapper.log" 2>&1
    status=$?
    set -e
    printf 'status=%s\nmode=%s\nrepeat=%s\n' \
      "$status" "$mode" "$repeat" >"$run_dir/v3_1_run_status.env"
    if [[ -s "$run_dir/backend_cycle_trace.jsonl" ]]; then
      analyze_args=(
        --trace "$run_dir/backend_cycle_trace.jsonl"
        --output "$run_dir/backend_cycle_analysis.json"
      )
      if [[ -s "$run_dir/simulation_performance.json" ]]; then
        analyze_args+=(--performance "$run_dir/simulation_performance.json")
      fi
      python3 "$SCRIPT_DIR/analyze_backend_cycle_trace.py" "${analyze_args[@]}" \
        >"$run_dir/backend_cycle_analysis.log" 2>&1 || true
    fi
    sleep 5
  done
done
