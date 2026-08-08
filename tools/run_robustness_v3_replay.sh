#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
WORKSPACE_ROOT=${WORKSPACE_ROOT:-$REPO_ROOT}
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
BAG_DIR=${BAG_DIR:-$REPO_ROOT/logs/tmp/performance_v2_full_replay_capture_b_20260808/full_online_backend_replay}
TRUTH_PATH=${TRUTH_PATH:-$REPO_ROOT/logs/tmp/performance_v2_full_replay_capture_b_20260808/online/trajectory/ground_truth.tum}
REFERENCE_ESTIMATE_PATH=${REFERENCE_ESTIMATE_PATH:-$REPO_ROOT/logs/tmp/performance_v2_full_replay_capture_b_20260808/online/trajectory/estimate.tum}
PROFILE_PATH=${PROFILE_PATH:-$REPO_ROOT/src/ultra_fusion_nav/uf_sensor_pipeline/config/robustness_v3_profiles.yaml}
PROFILE=${PROFILE:-nominal}
FRS_MODE=${FRS_MODE:-on}
REPLAY_RATE=${REPLAY_RATE:-1.0}
POST_PLAY_SETTLE_S=${POST_PLAY_SETTLE_S:-0.5}
ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-83}
OUTPUT_DIR=${OUTPUT_DIR:-$REPO_ROOT/logs/tmp/robustness_v3_${PROFILE}_${FRS_MODE}_$(date +%Y%m%d_%H%M%S)}
export ROS_DOMAIN_ID

if [[ "$FRS_MODE" != on && "$FRS_MODE" != off ]]; then
  printf 'FRS_MODE must be on or off, got %s\n' "$FRS_MODE" >&2
  exit 2
fi
for required in "$BAG_DIR/metadata.yaml" "$TRUTH_PATH" "$REFERENCE_ESTIMATE_PATH" "$PROFILE_PATH"; do
  [[ -e "$required" ]] || { printf 'Missing required input: %s\n' "$required" >&2; exit 2; }
done

source /opt/ros/humble/setup.bash
source "$WORKSPACE_ROOT/install/setup.bash"
source "$LIDAR_WS/install/setup.bash"
mkdir -p "$OUTPUT_DIR"

pids=()
cleanup() {
  trap - EXIT INT TERM
  for pid in "${pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${pids[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

mapfile -t profile_overrides < <(python3 - "$PROFILE_PATH" "$PROFILE" <<'PY'
import json
import sys
from uf_sensor_pipeline.fault_profiles import load_fault_profile, profile_backend_overrides

profile = load_fault_profile(sys.argv[1], sys.argv[2])
for key, value in profile_backend_overrides(profile).items():
    print(f"{key}:={json.dumps(value, separators=(',', ':'))}")
PY
)

backend_command=(
  ros2 run uf_backend_fusion online_backend_fusion
  --ros-args
  --params-file "$WORKSPACE_ROOT/install/uf_backend_fusion/share/uf_backend_fusion/config/online_backend.yaml"
  -p use_sim_time:=true
  -p visual_factor_mode:=paper_reprojection
  -p visual_pending_enabled:=true
  -p native_lidar_factor_enabled:=true
  -p input_trigger_mode:=native_factor
  -p frontend_scan_prediction_enabled:=true
  -p allow_lio_pose_fallback:=false
  -p imu_factor_enabled:=true
  -p performance_profiling_enabled:=true
  -p performance_trace_path:="$OUTPUT_DIR/backend_cycle_trace.jsonl"
)
if [[ "$FRS_MODE" == on ]]; then
  backend_command+=(-p reliability_mode:=dynamic)
else
  backend_command+=(
    -p reliability_mode:=fixed
    -p fixed_lidar_weight:=1.0
    -p fixed_imu_weight:=1.0
    -p fixed_gnss_weight:=1.0
    -p fixed_optical_flow_weight:=1.0
    -p fixed_vision_weight:=1.0
    -p fixed_covariance_inflation:=1.0
  )
fi
for override in "${profile_overrides[@]:-}"; do
  [[ -n "$override" ]] && backend_command+=(-p "$override")
done

setsid ros2 run uf_sensor_pipeline robustness_fault_injector --ros-args \
  -p use_sim_time:=true -p profile_path:="$PROFILE_PATH" -p profile:="$PROFILE" \
  >"$OUTPUT_DIR/fault_injector.log" 2>&1 &
pids+=("$!")

setsid ros2 run uf_reliability reliability_scheduler --ros-args \
  --params-file "$WORKSPACE_ROOT/install/uf_reliability/share/uf_reliability/config/scheduler_config.yaml" \
  -p use_sim_time:=true \
  -p active_modalities:="[lidar,gnss,imu,optical_flow,vision]" \
  -p required_modalities:="[imu]" -p minimum_usable_modalities:=2 \
  -p automatic_relocalization_enabled:=false \
  >"$OUTPUT_DIR/scheduler.log" 2>&1 &
pids+=("$!")

setsid /usr/bin/time -v -o "$OUTPUT_DIR/backend_resource.txt" \
  env OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  NUMEXPR_NUM_THREADS=1 "${backend_command[@]}" \
  >"$OUTPUT_DIR/backend.log" 2>&1 &
pids+=("$!")

setsid python3 "$REPO_ROOT/tools/record_backend_replay_metrics.py" \
  --output "$OUTPUT_DIR/replay_metrics.json" \
  --trajectory-output "$OUTPUT_DIR/estimate.tum" --wall-timeout 1800 \
  >"$OUTPUT_DIR/metrics_recorder.log" 2>&1 &
metrics_pid=$!
pids+=("$metrics_pid")

setsid python3 "$REPO_ROOT/src/ultra_fusion_nav/scripts/record_reliability_timeline.py" \
  --duration 100000 --wall-timeout 1800 \
  --output "$OUTPUT_DIR/reliability_timeline.json" \
  >"$OUTPUT_DIR/timeline_recorder.log" 2>&1 &
timeline_pid=$!
pids+=("$timeline_pid")
sleep 5

play_started=$(date +%s.%N)
set +e
timeout 1800s ros2 bag play "$BAG_DIR" --rate "$REPLAY_RATE" \
  --remap \
  /fast_lio/native_lidar_factor:=/robustness/raw/native_lidar_factor \
  /sensors/imu:=/robustness/raw/imu \
  /sensors/gnss/fix:=/robustness/raw/gnss \
  /sensors/optical_flow/rad:=/robustness/raw/optical_flow \
  /vision/feature_tracks:=/robustness/raw/visual_tracks \
  /reliability/lidar_score:=/robustness/raw/lidar_score \
  /reliability/imu_score:=/robustness/raw/imu_score \
  /reliability/gnss_score:=/robustness/raw/gnss_score \
  /reliability/optical_flow_score:=/robustness/raw/optical_flow_score \
  /reliability/vision_score:=/robustness/raw/vision_score \
  /reliability/scheduler_state:=/robustness/recorded_scheduler_state \
  >"$OUTPUT_DIR/rosbag_play.log" 2>&1
play_status=$?
set -e
play_finished=$(date +%s.%N)
# Stop evidence capture before the frozen sources become stale.  A long
# post-play wait would manufacture a terminal FAILSAFE unrelated to the fault.
sleep "$POST_PLAY_SETTLE_S"
kill -INT "$metrics_pid" "$timeline_pid" 2>/dev/null || true
for _ in {1..50}; do
  [[ -s "$OUTPUT_DIR/replay_metrics.json" && -s "$OUTPUT_DIR/reliability_timeline.json" ]] && break
  sleep 0.2
done
cleanup
trap - EXIT INT TERM

python3 "$REPO_ROOT/src/ultra_fusion_nav/scripts/evaluate_lio_trajectory.py" \
  --estimate "$OUTPUT_DIR/estimate.tum" --truth "$TRUTH_PATH" \
  --max-delta 0.05 --output "$OUTPUT_DIR/trajectory_metrics.json" \
  >"$OUTPUT_DIR/trajectory_evaluation.log" 2>&1 || true

python3 "$REPO_ROOT/tools/analyze_robustness_v3_run.py" \
  --run-dir "$OUTPUT_DIR" --profile-path "$PROFILE_PATH" \
  --profile "$PROFILE" --frs "$FRS_MODE" --truth "$TRUTH_PATH" \
  --reference-estimate "$REFERENCE_ESTIMATE_PATH" \
  --play-started "$play_started" --play-finished "$play_finished" \
  --output "$OUTPUT_DIR/robustness_report.json"

printf 'play_status=%s\nprofile=%s\nfrs=%s\nrate=%s\nworkspace=%s\nbag=%s\ntruth=%s\n' \
  "$play_status" "$PROFILE" "$FRS_MODE" "$REPLAY_RATE" "$WORKSPACE_ROOT" \
  "$BAG_DIR" "$TRUTH_PATH" >"$OUTPUT_DIR/replay_result.env"
cat "$OUTPUT_DIR/robustness_report.json"
exit "$play_status"
