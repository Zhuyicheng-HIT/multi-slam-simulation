#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
RUN_SET=${RUN_SET:-smoke}
FRS_MODES=${FRS_MODES:-"on off"}
REPLAY_RATE=${REPLAY_RATE:-1.0}
PROFILE_PATH=${PROFILE_PATH:-$REPO_ROOT/src/ultra_fusion_nav/uf_sensor_pipeline/config/robustness_v3_profiles.yaml}
TRUTH_PATH=${TRUTH_PATH:-$REPO_ROOT/logs/tmp/performance_v2_full_replay_capture_b_20260808/online/trajectory/ground_truth.tum}
REFERENCE_ESTIMATE_PATH=${REFERENCE_ESTIMATE_PATH:-$REPO_ROOT/logs/tmp/performance_v2_full_replay_capture_b_20260808/online/trajectory/estimate.tum}
MATRIX_ROOT=${MATRIX_ROOT:-$REPO_ROOT/logs/tmp/robustness_v3_matrix_${RUN_SET}_$(date +%Y%m%d_%H%M%S)}
BASE_DOMAIN=${BASE_DOMAIN:-90}
RESUME_EXISTING=${RESUME_EXISTING:-0}
mkdir -p "$MATRIX_ROOT"

# The final analyzer imports the in-workspace package. Source the local
# overlay here as well as in each replay child so a completed matrix can be
# summarized reliably from a fresh shell.
if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
  # shellcheck disable=SC1091
  source "$REPO_ROOT/install/setup.bash"
fi

singles=(
  nominal
  visual_light visual_medium visual_heavy
  lidar_light lidar_medium lidar_heavy
  gnss_denial_light gnss_denial_medium gnss_denial_heavy
  gnss_jump_light gnss_jump_medium gnss_jump_heavy
  flow_light flow_medium flow_heavy
  imu_light imu_medium imu_heavy
)
calibration=(
  camera_time_light camera_time_medium camera_time_heavy camera_time_negative_medium
  lidar_time_light lidar_time_medium lidar_time_heavy lidar_time_negative_medium
  d435_extrinsic_rot_light d435_extrinsic_rot_medium d435_extrinsic_rot_heavy
  d435_extrinsic_trans_light d435_extrinsic_trans_medium d435_extrinsic_trans_heavy
  mid360_extrinsic_rot_light mid360_extrinsic_rot_medium mid360_extrinsic_rot_heavy
  mid360_extrinsic_trans_light mid360_extrinsic_trans_medium mid360_extrinsic_trans_heavy
)
doubles=(
  dual_visual_gnss_medium dual_lidar_gnss_medium dual_imu_flow_medium
  dual_visual_lidar_heavy
)
endurance=(long_visual_gnss_cycles)

case "$RUN_SET" in
  smoke) profiles=(nominal visual_medium lidar_medium gnss_jump_medium flow_medium imu_medium) ;;
  singles) profiles=("${singles[@]}") ;;
  calibration) profiles=("${calibration[@]}") ;;
  doubles) profiles=("${doubles[@]}") ;;
  endurance) profiles=("${endurance[@]}") ;;
  all) profiles=("${singles[@]}" "${calibration[@]}" "${doubles[@]}" "${endurance[@]}") ;;
  *) printf 'Unknown RUN_SET=%s\n' "$RUN_SET" >&2; exit 2 ;;
esac

printf 'profile\tfrs\tstatus\toutput\n' >"$MATRIX_ROOT/matrix_manifest.tsv"
run_index=0
failures=0
for profile in "${profiles[@]}"; do
  for frs in $FRS_MODES; do
    run_index=$((run_index + 1))
    output="$MATRIX_ROOT/${run_index}_${profile}_${frs}"
    printf '[%d] profile=%s frs=%s output=%s\n' \
      "$run_index" "$profile" "$frs" "$output"
    if [[ "$RESUME_EXISTING" == 1 && -s "$output/replay_metrics.json" \
          && -s "$output/robustness_report.json" ]]; then
      printf '%s\t%s\t0\t%s\n' "$profile" "$frs" "$output" \
        >>"$MATRIX_ROOT/matrix_manifest.tsv"
      continue
    fi
    set +e
    PROFILE="$profile" FRS_MODE="$frs" OUTPUT_DIR="$output" \
      PROFILE_PATH="$PROFILE_PATH" TRUTH_PATH="$TRUTH_PATH" \
      REFERENCE_ESTIMATE_PATH="$REFERENCE_ESTIMATE_PATH" \
      REPLAY_RATE="$REPLAY_RATE" ROS_DOMAIN_ID=$((BASE_DOMAIN + run_index)) \
      "$SCRIPT_DIR/run_robustness_v3_replay.sh" \
      >"$MATRIX_ROOT/${run_index}_${profile}_${frs}.log" 2>&1
    status=$?
    set -e
    [[ "$status" == 0 ]] || failures=$((failures + 1))
    printf '%s\t%s\t%s\t%s\n' "$profile" "$frs" "$status" "$output" \
      >>"$MATRIX_ROOT/matrix_manifest.tsv"
  done
done

# Re-run the lightweight analyzer after the matrix so every report uses the
# same final schema even if diagnostics were improved during a long campaign.
while IFS=$'\t' read -r profile frs status output; do
  [[ "$profile" == profile || ! -s "$output/replay_metrics.json" ]] && continue
  wall=$(python3 - "$output/robustness_report.json" <<'PY'
import json, sys
try:
    print(float(json.load(open(sys.argv[1]))["play_wall_s"]))
except Exception:
    print(0.0)
PY
)
  python3 "$SCRIPT_DIR/analyze_robustness_v3_run.py" \
    --run-dir "$output" --profile-path "$PROFILE_PATH" \
    --profile "$profile" --frs "$frs" --truth "$TRUTH_PATH" \
    --reference-estimate "$REFERENCE_ESTIMATE_PATH" \
    --play-started 0 --play-finished "$wall" \
    --output "$output/robustness_report.json" >/dev/null
done <"$MATRIX_ROOT/matrix_manifest.tsv"

python3 "$SCRIPT_DIR/summarize_robustness_v3_matrix.py" \
  --matrix-root "$MATRIX_ROOT" --output "$MATRIX_ROOT/matrix_summary.json"
printf 'runs=%d failures=%d summary=%s\n' \
  "$run_index" "$failures" "$MATRIX_ROOT/matrix_summary.json"
exit "$failures"
