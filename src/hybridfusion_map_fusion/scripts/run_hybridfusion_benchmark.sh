#!/usr/bin/env bash
set -Eeo pipefail

source /opt/ros/humble/setup.bash
if [[ -n "${HYBRIDFUSION_WORKSPACE_SETUP:-}" ]]; then
  source "$HYBRIDFUSION_WORKSPACE_SETUP"
fi

PACKAGE_PREFIX=$(ros2 pkg prefix hybridfusion_map_fusion)
PACKAGE_SHARE="$PACKAGE_PREFIX/share/hybridfusion_map_fusion"
OUTPUT_ROOT=${1:-$PWD/logs/hybridfusion/benchmark_$(date +%Y%m%d_%H%M%S)}
CONFIG=${2:-$PACKAGE_SHARE/config/hybridfusion.yaml}
RUNS=${HYBRIDFUSION_RUNS:-3}
DATASET_DIR="$OUTPUT_ROOT/dataset"
mkdir -p "$OUTPUT_ROOT"

ros2 run hybridfusion_map_fusion generate_hybridfusion_dataset.py \
  --output "$DATASET_DIR" --preset benchmark --seed 20260805 \
  >"$OUTPUT_ROOT/dataset_generation.log" 2>&1

printf 'run\tmethod\texit_code\tresult\n' >"$OUTPUT_ROOT/run_matrix.tsv"
overall_status=0
for run in $(seq 1 "$RUNS"); do
  run_name=$(printf 'run_%02d' "$run")
  for method in initial gicp hybrid; do
    result_dir="$OUTPUT_ROOT/$run_name/$method"
    mkdir -p "$result_dir"
    set +e
    /usr/bin/time -v ros2 run hybridfusion_map_fusion hybridfusion_offline \
      --dataset "$DATASET_DIR/dataset.yaml" \
      --config "$CONFIG" --method "$method" --output "$result_dir" \
      >"$result_dir/console.log" 2>"$result_dir/time.log"
    status=$?
    set -e
    if (( status != 0 )); then
      overall_status=1
    fi
    printf '%s\t%s\t%s\t%s\n' \
      "$run_name" "$method" "$status" "$result_dir/result.json" \
      >>"$OUTPUT_ROOT/run_matrix.tsv"
  done
done

ros2 run hybridfusion_map_fusion evaluate_hybridfusion_runs.py "$OUTPUT_ROOT"
printf 'HybridFusion benchmark complete: %s\n' "$OUTPUT_ROOT"
cat "$OUTPUT_ROOT/run_matrix.tsv"
exit "$overall_status"
