#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
profile=${RELOCALIZATION_REINIT_PROFILE:-baseline}

case "$profile" in
  baseline)
    velocity_policy=rotate
    bias_policy=preserve
    ;;
  stationary_velocity)
    velocity_policy=stationary_zero
    bias_policy=preserve
    ;;
  stationary_velocity_bias)
    velocity_policy=stationary_zero
    bias_policy=stationary_imu
    ;;
  *)
    printf 'Unknown RELOCALIZATION_REINIT_PROFILE: %s\n' "$profile" >&2
    exit 2
    ;;
esac

export VALIDATION_RELOCALIZATION_CHECKPOINTS=${VALIDATION_RELOCALIZATION_CHECKPOINTS:-4,8}
export VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S=${VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S:-15.0}
export VALIDATION_RELOCALIZATION_VELOCITY_POLICY=$velocity_policy
export VALIDATION_RELOCALIZATION_BIAS_POLICY=$bias_policy
export VALIDATION_RELOCALIZATION_STATIONARY_MAXIMUM_SPEED_MPS=${VALIDATION_RELOCALIZATION_STATIONARY_MAXIMUM_SPEED_MPS:-0.35}
export LOG_DIR=${LOG_DIR:-$REPO_ROOT/logs/passive_relocalization_${profile}}

printf 'Passive relocalization profile: %s (velocity=%s, bias=%s)\n' \
  "$profile" "$velocity_policy" "$bias_policy"
exec bash "$REPO_ROOT/tools/run_frozen_low_figure8_validation.sh" "$@"
