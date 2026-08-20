#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
motion_profile=${ACTIVE_RELOCALIZATION_MOTION_PROFILE:-hold}

case "$motion_profile" in
  hold|yaw_scan|circle|figure8) ;;
  *)
    printf 'Unknown ACTIVE_RELOCALIZATION_MOTION_PROFILE: %s\n' \
      "$motion_profile" >&2
    exit 2
    ;;
esac

# Keep the best verified passive reinitialization policy fixed while screening
# observation motions. Matching thresholds and the frozen five-source stack are
# intentionally unchanged.
export VALIDATION_RELOCALIZATION_CHECKPOINTS=${VALIDATION_RELOCALIZATION_CHECKPOINTS:-8}
export VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S=${VALIDATION_RELOCALIZATION_SEARCH_TIMEOUT_S:-15.0}
export VALIDATION_RELOCALIZATION_VELOCITY_POLICY=stationary_zero
export VALIDATION_RELOCALIZATION_BIAS_POLICY=preserve
export VALIDATION_RELOCALIZATION_STATIONARY_MAXIMUM_SPEED_MPS=${VALIDATION_RELOCALIZATION_STATIONARY_MAXIMUM_SPEED_MPS:-0.35}
export VALIDATION_RELOCALIZATION_MOTION_PROFILE=$motion_profile
export VALIDATION_RELOCALIZATION_MOTION_RADIUS_M=${VALIDATION_RELOCALIZATION_MOTION_RADIUS_M:-0.6}
export VALIDATION_RELOCALIZATION_MOTION_SPEED_MPS=${VALIDATION_RELOCALIZATION_MOTION_SPEED_MPS:-0.25}
export VALIDATION_RELOCALIZATION_MOTION_YAW_RATE_DEG_S=${VALIDATION_RELOCALIZATION_MOTION_YAW_RATE_DEG_S:-12.0}
export VALIDATION_RELOCALIZATION_MOTION_YAW_STEP_DEG=${VALIDATION_RELOCALIZATION_MOTION_YAW_STEP_DEG:-45.0}
export VALIDATION_RELOCALIZATION_MOTION_SETTLE_S=${VALIDATION_RELOCALIZATION_MOTION_SETTLE_S:-2.5}
export LOG_DIR=${LOG_DIR:-$REPO_ROOT/logs/active_relocalization_${motion_profile}}

printf 'Active relocalization motion profile: %s\n' "$motion_profile"
exec bash "$REPO_ROOT/tools/run_frozen_low_figure8_validation.sh" "$@"
