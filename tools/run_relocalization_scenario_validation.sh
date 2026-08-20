#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
scenario=${RELOCALIZATION_SCENARIO:-nominal}
motion_profile=${RELOCALIZATION_LOGIC:-hold}
run_id=${RELOCALIZATION_RUN_ID:-$(date +%Y%m%d_%H%M%S)}

case "$motion_profile" in
  hold|yaw_scan|circle|figure8) ;;
  *)
    printf 'Unsupported relocalization logic: %s\n' "$motion_profile" >&2
    exit 2
    ;;
esac

case "$scenario" in
  nominal)
    world=low_indoor_apm_rgbd_mid360
    route_speed=0.35
    checkpoint=${RELOCALIZATION_CHECKPOINT:-8}
    ;;
  fast)
    world=low_indoor_apm_rgbd_mid360
    route_speed=0.70
    checkpoint=${RELOCALIZATION_CHECKPOINT:-8}
    ;;
  structural_window)
    world=low_indoor_apm_rgbd_mid360_window
    route_speed=0.35
    checkpoint=${RELOCALIZATION_CHECKPOINT:-8}
    ;;
  *)
    printf 'Unsupported relocalization scenario: %s\n' "$scenario" >&2
    exit 2
    ;;
esac

export VALIDATION_WORLD_NAME=$world
export S_CURVE_SPEED=$route_speed
export VALIDATION_RELOCALIZATION_CHECKPOINTS=$checkpoint
export VALIDATION_RELOCALIZATION_MOTION_PROFILE=$motion_profile
export VALIDATION_RELOCALIZATION_MOTION_SETTLE_S=${VALIDATION_RELOCALIZATION_MOTION_SETTLE_S:-1.5}
# Active excitation invalidates the stationary observation used by
# stationary_zero; keep the static policy for hold and use rotated velocity
# for motion-assisted relocalization unless the caller overrides it.
if [[ -z "${VALIDATION_RELOCALIZATION_VELOCITY_POLICY:-}" ]]; then
  if [[ "$motion_profile" == "hold" ]]; then
    export VALIDATION_RELOCALIZATION_VELOCITY_POLICY=stationary_zero
  else
    export VALIDATION_RELOCALIZATION_VELOCITY_POLICY=rotate
  fi
else
  export VALIDATION_RELOCALIZATION_VELOCITY_POLICY
fi
export VALIDATION_RELOCALIZATION_BIAS_POLICY=${VALIDATION_RELOCALIZATION_BIAS_POLICY:-preserve}
export VALIDATION_STOP_AFTER_LANDING=${VALIDATION_STOP_AFTER_LANDING:-true}
export VALIDATION_LANDING_GRACE_S=${VALIDATION_LANDING_GRACE_S:-5}
export LOG_DIR=${LOG_DIR:-$REPO_ROOT/logs/${scenario}_${motion_profile}_${run_id}}

printf 'Relocalization scenario: %s\n' "$scenario"
printf 'Relocalization logic: %s\n' "$motion_profile"
printf 'Scenario contract: world=%s speed=%s checkpoint=%s\n' \
  "$world" "$route_speed" "$checkpoint"
printf 'Evidence directory: %s\n' "$LOG_DIR"

exec bash "$REPO_ROOT/tools/run_frozen_low_figure8_validation.sh" "$@"
