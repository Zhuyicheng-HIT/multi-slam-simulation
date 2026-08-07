#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

# Stable four-source frontend mode. FAST-LIO retains its proven internal
# prediction only for deskew/matching, exports native point-to-plane factors,
# and accepts map insertion from the unified backend. It does not own the
# route feedback or final published navigation state.
export FASTLIO_INPUT_MODE=${FASTLIO_INPUT_MODE:-livox}
export FASTLIO_NATIVE_FACTOR_EXPORT=${FASTLIO_NATIVE_FACTOR_EXPORT:-1}
export FASTLIO_DOWNSTREAM_BACKEND=${FASTLIO_DOWNSTREAM_BACKEND:-1}
export FASTLIO_DIAGNOSTIC_ODOMETRY=${FASTLIO_DIAGNOSTIC_ODOMETRY:-1}
export FASTLIO_DIAGNOSTIC_PATH=${FASTLIO_DIAGNOSTIC_PATH:-0}
export FASTLIO_DIAGNOSTIC_TF=${FASTLIO_DIAGNOSTIC_TF:-0}
export FASTLIO_MAP_INSERTION_MODE=${FASTLIO_MAP_INSERTION_MODE:-backend_confirmed}
export FASTLIO_BACKEND_STATE_TOPIC=${FASTLIO_BACKEND_STATE_TOPIC:-/fusion/unified/map_pose}
export FASTLIO_BACKEND_ACTIVATION_STATE_TOPIC=${FASTLIO_BACKEND_ACTIVATION_STATE_TOPIC:-/fusion/unified/odom}
export FASTLIO_BACKEND_TRAJECTORY_FRONTEND=${FASTLIO_BACKEND_TRAJECTORY_FRONTEND:-0}

exec bash "$SCRIPT_DIR/run_fastlio_mapping.sh" "$@"
