#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
FAST_LIO_SRC=${FAST_LIO_SRC:-$LIDAR_WS/src/FAST_LIO_ROS2}
EXPECTED_COMMIT=a4743b095409588842a5b30ddfa27e29d2f99164
PATCH_DIR="$REPO_ROOT/src/ultra_fusion_nav/uf_lio_adapter/fast_lio_patches"
PATCH_FILES=(
  "$PATCH_DIR/0001-native-lidar-factor-export.patch"
  "$PATCH_DIR/0002-backend-trajectory-map-activation-decoupling.patch"
  "$PATCH_DIR/0003-relocalization-epoch-gate.patch"
  "$PATCH_DIR/0004-native-lidar-factor-epoch-contract.patch"
  "$PATCH_DIR/0005-reliable-native-factor-qos.patch"
)

if ! git -C "$FAST_LIO_SRC" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "FAST-LIO source checkout not found: $FAST_LIO_SRC" >&2
  exit 2
fi
for patch_file in "${PATCH_FILES[@]}"; do
  if [[ ! -f "$patch_file" ]]; then
    echo "Patch file not found: $patch_file" >&2
    exit 2
  fi
done

actual_commit=$(git -C "$FAST_LIO_SRC" rev-parse HEAD)
if [[ "$actual_commit" != "$EXPECTED_COMMIT" ]]; then
  echo "Unexpected FAST-LIO commit: $actual_commit" >&2
  echo "Expected: $EXPECTED_COMMIT" >&2
  exit 3
fi
if ! git -C "$FAST_LIO_SRC" diff --quiet; then
  echo "FAST-LIO checkout is dirty; inspect or preserve its changes before applying." >&2
  git -C "$FAST_LIO_SRC" status --short >&2
  exit 4
fi

# Later patches build on files introduced or changed by earlier patches, so
# validate and apply the pinned series in order.
for patch_file in "${PATCH_FILES[@]}"; do
  git -C "$FAST_LIO_SRC" apply --unidiff-zero --check "$patch_file"
  git -C "$FAST_LIO_SRC" apply --unidiff-zero "$patch_file"
done
echo "Applied FAST-LIO downstream-backend patch series to $FAST_LIO_SRC"
