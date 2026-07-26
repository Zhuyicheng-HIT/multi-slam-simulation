#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
FAST_LIO_SRC=${FAST_LIO_SRC:-$LIDAR_WS/src/FAST_LIO_ROS2}
EXPECTED_COMMIT=a4743b095409588842a5b30ddfa27e29d2f99164
PATCH_FILE="$REPO_ROOT/src/ultra_fusion_nav/uf_lio_adapter/fast_lio_patches/0001-native-lidar-factor-export.patch"

if [[ ! -d "$FAST_LIO_SRC/.git" ]]; then
  echo "FAST-LIO source checkout not found: $FAST_LIO_SRC" >&2
  exit 2
fi
if [[ ! -f "$PATCH_FILE" ]]; then
  echo "Patch file not found: $PATCH_FILE" >&2
  exit 2
fi

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

git -C "$FAST_LIO_SRC" apply --check "$PATCH_FILE"
git -C "$FAST_LIO_SRC" apply "$PATCH_FILE"
echo "Applied native LiDAR factor export patch to $FAST_LIO_SRC"
