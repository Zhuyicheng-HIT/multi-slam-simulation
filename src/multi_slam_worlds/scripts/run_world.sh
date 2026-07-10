#!/usr/bin/env bash
set -eo pipefail
WORLD=${1:?Usage: run_world.sh WORLD_FILE_OR_NAME [extra gz sim args...]}
shift || true
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
source "$WS_INSTALL/setup.bash"
source "$SCRIPT_DIR/env.sh"
if [ -f "$WORLD" ]; then
  exec gz sim -r "$WORLD" "$@"
elif [ -f "$MULTI_SLAM_SHARE/worlds/$WORLD" ]; then
  exec gz sim -r "$MULTI_SLAM_SHARE/worlds/$WORLD" "$@"
else
  MATCH=$(find "$MULTI_SLAM_SHARE/worlds" "$MULTI_SLAM_EXTERNAL_DIR" -type f \( -name "$WORLD" -o -name "$WORLD.sdf" -o -name "$WORLD.world" \) 2>/dev/null | head -1)
  if [ -z "$MATCH" ]; then
    echo "World not found: $WORLD" >&2
    exit 2
  fi
  exec gz sim -r "$MATCH" "$@"
fi
