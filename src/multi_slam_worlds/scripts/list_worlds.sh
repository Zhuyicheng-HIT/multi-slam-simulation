#!/usr/bin/env bash
set -eo pipefail
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
source "$WS_INSTALL/setup.bash"
source "$SCRIPT_DIR/env.sh"
printf '== packaged worlds ==\n'
find "$MULTI_SLAM_SHARE/worlds" -maxdepth 4 -type f \( -name '*.sdf' -o -name '*.world' \) | sort
printf '\n== source worlds ==\n'
find "$MULTI_SLAM_EXTERNAL_DIR" "$ARDUPILOT_GAZEBO_DIR/worlds" /usr/share/gz/gz-sim8/worlds -maxdepth 6 -type f \( -name '*.sdf' -o -name '*.world' \) 2>/dev/null | sort
