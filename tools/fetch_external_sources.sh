#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
DEPS_ROOT=${DEPS_ROOT:-$HOME/multi-slam-deps}
ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/ardupilot}
ARDUPILOT_GAZEBO_DIR=${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}
LIDAR_WS=${LIDAR_WS:-$DEPS_ROOT/mid360_ws}
LIVOX_SDK2_DIR=${LIVOX_SDK2_DIR:-$DEPS_ROOT/Livox-SDK2}

clone_at_commit() {
  local url=$1
  local directory=$2
  local commit=$3
  shift 3
  if [[ ! -d "$directory/.git" ]]; then
    git clone "$@" "$url" "$directory"
  fi
  git -C "$directory" fetch origin "$commit"
  git -C "$directory" checkout "$commit"
}

mkdir -p "$DEPS_ROOT" "$LIDAR_WS/src"

clone_at_commit \
  https://github.com/ArduPilot/ardupilot.git \
  "$ARDUPILOT_DIR" \
  f9d619e26002d6aaa41643ee99c0ae0ee01e2247 \
  --recurse-submodules
git -C "$ARDUPILOT_DIR" submodule update --init --recursive

clone_at_commit \
  https://github.com/ArduPilot/ardupilot_gazebo.git \
  "$ARDUPILOT_GAZEBO_DIR" \
  082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5

clone_at_commit \
  https://github.com/Livox-SDK/Livox-SDK2.git \
  "$LIVOX_SDK2_DIR" \
  f5d9375f84efe2b15bc0a052d3e18482ed13adf4

vcs import --recursive --skip-existing "$LIDAR_WS/src" \
  < "$REPO_ROOT/dependencies.repos"

cat <<EOF
External source download complete.

ArduPilot:         $ARDUPILOT_DIR
ArduPilot Gazebo:  $ARDUPILOT_GAZEBO_DIR
MID360 workspace:  $LIDAR_WS
Livox SDK2:        $LIVOX_SDK2_DIR

No external project was copied into this repository.
Continue with docs/INSTALL.md to build each dependency.
EOF

