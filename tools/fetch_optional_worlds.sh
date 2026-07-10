#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
EXTERNAL_DIR="$REPO_ROOT/external"

clone_at_commit() {
  local url=$1
  local directory=$2
  local commit=$3
  if [[ ! -d "$directory/.git" ]]; then
    git clone "$url" "$directory"
  fi
  git -C "$directory" fetch origin "$commit"
  git -C "$directory" checkout "$commit"
}

mkdir -p "$EXTERNAL_DIR"

clone_at_commit \
  https://github.com/clearpathrobotics/clearpath_simulator.git \
  "$EXTERNAL_DIR/clearpath_simulator" \
  25997cb564d65867d85de155233b95567e8724a3

clone_at_commit \
  https://github.com/saiaravind19/gazebo_terrain_generator.git \
  "$EXTERNAL_DIR/gazebo_terrain_generator" \
  4946f4c8150633e4c1fb2ffe9a2ab4f495de9577

cat <<EOF
可选大型场景已下载到：
  $EXTERNAL_DIR

这些目录已被 Git 忽略，不会上传到项目仓库。
EOF
