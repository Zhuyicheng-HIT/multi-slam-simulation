#!/usr/bin/env bash
set -Eeuo pipefail

if [[ $# -lt 1 ]]; then
  printf 'Usage: %s BAG_DIRECTORY [RATE]\n' "$0" >&2
  exit 2
fi

BAG=$1
RATE=${2:-1.0}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/../../.." && pwd)
set +u
source /opt/ros/humble/setup.bash
source "$REPO_ROOT/install/setup.bash"
set -u
exec ros2 bag play "$BAG" --clock --rate "$RATE" --read-ahead-queue-size 1000
