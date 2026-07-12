#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
REQUIRED_SCRIPT=${1:-}

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  cat >&2 <<'EOF'
未找到 ROS 2 Humble。请先按照 README 使用鱼香 ROS 安装 Humble 桌面版。
EOF
  exit 2
fi

if [[ -n "$REQUIRED_SCRIPT" && -f "$REPO_ROOT/$REQUIRED_SCRIPT" ]]; then
  exit 0
fi

printf '未找到已编译的启动脚本，开始自动编译主仓库...\n'
cd "$REPO_ROOT"
source /opt/ros/humble/setup.bash
python3 -m pip install --user -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y --rosdistro humble \
  --skip-keys ament_python
colcon build --symlink-install

if [[ -n "$REQUIRED_SCRIPT" && ! -f "$REPO_ROOT/$REQUIRED_SCRIPT" ]]; then
  printf '编译完成，但仍未找到：%s\n' "$REPO_ROOT/$REQUIRED_SCRIPT" >&2
  exit 2
fi
