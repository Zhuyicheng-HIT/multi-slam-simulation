#!/usr/bin/env bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)

export MULTI_SLAM_WS="$WS_ROOT"
export MULTI_SLAM_UAV_SIM_SHARE="$PKG_SHARE"
export GZ_SIM_RESOURCE_PATH="$PKG_SHARE/worlds:$PKG_SHARE/models:$WS_ROOT/src/multi_slam_worlds/worlds:$WS_ROOT/src/multi_slam_worlds/models:${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}/worlds:${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_RENDER_ENGINE="${GZ_RENDER_ENGINE:-ogre2}"
export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
