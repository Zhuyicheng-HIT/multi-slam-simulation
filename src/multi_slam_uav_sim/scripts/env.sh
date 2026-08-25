#!/usr/bin/env bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)

export MULTI_SLAM_WS="$WS_ROOT"
export MULTI_SLAM_UAV_SIM_SHARE="$PKG_SHARE"
# Fast DDS is unreliable in the restored WSL image.  Keep all ROS 2 sensor
# and bridge processes on the validated Cyclone DDS transport unless the
# caller explicitly selects another RMW implementation.
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export GZ_SIM_RESOURCE_PATH="$PKG_SHARE/worlds:$PKG_SHARE/models:$WS_ROOT/src/multi_slam_worlds/worlds:$WS_ROOT/src/multi_slam_worlds/models:${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}/worlds:${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}/models:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_RENDER_ENGINE="${GZ_RENDER_ENGINE:-ogre2}"

# WSLg exposes all Windows GPUs through Mesa D3D12. Prefer the discrete NVIDIA
# adapter when it is present, while leaving native Linux and non-NVIDIA hosts
# untouched. Set GAZEBO_GPU_ADAPTER explicitly (including to an empty string)
# to override this auto-selection.
if [[ -z "${GAZEBO_GPU_ADAPTER+x}" ]]; then
  GAZEBO_GPU_ADAPTER=""
  if [[ -e /dev/dxg ]] && command -v nvidia-smi >/dev/null 2>&1; then
    if nvidia-smi -L 2>/dev/null | grep -qi NVIDIA; then
      GAZEBO_GPU_ADAPTER="NVIDIA"
    fi
  fi
fi
export GAZEBO_GPU_ADAPTER
if [[ -e /dev/dxg && -n "$GAZEBO_GPU_ADAPTER" ]]; then
  export MESA_D3D12_DEFAULT_ADAPTER_NAME="$GAZEBO_GPU_ADAPTER"
fi
if [[ -z "${REQUIRE_GAZEBO_GPU+x}" ]]; then
  REQUIRE_GAZEBO_GPU=0
  if [[ -e /dev/dxg && -n "$GAZEBO_GPU_ADAPTER" ]]; then
    REQUIRE_GAZEBO_GPU=1
  fi
fi
export REQUIRE_GAZEBO_GPU

export PROTOCOL_BUFFERS_PYTHON_IMPLEMENTATION=python
