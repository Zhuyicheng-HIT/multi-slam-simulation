#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)

export ENABLE_GAZEBO_FLOW=${ENABLE_GAZEBO_FLOW:-1}
export ENABLE_FCU_FLOW=0
export ENABLE_FCU_RANGE=0
export ENABLE_NONGPS_FLOW=0
export ENABLE_EXTERNALNAV_EKF3=1
export ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV=0
export ENABLE_D435_BRIDGE=${ENABLE_D435_BRIDGE:-0}
export ENABLE_D435_POINTCLOUD=false
export ENABLE_MID360_BRIDGE=${ENABLE_MID360_BRIDGE:-1}
export SHOW_FLOW_WINDOW=${SHOW_FLOW_WINDOW:-0}
export FLOW_DEBUG=${FLOW_DEBUG:-false}
export FLOW_RESTAMP_OUTPUT=true
export FLOW_USE_PHYSICS=false
export FLOW_PUBLISH_ALL_FRAMES=${FLOW_PUBLISH_ALL_FRAMES:-false}
export FLOW_BRIDGE_HZ=${FLOW_BRIDGE_HZ:-15.0}
export WIPE_EEPROM=${WIPE_EEPROM:-1}
export RECTANGLE_FLOW_TEST=0
export AUTO_FLIGHT=0

# On WSLg, explicitly select NVIDIA when it is available so Gazebo does not
# silently fall back to the integrated adapter.
if command -v nvidia-smi >/dev/null 2>&1 && nvidia-smi -L 2>/dev/null | grep -qi NVIDIA; then
  export GAZEBO_GPU_ADAPTER=${GAZEBO_GPU_ADAPTER:-NVIDIA}
  export MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}
  export REQUIRE_GAZEBO_GPU=${REQUIRE_GAZEBO_GPU:-1}
fi

exec bash "$SCRIPT_DIR/run_apm_sensor_stack.sh"
