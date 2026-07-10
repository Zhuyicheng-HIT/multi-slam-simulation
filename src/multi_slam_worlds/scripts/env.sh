#!/usr/bin/env bash
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
MULTI_SLAM_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$MULTI_SLAM_SHARE/../../.." && pwd)
MULTI_SLAM_WS=$(cd "$WS_INSTALL/.." && pwd)
MULTI_SLAM_EXTERNAL_DIR=${MULTI_SLAM_EXTERNAL_DIR:-$MULTI_SLAM_WS/external}
ARDUPILOT_GAZEBO_DIR=${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}

export MULTI_SLAM_SHARE MULTI_SLAM_WS MULTI_SLAM_EXTERNAL_DIR ARDUPILOT_GAZEBO_DIR
export GZ_SIM_RESOURCE_PATH="$MULTI_SLAM_SHARE/worlds:$MULTI_SLAM_SHARE/models:$MULTI_SLAM_SHARE/models/tunnel_texture_fixes:$ARDUPILOT_GAZEBO_DIR/worlds:$ARDUPILOT_GAZEBO_DIR/models:$MULTI_SLAM_EXTERNAL_DIR/clearpath_simulator:$MULTI_SLAM_EXTERNAL_DIR/clearpath_simulator/clearpath_gz/worlds:$MULTI_SLAM_EXTERNAL_DIR/clearpath_simulator/clearpath_gz/models:$MULTI_SLAM_EXTERNAL_DIR/clearpath_simulator/clearpath_gz/meshes:$MULTI_SLAM_EXTERNAL_DIR/gazebo_terrain_generator:$MULTI_SLAM_EXTERNAL_DIR/gazebo_terrain_generator/sample_worlds:${GZ_SIM_RESOURCE_PATH:-}"
export GZ_SIM_SYSTEM_PLUGIN_PATH="$ARDUPILOT_GAZEBO_DIR/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}"
export GZ_RENDER_ENGINE=${GZ_RENDER_ENGINE:-ogre2}
