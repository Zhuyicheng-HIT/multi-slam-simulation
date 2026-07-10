#!/usr/bin/env bash
set -eo pipefail

NAME=${1:?Usage: run_named_world.sh NAME [extra gz sim args...]}
shift || true

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
source "$WS_INSTALL/setup.bash"
source "$SCRIPT_DIR/env.sh"

case "$NAME" in
  ardupilot_warehouse|iris_warehouse)
    WORLD="$MULTI_SLAM_SHARE/worlds/ardupilot_iris_warehouse.sdf"
    ;;
  simple_test|simple_uav_test)
    WORLD="$MULTI_SLAM_SHARE/worlds/simple_uav_test.sdf"
    ;;
  tunnel|lidar_tunnel)
    WORLD="$MULTI_SLAM_SHARE/worlds/gz_builtin_tunnel.sdf"
    ;;
  clearpath_warehouse|warehouse)
    WORLD="$MULTI_SLAM_EXTERNAL_DIR/clearpath_simulator/clearpath_gz/worlds/warehouse.sdf"
    ;;
  clearpath_office|office)
    WORLD="$MULTI_SLAM_EXTERNAL_DIR/clearpath_simulator/clearpath_gz/worlds/office.sdf"
    ;;
  clearpath_construction|construction)
    WORLD="$MULTI_SLAM_EXTERNAL_DIR/clearpath_simulator/clearpath_gz/worlds/construction.sdf"
    ;;
  city_applepark|applepark)
    WORLD="$MULTI_SLAM_EXTERNAL_DIR/gazebo_terrain_generator/sample_worlds/applepark/applepark.world"
    ;;
  city_joshimath|joshimath)
    WORLD="$MULTI_SLAM_EXTERNAL_DIR/gazebo_terrain_generator/sample_worlds/Joshimath/Joshimath.world"
    ;;
  *)
    echo "Unknown world alias: $NAME" >&2
    echo "Known aliases: simple_test, ardupilot_warehouse, tunnel, clearpath_warehouse, office, construction, city_applepark, city_joshimath" >&2
    exit 2
    ;;
esac

exec gz sim -r "$WORLD" "$@"
