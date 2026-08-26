#!/usr/bin/env bash
set -eo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PKG_SHARE=$(cd "$SCRIPT_DIR/.." && pwd)
WS_INSTALL=$(cd "$PKG_SHARE/../../.." && pwd)
WS_ROOT=$(cd "$WS_INSTALL/.." && pwd)

source /opt/ros/humble/setup.bash
source "$WS_INSTALL/setup.bash"
source "$PKG_SHARE/scripts/env.sh"

ARDUPILOT_DIR=${ARDUPILOT_DIR:-$HOME/ardupilot}
ARDUPILOT_GAZEBO_DIR=${ARDUPILOT_GAZEBO_DIR:-$HOME/ardupilot_gazebo}
# The default demo is a low-altitude indoor route. The legacy urban scene
# remains selectable with WORLD/WORLD_NAME for regression and high-wall tests.
WORLD=${WORLD:-$PKG_SHARE/worlds/low_indoor_apm_rgbd_mid360.sdf}
WORLD_NAME=${WORLD_NAME:-low_indoor_apm_rgbd_mid360}
LOG_DIR=${LOG_DIR:-$WS_ROOT/logs/apm_sensor_stack_$(date +%Y%m%d_%H%M%S)}
LOCK_FILE=${LOCK_FILE:-/tmp/multi_slam_apm_sensor_stack.lock}
# Keep FCU source configuration separate from the estimator that publishes
# ExternalNav. The legacy flag retains its original all-in-one behavior.
ENABLE_EXTERNALNAV_EKF3=${ENABLE_EXTERNALNAV_EKF3:-${ENABLE_EXTERNALNAV_FUSION:-0}}
ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV=${ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV:-${ENABLE_EXTERNALNAV_FUSION:-0}}
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
USE_SIM_TIME=${USE_SIM_TIME:-true}
if [[ -z "${MID360_SIM_BRIDGE_MODE+x}" ]]; then
  if [[ "${ENABLE_MID360_BRIDGE:-1}" == "1" ]]; then
    # Keep simulation on the same Livox CustomMsg boundary as hardware.  The
    # Python PointCloud2 bridge remains an explicit legacy/debug option only.
    MID360_SIM_BRIDGE_MODE=direct_livox
  else
    MID360_SIM_BRIDGE_MODE=disabled
  fi
fi
case "$MID360_SIM_BRIDGE_MODE" in
  direct_livox|pointcloud_python|disabled) ;;
  *)
    printf 'Unsupported MID360_SIM_BRIDGE_MODE=%s. Use direct_livox, pointcloud_python, or disabled.\n' \
      "$MID360_SIM_BRIDGE_MODE" >&2
    exit 2
    ;;
esac
MID360_BODY_FILTER_ENABLED=${MID360_BODY_FILTER_ENABLED:-true}
MID360_BODY_MIN_X_M=${MID360_BODY_MIN_X_M:--0.45}
MID360_BODY_MAX_X_M=${MID360_BODY_MAX_X_M:-0.45}
MID360_BODY_MIN_Y_M=${MID360_BODY_MIN_Y_M:--0.45}
MID360_BODY_MAX_Y_M=${MID360_BODY_MAX_Y_M:-0.45}
MID360_BODY_MIN_Z_M=${MID360_BODY_MIN_Z_M:--0.35}
MID360_BODY_MAX_Z_M=${MID360_BODY_MAX_Z_M:-0.15}
MID360_LIDAR_TO_BODY_X_M=${MID360_LIDAR_TO_BODY_X_M:-0.05}
MID360_LIDAR_TO_BODY_Y_M=${MID360_LIDAR_TO_BODY_Y_M:-0.0}
MID360_LIDAR_TO_BODY_Z_M=${MID360_LIDAR_TO_BODY_Z_M:-0.10}

if [[ "$MID360_SIM_BRIDGE_MODE" == "direct_livox" ]]; then
  if [[ ! -f "$LIDAR_WS/install/setup.bash" ]]; then
    printf 'Direct Livox simulation bridge requires %s/install/setup.bash\n' "$LIDAR_WS" >&2
    exit 2
  fi
  # Keep the dependency workspace's environment local. Its full setup can
  # overwrite simulator plugin paths needed by ArduPilot Gazebo.
  source "$LIDAR_WS/install/local_setup.bash"
  source "$WS_INSTALL/setup.bash"
fi

if [[ -f "$LOCK_FILE" ]]; then
  old_pid=$(cat "$LOCK_FILE" 2>/dev/null || true)
  if [[ -n "${old_pid:-}" ]] && kill -0 "$old_pid" 2>/dev/null; then
    cat <<EOF
APM sensor stack is already running.

Existing stack PID:
  $old_pid

Use the second-window state-machine script only:
  run_rectangle_state_machine.sh

If this is stale, remove:
  rm -f $LOCK_FILE
EOF
    exit 2
  fi
fi
printf '%s\n' "$$" > "$LOCK_FILE"
mkdir -p "$LOG_DIR"

pids=()
SITL_PID_FILE="$LOG_DIR/arducopter.pid"
cleanup_started=0
cleanup() {
  if [[ "$cleanup_started" == "1" ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  printf '\nStopping APM sensor stack...\n'
  rm -f "$LOCK_FILE"
  if [[ -f "$SITL_PID_FILE" ]]; then
    sitl_pid=$(cat "$SITL_PID_FILE" 2>/dev/null || true)
    if [[ "$sitl_pid" =~ ^[0-9]+$ ]]; then
      kill -INT "$sitl_pid" 2>/dev/null || true
    fi
  fi
  for pid in "${pids[@]:-}"; do
    kill -INT "$pid" 2>/dev/null || true
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  sleep 1
  if [[ -n "${sitl_pid:-}" ]] && [[ "$sitl_pid" =~ ^[0-9]+$ ]]; then
    kill -TERM "$sitl_pid" 2>/dev/null || true
  fi
  for pid in "${pids[@]:-}"; do
    kill -TERM "$pid" 2>/dev/null || true
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 1
  if [[ -n "${sitl_pid:-}" ]] && [[ "$sitl_pid" =~ ^[0-9]+$ ]]; then
    kill -KILL "$sitl_pid" 2>/dev/null || true
  fi
  for pid in "${pids[@]:-}"; do
    kill -KILL "$pid" 2>/dev/null || true
    kill -KILL -- "-$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

printf 'Logs: %s\n' "$LOG_DIR"
printf 'World: %s\n' "$WORLD"
printf 'World name: %s\n' "$WORLD_NAME"
printf 'MID360 simulation bridge: %s\n' "$MID360_SIM_BRIDGE_MODE"
if [[ "$MID360_SIM_BRIDGE_MODE" == "direct_livox" ]]; then
  printf 'MID360 body exclusion: %s, x=[%s, %s], y=[%s, %s], z=[%s, %s] m\n' \
    "$MID360_BODY_FILTER_ENABLED" \
    "$MID360_BODY_MIN_X_M" "$MID360_BODY_MAX_X_M" \
    "$MID360_BODY_MIN_Y_M" "$MID360_BODY_MAX_Y_M" \
    "$MID360_BODY_MIN_Z_M" "$MID360_BODY_MAX_Z_M"
  printf 'MID360 body extrinsic: lidar origin [%.3f, %.3f, %.3f] m; pitch=+15 deg\n' \
    "$MID360_LIDAR_TO_BODY_X_M" "$MID360_LIDAR_TO_BODY_Y_M" \
    "$MID360_LIDAR_TO_BODY_Z_M"
fi

GPU_REPORT="$LOG_DIR/gpu_acceleration.log"
if ! bash "$PKG_SHARE/scripts/check_gpu_acceleration.sh" >"$GPU_REPORT" 2>&1; then
  cat "$GPU_REPORT" >&2
  printf 'GPU validation failed. Set REQUIRE_GAZEBO_GPU=0 only for an intentional CPU fallback.\n' >&2
  exit 3
fi
cat "$GPU_REPORT"

if [[ "${HEADLESS:-0}" == "1" ]]; then
  setsid gz sim -s -r --headless-rendering -v 2 "$WORLD" >"$LOG_DIR/gazebo.log" 2>&1 &
else
  setsid gz sim -r -v 2 --render-engine-gui ogre2 "$WORLD" >"$LOG_DIR/gazebo.log" 2>&1 &
fi
pids+=("$!")
# Headless rendering can take several seconds to initialise before the world
# transport topic is discoverable. Start the bridge after that settling period;
# the ROS-side advancing-clock check below remains the authoritative gate.
sleep "${GAZEBO_CLOCK_STARTUP_DELAY_S:-10}"

setsid ros2 run multi_slam_uav_sim gazebo_clock_bridge --ros-args \
  -p use_sim_time:=false \
  -p world_name:="$WORLD_NAME" \
  >"$LOG_DIR/ros_clock_bridge.log" 2>&1 &
pids+=("$!")

wait_for_single_clock_publisher() {
  local info publishers sample sec nanosec
  for _attempt in $(seq 1 "${ROS_CLOCK_OWNERSHIP_ATTEMPTS:-60}"); do
    info=$(timeout 5 ros2 topic info /clock --no-daemon 2>/dev/null) || info=
    publishers=$(awk -F: '/Publisher count:/ {gsub(/[[:space:]]/, "", $2); print $2; exit}' <<<"$info")
    if [[ "$publishers" =~ ^[0-9]+$ ]] && (( publishers > 1 )); then
      printf 'ROS /clock ownership error: publishers=%s; expected exactly one Gazebo clock bridge.\n' \
        "$publishers" >&2
      return 1
    fi
    if [[ "$publishers" == "1" ]]; then
      return 0
    fi
    sleep 0.5
  done
  # Cyclone DDS discovery can lag behind the bridge in WSL even though a
  # subscriber already receives the topic.  Do not tear down Gazebo solely
  # because the CLI graph query timed out; require an actual valid clock sample
  # and retain an explicit diagnostic for post-run publisher auditing.
  sample=$(timeout 8 ros2 topic echo /clock rosgraph_msgs/msg/Clock \
    --no-daemon --spin-time 2.0 --once --qos-reliability best_effort \
    2>/dev/null) || sample=
  sec=$(awk '$1 == "sec:" {print $2; exit}' <<<"$sample")
  nanosec=$(awk '$1 == "nanosec:" {print $2; exit}' <<<"$sample")
  if [[ "$sec" =~ ^[0-9]+$ && "$nanosec" =~ ^[0-9]+$ ]]; then
    printf 'ROS /clock publisher count unavailable after discovery timeout; valid sample received (single-owner check deferred to replay audit).\n' >&2
    return 0
  fi
  printf 'ROS /clock publisher did not appear after starting the Gazebo clock bridge.\n' >&2
  return 1
}

wait_for_single_clock_publisher

read_clock_ns() {
  local sample sec nanosec
  for _attempt in $(seq 1 "${ROS_CLOCK_ATTEMPTS:-15}"); do
    # Humble's ros2topic CLI does not reliably emit the nested Clock.clock
    # field when --field is combined with --no-daemon.  Read the complete
    # one-message sample; the parser below already selects sec/nanosec.
    sample=$(timeout 8 ros2 topic echo /clock rosgraph_msgs/msg/Clock \
      --no-daemon --spin-time 2.0 --once \
      --qos-reliability best_effort 2>/dev/null) || sample=
    sec=$(awk '$1 == "sec:" {print $2; exit}' <<<"$sample")
    nanosec=$(awk '$1 == "nanosec:" {print $2; exit}' <<<"$sample")
    if [[ "$sec" =~ ^[0-9]+$ && "$nanosec" =~ ^[0-9]+$ ]]; then
      printf '%s\n' "$((sec * 1000000000 + nanosec))"
      return 0
    fi
    sleep 0.5
  done
  return 1
}

clock_first_ns=$(read_clock_ns) || {
  printf 'ROS /clock did not produce a valid Gazebo simulation timestamp.\n' >&2
  exit 4
}
sleep 0.25
clock_second_ns=$(read_clock_ns) || {
  printf 'ROS /clock stopped before the second startup sample.\n' >&2
  exit 4
}
if (( clock_second_ns <= clock_first_ns )); then
  printf 'ROS /clock is not advancing: first=%s second=%s\n' \
    "$clock_first_ns" "$clock_second_ns" >&2
  exit 4
fi
printf 'ROS simulation clock active: delta=%.6fs\n' \
  "$(awk -v a="$clock_first_ns" -v b="$clock_second_ns" \
    'BEGIN {printf "%.6f", (b-a)/1e9}')"

if [[ "${ENABLE_D435_BRIDGE:-1}" == "1" ]]; then
  setsid ros2 run multi_slam_uav_sim d435i_sim_bridge --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p gz_prefix:=/front/d435i/gz \
    -p ros_prefix:=/front/d435i \
    -p publish_hz:=30.0 \
    -p publish_pointcloud:=${ENABLE_D435_POINTCLOUD:-false} \
    -p pointcloud_hz:=10.0 \
    -p pointcloud_stride:=4 \
    >"$LOG_DIR/d435i_sim_bridge.log" 2>&1 &
  pids+=("$!")
fi

if [[ "${ENABLE_GAZEBO_FLOW:-0}" == "1" \
      || "${ENABLE_FCU_FLOW:-0}" == "1" \
      || "${ENABLE_FCU_FLOW_ROUTER:-0}" == "1" ]]; then
  publish_to_fcu=false
  if [[ "${ENABLE_FCU_FLOW:-0}" == "1" ]]; then
    publish_to_fcu=true
  fi
  fcu_range_topic=""
  if [[ "${ENABLE_FCU_RANGE:-0}" == "1" || "${ENABLE_NONGPS_FLOW:-0}" == "1" ]]; then
    fcu_range_topic="/mavros/rangefinder_sub"
  fi
  setsid ros2 run multi_slam_uav_sim gz_rgbd_latest_bridge --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p gz_prefix:=/camera/camera \
    -p ros_prefix:=/camera/camera \
    -p publish_hz:=${FLOW_BRIDGE_HZ:-30.0} \
    -p publish_all_frames:=${FLOW_PUBLISH_ALL_FRAMES:-true} \
    -p restamp:=false \
    >"$LOG_DIR/gz_rgbd_latest_bridge.log" 2>&1 &
  pids+=("$!")

  flow_args=(
    -p use_sim_time:="$USE_SIM_TIME"
    -p image_topic:=/camera/camera/color/image_raw
    -p camera_info_topic:=/camera/camera/color/camera_info
    -p depth_topic:=/camera/camera/depth/image_rect_raw
    -p flow_topic:=/sim/optical_flow/raw
    -p rad_topic:=/sim/optical_flow/rad_native
    -p range_topic:=/sim/optical_flow/range_native
    -p gazebo_range_topic:=/flow/range
    -p gazebo_imu_topic:=/flow/imu
    -p imu_topic:=/livox/imu
    # The MTF companion path is consumed by a 10 Hz LiDAR-triggered backend.
    # A deterministic 15 Hz stream preserves fresh zero-motion observations
    # without spending CPU on frames the estimator cannot consume.
    -p max_rate_hz:=${FLOW_RATE_HZ:-15.0}
    -p angular_scale:=1.0
    # The camera intrinsics define the metric scale. Keep an explicit override
    # for A/B tests, but do not hide geometry errors behind an empirical gain.
    -p translation_scale:=${FLOW_TRANSLATION_SCALE:-1.0}
    -p use_physics_flow:=${FLOW_USE_PHYSICS:-false}
    -p use_gazebo_height:=false
    -p gazebo_world_name:="$WORLD_NAME"
    -p gazebo_height_model:=apm_iris
    -p publish_to_fcu:="$publish_to_fcu"
    -p fcu_flow_topic:=/mavros/optical_flow/raw/send
    -p restamp_output:=${FLOW_RESTAMP_OUTPUT:-false}
    -p debug:=${FLOW_DEBUG:-false}
  )
  if [[ -n "$fcu_range_topic" ]]; then
    flow_args+=(-p fcu_range_topic:="$fcu_range_topic")
  fi
  setsid ros2 run multi_slam_uav_sim gazebo_optical_flow_to_mavros --ros-args \
    "${flow_args[@]}" \
    >"$LOG_DIR/gazebo_optical_flow_to_mavros.log" 2>&1 &
  pids+=("$!")

  # Companion observations use the same Gazebo sensor clock as the MID360 IMU.
  setsid ros2 run multi_slam_uav_sim mtf01p_mavlink_bridge --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p mode:=sim \
    -p input_topic:=/sim/optical_flow/rad_native \
    -p flow_topic:=/sim/optical_flow/rad \
    -p range_topic:=/sim/optical_flow/range \
    -p raw_frame_topic:=/sim/mtf01/mavlink_frame \
    -p imu_topic:=/livox/imu \
    -p nominal_rate_hz:=${FLOW_RATE_HZ:-15.0} \
    -p restamp_output:=${MTF_RESTAMP_OUTPUT:-false} \
    -p report_path:="$LOG_DIR/mtf01_mavlink_bridge.json" \
    >"$LOG_DIR/mtf01_mavlink_bridge.log" 2>&1 &
  pids+=("$!")

  if [[ "${SHOW_FLOW_WINDOW:-0}" == "1" ]]; then
    setsid ros2 run multi_slam_uav_sim optical_flow_viewer --ros-args \
      -p use_sim_time:="$USE_SIM_TIME" \
      -p image_topic:=/camera/camera/color/image_raw \
      -p flow_topic:=/sim/optical_flow/raw \
      >"$LOG_DIR/optical_flow_viewer.log" 2>&1 &
    pids+=("$!")
  fi
fi

if [[ "${START_SITL:-1}" == "1" ]]; then
  sitl_defaults=(
    "Tools/autotest/default_params/copter.parm"
    "Tools/autotest/default_params/gazebo-iris.parm"
    "$PKG_SHARE/params/vision_mavros_guided.parm"
  )
  if [[ "${ENABLE_SITL_FLOW:-0}" == "1" ]]; then
    sitl_defaults+=("Tools/autotest/default_params/copter-optflow.parm")
  fi
  if [[ "${ENABLE_FCU_FLOW:-0}" == "1" ]]; then
    if [[ "${ENABLE_NONGPS_FLOW:-0}" == "1" ]]; then
      sitl_defaults+=("$PKG_SHARE/params/apm_mavlink_optflow_nongps.parm")
    else
      sitl_defaults+=("$PKG_SHARE/params/apm_mavlink_optflow_gps.parm")
    fi
  fi
  if [[ "${ENABLE_FCU_FLOW_ROUTER:-0}" == "1" ]]; then
    sitl_defaults+=("$PKG_SHARE/params/apm_mtf01p_routing.parm")
  fi
  if [[ "$ENABLE_EXTERNALNAV_EKF3" == "1" ]]; then
    sitl_defaults+=("$PKG_SHARE/params/apm_externalnav_gps_flow.parm")
  fi
  case "${ENABLE_IRIS_ROLL_STABILITY_PROFILE:-1}" in
    1)
      iris_stability_profile="$PKG_SHARE/params/iris_roll_stability.parm"
      if [[ ! -f "$iris_stability_profile" ]]; then
        printf 'Iris roll stability profile does not exist: %s\n' \
          "$iris_stability_profile" >&2
        exit 2
      fi
      sitl_defaults+=("$iris_stability_profile")
      printf 'SITL Iris roll stability profile: %s\n' \
        "$iris_stability_profile"
      ;;
    0) ;;
    *)
      printf 'ENABLE_IRIS_ROLL_STABILITY_PROFILE must be 0 or 1, got %s.\n' \
        "$ENABLE_IRIS_ROLL_STABILITY_PROFILE" >&2
      exit 2
      ;;
  esac
  if [[ -n "${SITL_EXTRA_DEFAULTS_FILE:-}" ]]; then
    if [[ ! -f "$SITL_EXTRA_DEFAULTS_FILE" ]]; then
      printf 'SITL extra defaults file does not exist: %s\n' \
        "$SITL_EXTRA_DEFAULTS_FILE" >&2
      exit 2
    fi
    sitl_defaults+=("$SITL_EXTRA_DEFAULTS_FILE")
    printf 'SITL extra defaults: %s\n' "$SITL_EXTRA_DEFAULTS_FILE"
  fi
  SITL_DEFAULTS=$(IFS=,; printf '%s' "${sitl_defaults[*]}")
  WIPE_ARG=""
  if [[ "${WIPE_EEPROM:-0}" == "1" ]]; then
    WIPE_ARG="-w"
  fi
  SITL_SERIAL_ARGS=""
  if [[ "${ENABLE_FCU_FLOW_ROUTER:-0}" == "1" ]]; then
    SITL_SERIAL_ARGS="--serial1 tcp:2"
  fi
  setsid bash -lc "cd '$ARDUPILOT_DIR' && echo \$\$ > '$SITL_PID_FILE' && exec build/sitl/bin/arducopter -S $WIPE_ARG $SITL_SERIAL_ARGS --model JSON --speedup 1 --slave 0 --defaults '$SITL_DEFAULTS' --sim-address=127.0.0.1 -I0" >"$LOG_DIR/sitl.log" 2>&1 &
  pids+=("$!")
  sleep 10
fi

if [[ "${ENABLE_FCU_FLOW_ROUTER:-0}" == "1" ]]; then
  setsid ros2 run multi_slam_uav_sim mtf01p_mavlink_sensor --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p input_topic:=/sim/optical_flow/rad \
    -p connection_url:=tcp:127.0.0.1:5762 \
    -p source_system:=200 \
    -p report_path:="$LOG_DIR/mtf01p_mavlink_sensor.json" \
    >"$LOG_DIR/mtf01p_mavlink_sensor.log" 2>&1 &
  pids+=("$!")

  setsid ros2 run multi_slam_uav_sim fcu_mavlink_flow_receiver --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p input_topic:=/uas1/mavlink_source \
    -p sensor_system_id:=200 \
    -p report_path:="$LOG_DIR/fcu_mavlink_flow_route.json" \
    >"$LOG_DIR/fcu_mavlink_flow_receiver.log" 2>&1 &
  pids+=("$!")
  sleep 2
fi

if [[ "${START_MAVROS:-1}" == "1" ]]; then
  if [[ -z "${MAVROS_PLUGINLISTS_FILE+x}" ]]; then
    if [[ "${ENABLE_FCU_FLOW:-0}" == "1" ||
      "${ENABLE_FCU_RANGE:-0}" == "1" ||
      "${ENABLE_NONGPS_FLOW:-0}" == "1" ]]; then
      MAVROS_PLUGINLISTS_FILE="$PKG_SHARE/config/mavros_validation_flow_pluginlists.yaml"
    else
      MAVROS_PLUGINLISTS_FILE="$PKG_SHARE/config/mavros_validation_pluginlists.yaml"
    fi
  fi
  if [[ ! -f "$MAVROS_PLUGINLISTS_FILE" ]]; then
    printf 'MAVROS plugin list is missing: %s\n' "$MAVROS_PLUGINLISTS_FILE" >&2
    exit 2
  fi
  setsid ros2 run mavros mavros_node --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    --params-file /opt/ros/humble/share/mavros/launch/apm_config.yaml \
    --params-file "$MAVROS_PLUGINLISTS_FILE" \
    --params-file "$PKG_SHARE/config/mavros_apm_rgbd.yaml" \
    >"$LOG_DIR/mavros.log" 2>&1 &
  pids+=("$!")
  sleep 4

  printf 'Waiting for MAVROS FCU connection...\n'
  for _ in {1..30}; do
    if grep -q 'CON: Got HEARTBEAT, connected' "$LOG_DIR/mavros.log" 2>/dev/null; then
      printf 'MAVROS FCU connected.\n'
      break
    fi
    sleep 1
  done
  printf 'Requesting ArduPilot telemetry streams for MAVROS pose/IMU/GPS/barometer topics...\n'
  : >"$LOG_DIR/mavros_stream_requester.log"
  mavros_imu_ready=0
  for stream_attempt in 1 2; do
    printf '\n--- telemetry request attempt %s ---\n' "$stream_attempt" \
      >>"$LOG_DIR/mavros_stream_requester.log"
    ros2 run multi_slam_uav_sim mavros_stream_requester --ros-args \
      -p use_sim_time:="$USE_SIM_TIME" \
      -p mavros_ns:=/mavros \
      -p timeout_s:=${MAVROS_STREAM_CONNECT_TIMEOUT_S:-60.0} \
      -p stream_rate_hz:=20 \
      -p position_rate_hz:=20.0 \
      -p imu_rate_hz:=100.0 \
      -p gps_rate_hz:=10.0 \
      -p barometer_rate_hz:=10.0 \
      >>"$LOG_DIR/mavros_stream_requester.log" 2>&1 || true
    if python3 "$PKG_SHARE/scripts/wait_for_ros_message.py" \
        --topic /mavros/imu/data_raw --timeout 15 \
        --reliability best_effort >/dev/null 2>&1; then
      mavros_imu_ready=1
      break
    fi
    sleep 2
  done
  if [[ "$mavros_imu_ready" != 1 ]]; then
    printf 'MAVROS connected but HIGHRES_IMU telemetry is unavailable.\n' >&2
    exit 5
  fi
fi

setsid ros2 run multi_slam_uav_sim flight_state_bridge --ros-args \
  -p use_sim_time:="$USE_SIM_TIME" \
  -p mavros_ns:=/mavros -p uav_ns:=/uav >"$LOG_DIR/flight_state_bridge.log" 2>&1 &
pids+=("$!")

if [[ "${ENABLE_SIM_BAROMETER:-1}" == "1" ]]; then
  BARO_REFERENCE_ALTITUDE_M=${BARO_REFERENCE_ALTITUDE_M:-584.0}
  setsid ros2 run multi_slam_uav_sim gz_barometer_sim --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p world_name:="$WORLD_NAME" \
    -p model_name:=apm_iris \
    -p link_name:=front_d435i_link \
    -p sensor_name:=barometer \
    -p publish_sim_topic:=true \
    -p publish_ros_topic:=false \
    -p reference_altitude_m:="$BARO_REFERENCE_ALTITUDE_M" \
    >"$LOG_DIR/gz_barometer_sim.log" 2>&1 &
  pids+=("$!")
  printf 'Gazebo barometer simulation: enabled (/sim/barometer/pressure)\n'
else
  printf 'Gazebo barometer simulation: disabled\n'
fi

if [[ "$MID360_SIM_BRIDGE_MODE" == "pointcloud_python" ]]; then
  setsid ros2 run multi_slam_uav_sim gz_mid360_pointcloud_bridge --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p gz_topic:=/mid360/lidar \
    -p gz_imu_topic:=/mid360/imu \
    -p raw_topic:=/sim/mid360/points_raw \
    -p imu_topic:=/sim/mid360/imu_raw \
    -p registered_topic:=/sim/mid360/cloud_registered \
    -p odom_topic:=/sim/mid360/ground_truth_odom \
    -p sensor_frame:=mid360_link \
    -p map_frame:=camera_init \
    -p point_stride:=${MID360_POINT_STRIDE:-1} \
    -p publish_registered:=${MID360_PUBLISH_REGISTERED:-true} \
    -p publish_tf:=${MID360_PUBLISH_TF:-true} \
    >"$LOG_DIR/gz_mid360_pointcloud_bridge.log" 2>&1 &
  pids+=("$!")
elif [[ "$MID360_SIM_BRIDGE_MODE" == "direct_livox" ]]; then
  # Bridge the independently simulated MID360 IMU and LiDAR from Gazebo.
  setsid ros2 run mid360_sim_bridge_cpp gz_livox_bridge_node --ros-args \
    -p use_sim_time:="$USE_SIM_TIME" \
    -p gz_topic:=/mid360/lidar \
    -p gz_imu_topic:=/mid360/imu \
    -p livox_lidar_topic:=/livox/lidar \
    -p livox_imu_topic:=/livox/imu \
    -p lidar_frame_id:=mid360_link \
    -p imu_frame_id:=base_link \
    -p point_stride:=${MID360_POINT_STRIDE:-1} \
    -p body_filter_enabled:="$MID360_BODY_FILTER_ENABLED" \
    -p body_min_x_m:="$MID360_BODY_MIN_X_M" \
    -p body_max_x_m:="$MID360_BODY_MAX_X_M" \
    -p body_min_y_m:="$MID360_BODY_MIN_Y_M" \
    -p body_max_y_m:="$MID360_BODY_MAX_Y_M" \
    -p body_min_z_m:="$MID360_BODY_MIN_Z_M" \
    -p body_max_z_m:="$MID360_BODY_MAX_Z_M" \
    -p lidar_to_body_translation:="[$MID360_LIDAR_TO_BODY_X_M, $MID360_LIDAR_TO_BODY_Y_M, $MID360_LIDAR_TO_BODY_Z_M]" \
    -p restamp_lidar:=false \
    -p publish_ground_truth_odom:=true \
    >"$LOG_DIR/gz_livox_bridge.log" 2>&1 &
  pids+=("$!")
fi

if [[ "$ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV" == "1" ]]; then
  setsid ros2 launch uf_sensor_pipeline gps_flow_externalnav.launch.py \
    use_sim_time:="$USE_SIM_TIME" \
    world_name:="$WORLD_NAME" \
    flow_truth_assistance:=${FLOW_USE_PHYSICS:-false} \
    performance_output_path:="$LOG_DIR/simulation_performance.json" \
    accuracy_output_path:="$LOG_DIR/externalnav_accuracy.json" \
    >"$LOG_DIR/gps_flow_externalnav.log" 2>&1 &
  pids+=("$!")
fi

if [[ "${RECTANGLE_FLOW_TEST:-0}" == "1" ]]; then
  setsid bash -lc "sleep 18; source /opt/ros/humble/setup.bash; source '$WS_INSTALL/setup.bash'; ros2 run multi_slam_uav_sim guided_rectangle_waypoints --ros-args -p use_sim_time:='$USE_SIM_TIME' -p takeoff_alt:=3.0 -p length_x:=6.0 -p length_y:=4.0 -p speed_mps:=0.8 -p land_at_end:=true" >"$LOG_DIR/guided_rectangle_waypoints.log" 2>&1 &
  pids+=("$!")
elif [[ "${AUTO_FLIGHT:-0}" == "1" ]]; then
  setsid bash -lc "sleep 18; source /opt/ros/humble/setup.bash; source '$WS_INSTALL/setup.bash'; ros2 run multi_slam_uav_sim guided_flight --ros-args -p use_sim_time:='$USE_SIM_TIME' -p takeoff_alt:=4.0 -p side_length:=5.0 -p hold_time:=5.0" >"$LOG_DIR/guided_flight.log" 2>&1 &
  pids+=("$!")
fi

cat <<EOF

APM UAV sensor stack is running.

Flight-controller-sourced topics:
  /uav/state
  /uav/local_pose
  /uav/local_odom
  /uav/global_fix
  /uav/imu
  /uav/velocity

Direct companion-computer sensors:
  /sim/mid360/points_raw
  /sim/mid360/cloud_registered
  /front/d435i/color/image_raw
  /front/d435i/depth/image_rect_raw
  /front/d435i/aligned_depth_to_color/image_raw
  /front/d435i/depth/color/points  (disabled by default; set ENABLE_D435_POINTCLOUD=true)
  /front/d435i/imu
  /camera/camera/color/image_raw
  /camera/camera/depth/image_rect_raw  (optional; optical flow can use Gazebo height)

Optical-flow test topic:
  /sim/optical_flow/raw  (mavros_msgs/msg/OpticalFlow, not sent to FCU)
  /sim/optical_flow/rad  (mavros_msgs/msg/OpticalFlowRad, MTF-01P-like diagnostics)
  /sim/optical_flow/range  (sensor_msgs/msg/Range)

Optional FCU optical-flow injection:
  ENABLE_FCU_FLOW=1 publishes optical flow to /mavros/optical_flow/raw/send
  ENABLE_FCU_RANGE=1 publishes range to /mavros/rangefinder_sub
  ENABLE_NONGPS_FLOW=1 enables FCU range and loads the optical-flow EKF source parameters

FCU-routed MTF01P observation path:
  ENABLE_FCU_FLOW_ROUTER=1 sends MAVLink1 OPTICAL_FLOW(100) and DISTANCE_SENSOR(132)
  MTF01P input: SERIAL1/tcp:5762 (MAV2_OPTIONS); companion link: SERIAL0/MAVROS (MAV1_OPTIONS)
  /fcu/mavlink/optical_flow and /fcu/mavlink/range decode /uas1/mavlink_source after routing
  Route report: $LOG_DIR/fcu_mavlink_flow_route.json

Optional companion GPS/flow ExternalNav:
  ENABLE_EXTERNALNAV_FUSION=1 starts /fusion/gps_flow/odom -> /mavros/odometry/out
  ENABLE_EXTERNALNAV_EKF3=1 configures EKF3 to consume ExternalNav without selecting a publishe
  ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV=1 starts only the legacy GPS/flow publishe
  FLOW_USE_PHYSICS=false is required for algorithm-quality evaluation
  MID360_SIM_BRIDGE_MODE=direct_livox uses C++: Gazebo LaserScan -> /livox/lidar CustomMsg
  MID360_SIM_BRIDGE_MODE=pointcloud_python retains /sim/mid360/points_raw for legacy testing
  MID360_SIM_BRIDGE_MODE=disabled starts no MID360 ROS adapte
  Real MID-360S must use the official livox_ros_driver2 and the same /livox/* interface;
  do not run the simulation adapter against real hardware.
  ENABLE_D435_BRIDGE=0 disables the D435 ROS bridge; the Gazebo sensor is lazy
  Performance report: $LOG_DIR/simulation_performance.json
  Accuracy report: $LOG_DIR/externalnav_accuracy.json

GPU diagnostics:
  Renderer selection and OpenCV backend: $GPU_REPORT
  REQUIRE_GAZEBO_GPU=1 rejects an unexpected adapter or software rendering
  HEADLESS=1 uses Gazebo OGRE2 EGL rendering without the GUI process

Optical-flow viewer:
  run_sim_with_flow.sh

GPS/GUIDED rectangle waypoint test:
  run_rectangle_state_machine.sh

EOF

wait "${pids[0]}"
