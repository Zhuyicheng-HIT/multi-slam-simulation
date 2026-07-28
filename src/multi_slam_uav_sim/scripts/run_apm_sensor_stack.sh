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
WORLD=${WORLD:-$PKG_SHARE/worlds/simple_apm_rgbd_mid360.sdf}
WORLD_NAME=${WORLD_NAME:-simple_apm_rgbd_mid360}
LOG_DIR=${LOG_DIR:-$WS_ROOT/logs/apm_sensor_stack_$(date +%Y%m%d_%H%M%S)}
LOCK_FILE=${LOCK_FILE:-/tmp/multi_slam_apm_sensor_stack.lock}

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
cleanup_started=0
cleanup() {
  if [[ "$cleanup_started" == "1" ]]; then
    return
  fi
  cleanup_started=1
  trap - EXIT INT TERM
  printf '\nStopping APM sensor stack...\n'
  rm -f "$LOCK_FILE"
  for pid in "${pids[@]:-}"; do
    kill -INT "$pid" 2>/dev/null || true
    kill -INT -- "-$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]:-}"; do
    kill -TERM "$pid" 2>/dev/null || true
    kill -TERM -- "-$pid" 2>/dev/null || true
  done
  sleep 1
  for pid in "${pids[@]:-}"; do
    kill -KILL "$pid" 2>/dev/null || true
    kill -KILL -- "-$pid" 2>/dev/null || true
  done
}
trap cleanup EXIT INT TERM

printf 'Logs: %s\n' "$LOG_DIR"
printf 'World: %s\n' "$WORLD"
printf 'World name: %s\n' "$WORLD_NAME"

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
sleep 6

if [[ "${ENABLE_D435_BRIDGE:-1}" == "1" ]]; then
  setsid ros2 run multi_slam_uav_sim d435i_sim_bridge --ros-args \
    -p gz_prefix:=/front/d435i/gz \
    -p ros_prefix:=/front/d435i \
    -p publish_hz:=30.0 \
    -p publish_pointcloud:=${ENABLE_D435_POINTCLOUD:-false} \
    -p pointcloud_hz:=10.0 \
    -p pointcloud_stride:=4 \
    >"$LOG_DIR/d435i_sim_bridge.log" 2>&1 &
  pids+=("$!")
fi

if [[ "${ENABLE_GAZEBO_FLOW:-0}" == "1" || "${ENABLE_FCU_FLOW:-0}" == "1" ]]; then
  publish_to_fcu=false
  if [[ "${ENABLE_FCU_FLOW:-0}" == "1" ]]; then
    publish_to_fcu=true
  fi
  fcu_range_topic=""
  if [[ "${ENABLE_FCU_RANGE:-0}" == "1" || "${ENABLE_NONGPS_FLOW:-0}" == "1" ]]; then
    fcu_range_topic="/mavros/rangefinder_sub"
  fi
  setsid ros2 run multi_slam_uav_sim gz_rgbd_latest_bridge --ros-args \
    -p gz_prefix:=/camera/camera \
    -p ros_prefix:=/camera/camera \
    -p publish_hz:=${FLOW_BRIDGE_HZ:-30.0} \
    -p publish_all_frames:=${FLOW_PUBLISH_ALL_FRAMES:-true} \
    -p restamp:=false \
    >"$LOG_DIR/gz_rgbd_latest_bridge.log" 2>&1 &
  pids+=("$!")

  flow_args=(
    -p image_topic:=/camera/camera/color/image_raw
    -p camera_info_topic:=/camera/camera/color/camera_info
    -p depth_topic:=/camera/camera/depth/image_rect_raw
    -p flow_topic:=/sim/optical_flow/raw
    -p gazebo_range_topic:=/flow/range
    -p gazebo_imu_topic:=/flow/imu
    -p imu_topic:=/mavros/imu/data_raw
    -p max_rate_hz:=30.0
    -p angular_scale:=1.0
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

  if [[ "${SHOW_FLOW_WINDOW:-0}" == "1" ]]; then
    setsid ros2 run multi_slam_uav_sim optical_flow_viewer --ros-args \
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
  if [[ "${ENABLE_EXTERNALNAV_FUSION:-0}" == "1" ]]; then
    sitl_defaults+=("$PKG_SHARE/params/apm_externalnav_gps_flow.parm")
  fi
  SITL_DEFAULTS=$(IFS=,; printf '%s' "${sitl_defaults[*]}")
  WIPE_ARG=""
  if [[ "${WIPE_EEPROM:-0}" == "1" ]]; then
    WIPE_ARG="-w"
  fi
  setsid bash -lc "cd '$ARDUPILOT_DIR' && build/sitl/bin/arducopter -S $WIPE_ARG --model JSON --speedup 1 --slave 0 --defaults '$SITL_DEFAULTS' --sim-address=127.0.0.1 -I0" >"$LOG_DIR/sitl.log" 2>&1 &
  pids+=("$!")
  sleep 10
fi

if [[ "${START_MAVROS:-1}" == "1" ]]; then
  setsid ros2 run mavros mavros_node --ros-args \
    --params-file /opt/ros/humble/share/mavros/launch/apm_config.yaml \
    --params-file /opt/ros/humble/share/mavros/launch/apm_pluginlists.yaml \
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
  printf 'Requesting ArduPilot telemetry streams for MAVROS pose/IMU/GPS topics...\n'
  ros2 run multi_slam_uav_sim mavros_stream_requester --ros-args \
    -p mavros_ns:=/mavros \
    -p stream_rate_hz:=20 \
    -p position_rate_hz:=20.0 \
    -p imu_rate_hz:=100.0 \
    -p gps_rate_hz:=10.0 \
    >"$LOG_DIR/mavros_stream_requester.log" 2>&1 || true
fi

setsid ros2 run multi_slam_uav_sim flight_state_bridge --ros-args \
  -p mavros_ns:=/mavros -p uav_ns:=/uav >"$LOG_DIR/flight_state_bridge.log" 2>&1 &
pids+=("$!")

if [[ "${ENABLE_MID360_BRIDGE:-1}" == "1" ]]; then
  setsid ros2 run multi_slam_uav_sim gz_mid360_pointcloud_bridge --ros-args \
    -p gz_topic:=/mid360/lidar \
    -p raw_topic:=/sim/mid360/points_raw \
    -p registered_topic:=/sim/mid360/cloud_registered \
    -p odom_topic:=/sim/mid360/ground_truth_odom \
    -p sensor_frame:=mid360_link \
    -p map_frame:=camera_init \
    >"$LOG_DIR/gz_mid360_pointcloud_bridge.log" 2>&1 &
  pids+=("$!")
fi

if [[ "${ENABLE_EXTERNALNAV_FUSION:-0}" == "1" ]]; then
  setsid ros2 launch uf_sensor_pipeline gps_flow_externalnav.launch.py \
    world_name:="$WORLD_NAME" \
    flow_truth_assistance:=${FLOW_USE_PHYSICS:-false} \
    performance_output_path:="$LOG_DIR/simulation_performance.json" \
    accuracy_output_path:="$LOG_DIR/externalnav_accuracy.json" \
    >"$LOG_DIR/gps_flow_externalnav.log" 2>&1 &
  pids+=("$!")
fi

if [[ "${RECTANGLE_FLOW_TEST:-0}" == "1" ]]; then
  setsid bash -lc "sleep 18; source /opt/ros/humble/setup.bash; source '$WS_INSTALL/setup.bash'; ros2 run multi_slam_uav_sim guided_rectangle_waypoints --ros-args -p takeoff_alt:=3.0 -p length_x:=6.0 -p length_y:=4.0 -p speed_mps:=0.8 -p land_at_end:=true" >"$LOG_DIR/guided_rectangle_waypoints.log" 2>&1 &
  pids+=("$!")
elif [[ "${AUTO_FLIGHT:-0}" == "1" ]]; then
  setsid bash -lc "sleep 18; source /opt/ros/humble/setup.bash; source '$WS_INSTALL/setup.bash'; ros2 run multi_slam_uav_sim guided_flight --ros-args -p takeoff_alt:=4.0 -p side_length:=5.0 -p hold_time:=5.0" >"$LOG_DIR/guided_flight.log" 2>&1 &
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

Optional companion GPS/flow ExternalNav:
  ENABLE_EXTERNALNAV_FUSION=1 starts /fusion/gps_flow/odom -> /mavros/odometry/out
  FLOW_USE_PHYSICS=false is required for algorithm-quality evaluation
  ENABLE_D435_BRIDGE=0 and ENABLE_MID360_BRIDGE=0 disable unused ROS conversion bridges
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
