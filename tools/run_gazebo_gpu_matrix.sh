#!/usr/bin/env bash
set -Eeo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)
LIDAR_WS=${LIDAR_WS:-$HOME/multi-slam-deps/mid360_ws}
OUTPUT_ROOT=${OUTPUT_ROOT:-$REPO_ROOT/logs/gazebo_gpu_matrix_$(date +%Y%m%d_%H%M%S)}
ADAPTERS=${ADAPTERS:-"AMD NVIDIA"}
PROFILES=${PROFILES:-"lidar lidar_flow lidar_flow_d435 full"}
WARMUP_S=${WARMUP_S:-20}
MEASURE_S=${MEASURE_S:-40}

source /opt/ros/humble/setup.bash
  source "$LIDAR_WS/install/local_setup.bash"
source "$REPO_ROOT/install/setup.bash"
mkdir -p "$OUTPUT_ROOT"

run_pids=()
cleanup_run() {
  for pid in "${run_pids[@]:-}"; do
    kill -INT -- "-$pid" 2>/dev/null || true
    kill -INT "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${run_pids[@]:-}"; do
    kill -TERM -- "-$pid" 2>/dev/null || true
    kill -TERM "$pid" 2>/dev/null || true
  done
  sleep 2
  for pid in "${run_pids[@]:-}"; do
    kill -KILL -- "-$pid" 2>/dev/null || true
    kill -KILL "$pid" 2>/dev/null || true
  done
  run_pids=()
  rm -f /tmp/multi_slam_apm_sensor_stack.lock
  ros2 daemon stop >/dev/null 2>&1 || true
}
trap cleanup_run EXIT INT TERM

wait_message() {
  local topic=$1
  local field=$2
  local timeout_s=$3
  local deadline=$((SECONDS + timeout_s))
  while ((SECONDS < deadline)); do
    if timeout 5s ros2 topic echo --once "$topic" --field "$field" \
        >/dev/null 2>&1; then
      printf 'ready: %s\n' "$topic"
      return 0
    fi
    sleep 1
  done
  printf 'timeout waiting for %s\n' "$topic" >&2
  ros2 topic list -t >&2 2>/dev/null || true
  ps -eo pid,stat,pcpu,pmem,comm,args --sort=-pcpu >&2 || true
  return 1
}

wait_topic_pattern() {
  local topic=$1
  local pattern=$2
  local timeout_s=$3
  local deadline=$((SECONDS + timeout_s))
  while ((SECONDS < deadline)); do
    if timeout 5s ros2 topic echo --once "$topic" 2>/dev/null \
        | grep -Eq "$pattern"; then
      printf 'ready: %s matches %s\n' "$topic" "$pattern"
      return 0
    fi
    sleep 1
  done
  printf 'timeout waiting for %s to match %s\n' "$topic" "$pattern" >&2
  return 1
}

profile_flags() {
  case "$1" in
    lidar)
      START_SITL=0; START_MAVROS=0; ENABLE_GAZEBO_FLOW=0; ENABLE_D435_BRIDGE=0
      ;;
    lidar_flow)
      START_SITL=0; START_MAVROS=0; ENABLE_GAZEBO_FLOW=1; ENABLE_D435_BRIDGE=0
      ;;
    lidar_flow_d435)
      START_SITL=0; START_MAVROS=0; ENABLE_GAZEBO_FLOW=1; ENABLE_D435_BRIDGE=1
      ;;
    full)
      START_SITL=1; START_MAVROS=1; ENABLE_GAZEBO_FLOW=1; ENABLE_D435_BRIDGE=1
      ;;
    *)
      printf 'Unsupported profile: %s\n' "$1" >&2
      return 2
      ;;
  esac
  export START_SITL START_MAVROS ENABLE_GAZEBO_FLOW ENABLE_D435_BRIDGE
}

ros_float() {
  local value=$1
  if [[ "$value" == *.* ]]; then
    printf '%s' "$value"
  else
    printf '%s.0' "$value"
  fi
}

run_case() {
  local adapter=$1
  local profile=$2
  local slug=${adapter,,}_${profile}
  local out="$OUTPUT_ROOT/$slug"
  mkdir -p "$out"
  cleanup_run
  profile_flags "$profile"

  printf '\n=== adapter=%s profile=%s ===\n' "$adapter" "$profile"
  setsid env \
    HEADLESS=1 \
    REQUIRE_GAZEBO_GPU=1 \
    GAZEBO_GPU_ADAPTER="$adapter" \
    MESA_D3D12_DEFAULT_ADAPTER_NAME="$adapter" \
    START_SITL="$START_SITL" \
    START_MAVROS="$START_MAVROS" \
    ENABLE_GAZEBO_FLOW="$ENABLE_GAZEBO_FLOW" \
    ENABLE_D435_BRIDGE="$ENABLE_D435_BRIDGE" \
    ENABLE_D435_POINTCLOUD=false \
    ENABLE_MID360_BRIDGE=0 \
    MID360_SIM_BRIDGE_MODE=direct_livox \
    MID360_POINT_STRIDE=1 \
    FLOW_PUBLISH_ALL_FRAMES=false \
    FLOW_BRIDGE_HZ=15.0 \
    ENABLE_EXTERNALNAV_EKF3=1 \
    ENABLE_LEGACY_GPS_FLOW_EXTERNALNAV=0 \
    WIPE_EEPROM=1 \
    LIDAR_WS="$LIDAR_WS" \
    LOG_DIR="$out/sim" \
    bash "$REPO_ROOT/install/multi_slam_uav_sim/share/multi_slam_uav_sim/scripts/run_apm_sensor_stack.sh" \
    >"$out/sim_launcher.log" 2>&1 &
  run_pids+=("$!")

  wait_message /livox/lidar point_num 90
  if [[ "$ENABLE_GAZEBO_FLOW" == "1" ]]; then
    wait_message /camera/camera/color/image_raw header.stamp 60
  fi
  if [[ "$ENABLE_D435_BRIDGE" == "1" ]]; then
    wait_message /front/d435i/color/image_raw header.stamp 60
    wait_message /front/d435i/depth/image_rect_raw header.stamp 60
  fi

  if [[ "$profile" == "full" ]]; then
    wait_message /mavros/imu/data_raw header.stamp 60
    setsid env LIDAR_WS="$LIDAR_WS" RVIZ=0 FASTLIO_INPUT_MODE=livox \
      FASTLIO_NATIVE_FACTOR_EXPORT=1 \
      START_LIVOX_POINTCLOUD_BRIDGE=0 LOG_DIR="$out/fastlio" \
      bash "$REPO_ROOT/tools/run_fastlio_mapping.sh" \
      >"$out/fastlio_launcher.log" 2>&1 &
    run_pids+=("$!")
    wait_message /Odometry header.stamp 90

    setsid env ENABLE_VISION=false LOG_DIR="$out/unified" \
      bash "$REPO_ROOT/tools/run_unified_backend_stack.sh" \
      >"$out/unified_launcher.log" 2>&1 &
    run_pids+=("$!")
    wait_message /fusion/unified/odom header.stamp 60
    wait_message /mavros/odometry/out header.stamp 60

    setsid env ENABLE_FLOW_ACCURACY=0 LOG_DIR="$out/flight" \
      bash "$REPO_ROOT/tools/run_rectangle_state_machine.sh" \
      >"$out/flight_launcher.log" 2>&1 &
    run_pids+=("$!")
    wait_topic_pattern /mavros/state '^armed: true$' 90
  fi

  setsid ros2 run multi_slam_uav_sim simulation_performance_monitor --ros-args \
    -p world_name:=simple_apm_rgbd_mid360 \
    -p rtf_topic:=/simulation/rtf \
    -p window_s:="$(ros_float "$MEASURE_S")" \
    -p report_period_s:=2.0 \
    -p minimum_live_rtf:=0.8 \
    -p fusion_topic:=/fusion/unified/odom \
    -p fusion_diagnostic_topic:=/fusion/unified/diagnostics \
    -p output_path:="$out/performance.json" \
    >"$out/performance.log" 2>&1 &
  run_pids+=("$!")

  printf 'warming up for %ss...\n' "$WARMUP_S"
  sleep "$WARMUP_S"
  ros2 daemon stop >/dev/null 2>&1 || true
  printf 'measuring for %ss...\n' "$MEASURE_S"
  for ((second = 0; second < MEASURE_S; ++second)); do
    printf 'sample=%d wall=%s\n' "$second" "$(date +%s.%N)" >>"$out/process_samples.txt"
    ps -eo pid=,pcpu=,pmem=,rss=,comm=,args= --sort=-pcpu \
      | grep -E 'gz sim|gz_livox_bridge|d435i_sim_bridge|gz_rgbd_latest|gazebo_optical|mtf01p_mavlink|arducopter|mavros_node|fastlio_mapping|fast_lio|online_backend|external_nav_gate' \
      >>"$out/process_samples.txt" || true
    nvidia-smi --query-gpu=timestamp,name,utilization.gpu,memory.used \
      --format=csv,noheader,nounits >>"$out/nvidia_samples.csv" 2>/dev/null || true
    sleep 1
  done

  ps -eo pid,pcpu,pmem,rss,comm,args --sort=-pcpu >"$out/processes_final.txt"
  cp "$out/sim/gpu_acceleration.log" "$out/gpu_acceleration.log"
  printf '%s\n' "$adapter" >"$out/adapter.txt"
  printf '%s\n' "$profile" >"$out/profile.txt"
  printf '1.0\n' >"$out/configured_rtf.txt"
  cleanup_run
}

for adapter in $ADAPTERS; do
  for profile in $PROFILES; do
    run_case "$adapter" "$profile"
  done
done

python3 "$REPO_ROOT/tools/summarize_gazebo_gpu_matrix.py" \
  --input "$OUTPUT_ROOT" \
  --csv "$OUTPUT_ROOT/summary.csv" \
  --json "$OUTPUT_ROOT/summary.json"
printf '\nmatrix_complete: %s\n' "$OUTPUT_ROOT"
