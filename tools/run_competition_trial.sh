#!/usr/bin/env bash
set -Eeuo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "$SCRIPT_DIR/.." && pwd)

usage() {
  cat <<'EOF'
Usage:
  run_competition_trial.sh --print-config MODE
  run_competition_trial.sh MODE [RUN_ID]

MODE is one of: metric1, metric2-full, metric2-no-camera,
metric3-four-source, metric3-five-source.

The estimator/sensor mode is fixed before takeoff. The script never changes
sources while the vehicle is flying. Set COMPETITION_RUNTIME_COMMAND to a
validated runtime launcher, or start the correctly configured runtime first.
EOF
}

configure_mode() {
  local mode=$1
  case "$mode" in
    metric1)
      enable_gnss=false
      enable_vision=false
      active_modalities=lidar,imu,optical_flow
      robustness_enabled=true
      robustness_profile=lidar_medium
      robustness_channels=native_lidar
      dynamic_observer=false
      manual_relocalization=false
      require_relocalization_database=false
      ;;
    metric2-full)
      enable_gnss=true
      enable_vision=true
      active_modalities=lidar,gnss,imu,optical_flow,vision
      robustness_enabled=false
      robustness_profile=nominal
      robustness_channels=none
      dynamic_observer=true
      manual_relocalization=false
      require_relocalization_database=false
      ;;
    metric2-no-camera)
      enable_gnss=true
      enable_vision=false
      active_modalities=lidar,gnss,imu,optical_flow
      robustness_enabled=false
      robustness_profile=nominal
      robustness_channels=none
      dynamic_observer=true
      manual_relocalization=false
      require_relocalization_database=false
      ;;
    metric3-four-source)
      enable_gnss=true
      enable_vision=false
      active_modalities=lidar,gnss,imu,optical_flow
      robustness_enabled=false
      robustness_profile=nominal
      robustness_channels=none
      dynamic_observer=false
      manual_relocalization=true
      require_relocalization_database=true
      ;;
    metric3|metric3-five-source)
      enable_gnss=true
      enable_vision=true
      active_modalities=lidar,gnss,imu,optical_flow,vision
      robustness_enabled=false
      robustness_profile=nominal
      robustness_channels=none
      dynamic_observer=false
      manual_relocalization=true
      require_relocalization_database=true
      ;;
    *)
      printf 'Unknown mode: %s\n' "$mode" >&2
      usage >&2
      return 2
      ;;
  esac
}

print_config() {
  printf 'mode=%s\n' "$mode"
  printf 'enable_gnss=%s\n' "$enable_gnss"
  printf 'enable_vision=%s\n' "$enable_vision"
  printf 'active_modalities=%s\n' "$active_modalities"
  printf 'robustness_enabled=%s\n' "$robustness_enabled"
  printf 'robustness_profile=%s\n' "$robustness_profile"
  printf 'robustness_channels=%s\n' "$robustness_channels"
  printf 'dynamic_observer=%s\n' "$dynamic_observer"
  printf 'manual_relocalization=%s\n' "$manual_relocalization"
  printf 'require_relocalization_database=%s\n' "$require_relocalization_database"
}

print_only=false
if [[ "${1:-}" == "--print-config" ]]; then
  print_only=true
  shift
fi
mode=${1:-}
[[ -n "$mode" ]] || { usage >&2; exit 2; }
shift || true
configure_mode "$mode"
if [[ "$print_only" == true ]]; then
  print_config
  exit 0
fi

run_id=${1:-${mode}_$(date +%Y%m%d_%H%M%S)}
run_root=${COMPETITION_RUN_ROOT:-$HOME/multi-slam-competition-runs}
run_dir=$run_root/$run_id
bag_dir=$run_dir/bag
[[ ! -e "$run_dir" ]] || {
  printf 'Run directory already exists; refusing to overwrite: %s\n' "$run_dir" >&2
  exit 2
}
mkdir -p "$run_dir"

set +u
source /opt/ros/humble/setup.bash
if [[ -f "$REPO_ROOT/install/setup.bash" ]]; then
  source "$REPO_ROOT/install/setup.bash"
fi
if [[ -n "${LIDAR_WS:-}" && -f "$LIDAR_WS/install/setup.bash" ]]; then
  source "$LIDAR_WS/install/setup.bash"
fi
set -u

mode_env=$run_dir/mode.env
cat >"$mode_env" <<EOF
# Generated before flight. Do not source this file during flight.
export COMPETITION_MODE='$mode'
export ENABLE_GNSS='$([[ "$enable_gnss" == true ]] && echo 1 || echo 0)'
export VISUAL_BRIDGE_ENABLED='$([[ "$enable_vision" == true ]] && echo 1 || echo 0)'
export VISUAL_FRONTEND_ENABLED='$([[ "$enable_vision" == true ]] && echo 1 || echo 0)'
export PR6_START_RTABMAP='$([[ "$enable_vision" == true ]] && echo 1 || echo 0)'
export ACTIVE_MODALITIES='[${active_modalities}]'
export ROBUSTNESS_ENABLED='$([[ "$robustness_enabled" == true ]] && echo 1 || echo 0)'
export ROBUSTNESS_PROFILE='$robustness_profile'
export ROBUSTNESS_CHANNELS='[${robustness_channels}]'
export START_DYNAMIC_OBSERVER='$([[ "$dynamic_observer" == true ]] && echo 1 || echo 0)'
export FASTLIO_DIAGNOSTIC_ODOMETRY='$([[ "$dynamic_observer" == true ]] && echo 1 || echo 0)'
export RELOCALIZATION_DATABASE_PATH='${RELOCALIZATION_DATABASE_PATH:-}'
export COMPETITION_PHYSICAL_LIDAR_DEGRADATION='${COMPETITION_PHYSICAL_LIDAR_DEGRADATION:-0}'
EOF
print_config >"$run_dir/mode.txt"
git -C "$REPO_ROOT" rev-parse HEAD >"$run_dir/source_commit.txt" 2>/dev/null || true

runtime_pid=
observer_pid=
cleanup() {
  local status=$?
  trap - EXIT INT TERM
  for pid in "$observer_pid" "$runtime_pid"; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    if kill -0 "$pid" 2>/dev/null; then
      kill -TERM -- "-$pid" 2>/dev/null || kill -TERM "$pid" 2>/dev/null || true
      wait "$pid" 2>/dev/null || true
    fi
  done
  exit "$status"
}
trap cleanup EXIT INT TERM

if [[ -n "${COMPETITION_RUNTIME_COMMAND:-}" ]]; then
  # shellcheck disable=SC1090
  source "$mode_env"
  setsid bash -lc "$COMPETITION_RUNTIME_COMMAND" >"$run_dir/runtime.log" 2>&1 &
  runtime_pid=$!
  printf 'Started configured runtime pid=%s\n' "$runtime_pid"
else
  printf 'Using the already-running onboard runtime; mode will be verified before recording.\n'
fi

wait_for_topic() {
  local topic=$1 timeout_s=${2:-20}
  local deadline=$((SECONDS + timeout_s))
  while (( SECONDS < deadline )); do
    if ros2 topic info "$topic" --no-daemon 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
      return 0
    fi
    sleep 1
  done
  printf 'Required topic has no publisher: %s\n' "$topic" >&2
  return 1
}

wait_for_topic /livox/lidar 30
wait_for_topic /livox/imu 30
wait_for_topic /sensors/optical_flow/rad 30
wait_for_topic /fusion/unified/odom 60

scheduler_modalities=$(ros2 param get /reliability_scheduler active_modalities \
  2>/dev/null || true)
[[ -n "$scheduler_modalities" ]] || {
  printf 'Cannot verify /reliability_scheduler active_modalities.\n' >&2
  exit 2
}
IFS=',' read -r -a expected_modalities <<<"$active_modalities"
for modality in "${expected_modalities[@]}"; do
  grep -qw "$modality" <<<"$scheduler_modalities" || {
    printf 'Scheduler mode mismatch: expected %s in %s\n' \
      "$modality" "$scheduler_modalities" >&2
    exit 2
  }
done
for modality in lidar imu optical_flow gnss vision; do
  if [[ ",$active_modalities," != *",$modality,"* ]] && \
      grep -qw "$modality" <<<"$scheduler_modalities"; then
    printf 'Scheduler still accepts disabled modality %s: %s\n' \
      "$modality" "$scheduler_modalities" >&2
    exit 2
  fi
done

if [[ "$enable_gnss" == true ]]; then
  wait_for_topic /sensors/gnss/fix 30
else
  if ros2 topic info /sensors/gnss/fix --no-daemon 2>/dev/null | \
      grep -Eq 'Publisher count: [1-9]'; then
    printf 'GNSS standardized output is still active; metric 1 refused.\n' >&2
    exit 2
  fi
fi
if [[ "$enable_vision" == true ]]; then
  wait_for_topic /sensors/rgbd/color 30
  wait_for_topic /sensors/rgbd/depth 30
else
  for topic in /sensors/rgbd/color /sensors/rgbd/depth; do
    if ros2 topic info "$topic" --no-daemon 2>/dev/null | \
        grep -Eq 'Publisher count: [1-9]'; then
      printf 'Vision standardized output is still active; %s refused.\n' "$mode" >&2
      exit 2
    fi
  done
fi

if [[ "$mode" == metric1 && "${COMPETITION_PHYSICAL_LIDAR_DEGRADATION:-0}" != 1 ]]; then
  profile=$(ros2 param get /robustness_v3_fault_injector profile 2>/dev/null || true)
  grep -q 'lidar_medium' <<<"$profile" || {
    printf 'Software lidar_medium profile is not active. Set physical-degradation flag only for a real degraded setup.\n' >&2
    exit 2
  }
fi

if [[ "$dynamic_observer" == true ]] && ! ros2 topic info \
    /dynamic_observer/statistics --no-daemon 2>/dev/null | grep -Eq 'Publisher count: [1-9]'; then
  setsid ros2 launch uf_dynamic_observer observer.launch.py \
    enabled:=true input_mode:=livox_custom \
    >"$run_dir/dynamic_observer.log" 2>&1 &
  observer_pid=$!
  wait_for_topic /dynamic_observer/statistics 30
fi
if [[ "$dynamic_observer" == true ]]; then
  dynamic_sample=$(timeout 12s ros2 topic echo /dynamic_observer/statistics \
    std_msgs/msg/String --once --no-daemon 2>/dev/null || true)
  [[ -n "$dynamic_sample" ]] || {
    printf 'Dynamic observer has no processed-scan statistics; trial refused.\n' >&2
    exit 2
  }
  dynamic_cloud=$(timeout 12s ros2 topic echo \
    /dynamic_observer/dynamic_candidates sensor_msgs/msg/PointCloud2 \
    --field header --once --no-daemon 2>/dev/null || true)
  [[ -n "$dynamic_cloud" ]] || {
    printf 'Dynamic observer has no candidate cloud; check FAST-LIO diagnostic pose and IMU coverage.\n' >&2
    exit 2
  }
fi

if [[ "$require_relocalization_database" == true ]]; then
  [[ -n "${RELOCALIZATION_DATABASE_PATH:-}" ]] || {
    printf 'Metric 3 requires RELOCALIZATION_DATABASE_PATH before takeoff.\n' >&2
    exit 2
  }
  [[ -d "$RELOCALIZATION_DATABASE_PATH" ]] || {
    printf 'Relocalization database directory does not exist: %s\n' \
      "$RELOCALIZATION_DATABASE_PATH" >&2
    exit 2
  }
  ready=$(timeout 8s ros2 topic echo /relocalization/ready std_msgs/msg/Bool \
    --once --no-daemon 2>/dev/null || true)
  grep -Eq 'data: true' <<<"$ready" || {
    printf 'Relocalization database is not ready; metric 3 recording refused.\n' >&2
    exit 2
  }
  ros2 service type /relocalization/manual_control 2>/dev/null | \
    grep -qx 'uf_interfaces/srv/ManualRelocalization' || {
      printf 'Manual relocalization service is unavailable.\n' >&2
      exit 2
    }
  for topic in /relocalization/request /mavros/setpoint_position/local; do
    publisher_count=$(ros2 topic info "$topic" --no-daemon 2>/dev/null | \
      sed -n 's/^Publisher count: //p' | head -n 1)
    [[ "$publisher_count" == 1 ]] || {
      printf 'Ownership preflight failed for %s: publishers=%s\n' \
        "$topic" "${publisher_count:-unknown}" >&2
      exit 2
    }
  done
fi

"$SCRIPT_DIR/check_competition_frequencies.sh" "$mode" "$run_dir/frequencies"
"$SCRIPT_DIR/competition_event.py" --mode "$mode" --event TRIAL_READY \
  --detail "$run_id" --repeat 10 --period 0.05

printf '\nRecording %s. Fly only after the preflight above passes.\n' "$mode"
if [[ "$manual_relocalization" == true ]]; then
  printf 'In another terminal trigger: bash tools/manual_relocalization.sh start\n'
fi
printf 'Press Ctrl+C once after landing/disarm; wait for bag verification.\n\n'
"$SCRIPT_DIR/record_competition_bag.sh" "$mode" "$bag_dir"

trap - EXIT INT TERM
cleanup
