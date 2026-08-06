
#!/usr/bin/env bash
# Shared, source-only lifecycle helpers for the D435i headless stack.

d435i_process_start_ticks() {
  local pid=$1
  [[ -r "/proc/$pid/stat" ]] || return 1
  awk '{print $22}' "/proc/$pid/stat"
}

d435i_active_reset() {
  D435I_ACTIVE_FORMAT=""
  D435I_ACTIVE_PID=""
  D435I_ACTIVE_PID_START_TICKS=""
  D435I_ACTIVE_STARTED_AT_UTC=""
  D435I_ACTIVE_STARTED_EPOCH_S=""
  D435I_ACTIVE_BRANCH=""
  D435I_ACTIVE_EXPERIMENT_ID=""
  D435I_ACTIVE_RUN_DIR=""
  D435I_ACTIVE_PROJECT_ROOT=""
  D435I_ACTIVE_RUN_TOKEN=""
  D435I_ACTIVE_WRAPPER_SCRIPT=""
}

d435i_active_read() {
  local marker=$1 key value legacy_pid legacy_run
  d435i_active_reset
  [[ -f "$marker" ]] || return 1
  if grep -q '^format_version=' "$marker" 2>/dev/null; then
    while IFS='=' read -r key value; do
      case "$key" in
        format_version) D435I_ACTIVE_FORMAT=$value ;;
        pid) D435I_ACTIVE_PID=$value ;;
        pid_start_ticks) D435I_ACTIVE_PID_START_TICKS=$value ;;
        started_at_utc) D435I_ACTIVE_STARTED_AT_UTC=$value ;;
        started_epoch_s) D435I_ACTIVE_STARTED_EPOCH_S=$value ;;
        branch) D435I_ACTIVE_BRANCH=$value ;;
        experiment_id) D435I_ACTIVE_EXPERIMENT_ID=$value ;;
        run_dir) D435I_ACTIVE_RUN_DIR=$value ;;
        project_root) D435I_ACTIVE_PROJECT_ROOT=$value ;;
        run_token) D435I_ACTIVE_RUN_TOKEN=$value ;;
        wrapper_script) D435I_ACTIVE_WRAPPER_SCRIPT=$value ;;
      esac
    done <"$marker"
  else
    read -r legacy_pid legacy_run <"$marker" || true
    D435I_ACTIVE_FORMAT=legacy
    D435I_ACTIVE_PID=${legacy_pid:-}
    D435I_ACTIVE_RUN_DIR=${legacy_run:-}
  fi
  [[ "$D435I_ACTIVE_PID" =~ ^[0-9]+$ && -n "$D435I_ACTIVE_RUN_DIR" ]]
}

d435i_active_pid_owned() {
  local pid=$1 project_root=$2 expected_ticks=${3:-} command actual_ticks
  [[ "$pid" =~ ^[0-9]+$ && -r "/proc/$pid/cmdline" ]] || return 1
  if [[ -n "$expected_ticks" ]]; then
    actual_ticks=$(d435i_process_start_ticks "$pid" 2>/dev/null || true)
    [[ "$actual_ticks" == "$expected_ticks" ]] || return 1
  fi
  command=$(tr '\0' ' ' <"/proc/$pid/cmdline")
  [[ "$command" == *"run_d435i_visual_slam_headless.sh"* ||
     "$command" == *"run_pr6_d435i_visual_headless.sh"* ]] || return 1
  [[ "$command" == *"$project_root/"* ]] || return 1
}

d435i_run_dir_owned() {
  local run_dir=$1 project_root=$2 resolved logs_root artifacts_root
  resolved=$(realpath -m "$run_dir")
  logs_root=$(realpath -m "$project_root/logs/d435i_visual_slam")
  artifacts_root=$(realpath -m "$project_root/artifacts")
  [[ "$resolved" == "$logs_root/"* || "$resolved" == "$artifacts_root/"* ]]
}

d435i_active_write() {
  local marker=$1 pid=$2 run_dir=$3 project_root=$4 branch=$5
  local experiment_id=$6 run_token=$7 wrapper_script=$8 temporary start_ticks
  temporary="${marker}.tmp.${pid}"
  start_ticks=$(d435i_process_start_ticks "$pid")
  mkdir -p "$(dirname "$marker")"
  umask 077
  {
    printf 'format_version=2\n'
    printf 'pid=%s\n' "$pid"
    printf 'pid_start_ticks=%s\n' "$start_ticks"
    printf 'started_at_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'started_epoch_s=%s\n' "$(date +%s)"
    printf 'branch=%s\n' "$branch"
    printf 'experiment_id=%s\n' "$experiment_id"
    printf 'run_dir=%s\n' "$run_dir"
    printf 'project_root=%s\n' "$project_root"
    printf 'run_token=%s\n' "$run_token"
    printf 'wrapper_script=%s\n' "$wrapper_script"
  } >"$temporary"
  mv -f -- "$temporary" "$marker"
}

d435i_active_archive() {
  local marker=$1 archive_dir=$2 reason=$3 target timestamp
  [[ -f "$marker" ]] || return 0
  timestamp=$(date -u +%Y%m%dT%H%M%S%NZ)
  mkdir -p "$archive_dir"
  target="$archive_dir/stale_active_${timestamp}_$$.env"
  cp -p -- "$marker" "$target"
  {
    printf 'archived_at_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'reason=%s\n' "$reason"
    printf 'source=%s\n' "$marker"
  } >"${target}.recovery"
  D435I_ACTIVE_ARCHIVE_PATH=$target
}

d435i_active_remove_owned() {
  local marker=$1 pid=$2 run_token=$3
  [[ -f "$marker" ]] || return 0
  d435i_active_read "$marker" || return 1
  [[ "$D435I_ACTIVE_PID" == "$pid" ]] || return 1
  [[ -n "$run_token" && "$D435I_ACTIVE_RUN_TOKEN" == "$run_token" ]] || return 1
  rm -f -- "$marker"
}

d435i_component_command_owned() {
  local component=$1 command=$2 project_root=$3
  case "$component" in
    stack_supervisor)
      [[ "$command" == *"$project_root/"* && "$command" == *"run_apm_sensor_stack.sh"* ]]
      ;;
    clock_bridge|clock_bridge_retry)
      [[ "$command" == *"parameter_bridge"* && "$command" == *"/clock"* ]]
      ;;
    ground_truth_bridge)
      [[ "$command" == *"gazebo_ground_truth_bridge"* &&
         ( "$command" == *"$project_root/"* ||
           "$command" == *"/opt/ros/humble/bin/ros2 run multi_slam_uav_sim"* ) ]]
      ;;
    rtabmap)
      [[ "$command" == *"d435i_rtabmap.launch.py"* ||
         "$command" == *"/opt/ros/humble/lib/rtabmap_odom/rgbd_odometry"* ||
         "$command" == *"/opt/ros/humble/lib/rtabmap_slam/rtabmap"* ]]
      ;;
    gazebo)
      [[ "$command" == *"gz sim"* && "$command" == *"$project_root/"* ]]
      ;;
    d435i_sim_bridge|d435i_rgbd_bridge_cpp|d435i_ros_gz_color|d435i_ros_gz_depth|d435i_ros_gz_aligned_depth|d435i_ros_gz_color_info|d435i_ros_gz_depth_info)
      [[ "$command" == *"d435i"* &&
         ( "$command" == *"$project_root/"* ||
           "$command" == *"/opt/ros/humble/bin/ros2 run d435i"* ||
           "$command" == *"/opt/ros/humble/bin/ros2 run ros_gz_bridge"* ) ]]
      ;;
    sitl)
      [[ "$command" == *"arducopter"* && "$command" == *"$project_root/"* ]]
      ;;
    mavros)
      [[ "$command" == *"mavros"* && "$command" == *"$project_root/"* ]]
      ;;
    flight_state_bridge)
      [[ "$command" == *"flight_state_bridge"* &&
         ( "$command" == *"$project_root/"* ||
           "$command" == *"/opt/ros/humble/bin/ros2 run multi_slam_uav_sim"* ) ]]
      ;;
    visual_degradation)
      [[ "$command" == *"d435i_visual_degradation"* &&
         ( "$command" == *"$project_root/"* ||
           "$command" == *"/opt/ros/humble/bin/ros2 run multi_slam_uav_sim"* ) ]]
      ;;
    visual_reliability)
      [[ "$command" == *"d435i_visual_reliability"* &&
         ( "$command" == *"$project_root/"* ||
           "$command" == *"/opt/ros/humble/bin/ros2 run multi_slam_uav_sim"* ) ]]
      ;;
    fastlio_supervisor)
      [[ "$command" == *"$project_root/"* &&
         "$command" == *"run_mid360_fastlio_mapping.sh"* ]]
      ;;
    integration_overlay)
      [[ ( "$command" == *"ros2 launch multi_slam_uav_sim"* &&
           "$command" == *"pr6_d435i_visual_integration.launch.py"* ) ||
         "$command" == *"$project_root/install/"* ||
         "$command" == /opt/ros/humble/lib/rtabmap_* ||
         "$command" == *"/opt/ros/humble/lib/tf2_ros/static_transform_publisher"* ]]
      ;;
    lio_adapter_fallback)
      [[ ( "$command" == *"ros2 launch uf_lio_adapter"* &&
           "$command" == *"lio_adapter.launch.py"* ) ||
         "$command" == *"$project_root/install/uf_lio_adapter/"* ]]
      ;;
    backend_fallback)
      [[ ( "$command" == *"ros2 launch uf_backend_fusion"* &&
           "$command" == *"online_backend_visual.launch.py"* ) ||
         "$command" == *"$project_root/install/uf_backend_fusion/"* ||
         "$command" == *"$project_root/install/uf_reliability/"* ||
         "$command" == *"$project_root/install/uf_sensor_pipeline/"* ||
         "$command" == *"$project_root/install/uf_relocalization/"* ]]
      ;;
    rectangle_motion)
      [[ "$command" == *"ros2 run multi_slam_uav_sim"* &&
         "$command" == *"guided_rectangle_waypoints"* ]]
      ;;
    *)
      [[ "$command" == *"$project_root/"* ]]
      ;;
  esac
}

d435i_group_records() {
  local expected_group=$1
  ps -eo pid=,pgid=,args= | awk -v expected="$expected_group" '
    $2 == expected {
      pid=$1; $1=""; $2=""; sub(/^[[:space:]]+/, "");
      print pid "\t" $0
    }'
}

d435i_signal_owned_group() {
  local signal=$1 component=$2 process_group=$3 expected_ticks=$4
  local project_root=$5 evidence_log=$6 record pid command actual_ticks
  local records=() owned=1
  mapfile -t records < <(d435i_group_records "$process_group")
  ((${#records[@]} > 0)) || return 0
  if [[ -n "$expected_ticks" && -r "/proc/$process_group/stat" ]]; then
    actual_ticks=$(d435i_process_start_ticks "$process_group" 2>/dev/null || true)
    if [[ "$actual_ticks" != "$expected_ticks" ]]; then
      printf 'REFUSE signal=%s component=%s pgid=%s reason=start_ticks_mismatch actual=%s expected=%s\n' \
        "$signal" "$component" "$process_group" "${actual_ticks:-missing}" \
        "$expected_ticks" >>"$evidence_log"
      return 1
    fi
  fi
  for record in "${records[@]}"; do
    IFS=$'\t' read -r pid command <<<"$record"
    if ! d435i_component_command_owned "$component" "$command" "$project_root"; then
      printf 'REFUSE signal=%s component=%s pgid=%s pid=%s command=%s\n' \
        "$signal" "$component" "$process_group" "$pid" "$command" \
        >>"$evidence_log"
      owned=0
    fi
  done
  [[ "$owned" == "1" ]] || return 1
  printf 'SIGNAL signal=%s component=%s pgid=%s members=%s\n' \
    "$signal" "$component" "$process_group" "${#records[@]}" >>"$evidence_log"
  kill -"$signal" -- "-$process_group" 2>/dev/null || true
}

d435i_load_manifest_records() {
  local manifest=$1 has_group=$2 component pid process_group start_ticks
  [[ -f "$manifest" ]] || return 0
  while IFS=$'\t' read -r component pid process_group start_ticks; do
    [[ "$pid" =~ ^[0-9]+$ ]] || continue
    if [[ "$has_group" != "1" ]]; then
      start_ticks=$process_group
      process_group=$pid
    fi
    printf '%s\t%s\t%s\n' "$component" "$process_group" "${start_ticks:-}"
  done < <(tail -n +2 "$manifest")
}

d435i_cleanup_run_manifests() {
  local run_dir=$1 project_root=$2 evidence_log=$3
  local records=() record component process_group start_ticks signal
  mkdir -p "$(dirname "$evidence_log")"
  {
    printf 'cleanup_started_at_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'run_dir=%s\nproject_root=%s\n' "$run_dir" "$project_root"
  } >>"$evidence_log"
  mapfile -t records < <(
    d435i_load_manifest_records "$run_dir/pids.tsv" 1
    d435i_load_manifest_records "$run_dir/stack_components.tsv" 0
  )
  for signal in INT TERM KILL; do
    for record in "${records[@]}"; do
      IFS=$'\t' read -r component process_group start_ticks <<<"$record"
      d435i_signal_owned_group "$signal" "$component" "$process_group" \
        "$start_ticks" "$project_root" "$evidence_log" || true
    done
    if [[ "$signal" != "KILL" ]]; then
      for _ in {1..10}; do sleep 0.2; done
    fi
  done
  printf 'cleanup_finished_at_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)" \
    >>"$evidence_log"
}

d435i_require_active_stack() {
  local marker=$1 project_root=$2
  if ! d435i_active_read "$marker"; then
    printf 'Invalid or missing active D435i marker: %s\n' "$marker" >&2
    return 2
  fi
  if ! kill -0 "$D435I_ACTIVE_PID" 2>/dev/null; then
    printf 'Recorded D435i wrapper is not running: %s\n' "$D435I_ACTIVE_PID" >&2
    return 2
  fi
  if ! d435i_active_pid_owned "$D435I_ACTIVE_PID" "$project_root" \
      "$D435I_ACTIVE_PID_START_TICKS"; then
    printf 'Recorded PID exists but is not this project headless wrapper: %s\n' \
      "$D435I_ACTIVE_PID" >&2
    return 3
  fi
}
