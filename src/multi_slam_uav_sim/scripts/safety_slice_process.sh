#!/usr/bin/env bash

# Process helper for top-level simulation runners.  It deliberately refuses to
# create a second automatic setpoint owner.  The caller remains responsible for
# adding SAFETY_SLICE_PID to its normal process-group cleanup when non-empty.

safety_slice_start() {
  local raw_topic=${1:-/livox/lidar}
  local use_sim_time=${2:-true}
  local log_path=${3:-/tmp/uf_safety_slice.log}
  local nodes publisher_count

  SAFETY_SLICE_PID=""
  SAFETY_SLICE_OWNED=0
  nodes=$(timeout 5s ros2 node list --no-daemon --spin-time 1.0 2>/dev/null || true)
  if grep -qx '/flight_command_arbiter' <<<"$nodes"; then
    printf 'Reusing the existing flight_command_arbiter; no second setpoint owner will be started.\n'
    return 0
  fi

  publisher_count=$(timeout 5s ros2 topic info --no-daemon --spin-time 1.0 \
    /mavros/setpoint_position/local 2>/dev/null |
    sed -n 's/^Publisher count: \([0-9][0-9]*\)$/\1/p' || true)
  publisher_count=${publisher_count:-0}
  if (( publisher_count > 0 )); then
    printf 'Refusing to start safety slice: %s unknown automatic setpoint publisher(s) already exist.\n' \
      "$publisher_count" >&2
    return 2
  fi

  setsid ros2 launch uf_safety_supervisor safety_slice.launch.py \
    use_sim_time:="$use_sim_time" raw_lidar_topic:="$raw_topic" \
    >"$log_path" 2>&1 &
  SAFETY_SLICE_PID=$!
  SAFETY_SLICE_OWNED=1

  for _safety_attempt in $(seq 1 30); do
    nodes=$(timeout 5s ros2 node list --no-daemon --spin-time 1.0 2>/dev/null || true)
    if grep -qx '/flight_command_arbiter' <<<"$nodes" &&
       grep -qx '/raw_obstacle_safety_monitor' <<<"$nodes"; then
      printf 'Safety slice ready: raw=%s pid=%s\n' "$raw_topic" "$SAFETY_SLICE_PID"
      return 0
    fi
    if ! kill -0 "$SAFETY_SLICE_PID" 2>/dev/null; then
      printf 'Safety slice exited during startup; see %s\n' "$log_path" >&2
      return 3
    fi
    sleep 0.2
  done
  printf 'Safety slice did not become ready; see %s\n' "$log_path" >&2
  return 4
}

safety_slice_stop_owned() {
  if [[ "${SAFETY_SLICE_OWNED:-0}" == 1 &&
        "${SAFETY_SLICE_PID:-}" =~ ^[0-9]+$ ]]; then
    kill -INT -- "-$SAFETY_SLICE_PID" 2>/dev/null || true
    kill -INT "$SAFETY_SLICE_PID" 2>/dev/null || true
    wait "$SAFETY_SLICE_PID" 2>/dev/null || true
  fi
  SAFETY_SLICE_PID=""
  SAFETY_SLICE_OWNED=0
}
