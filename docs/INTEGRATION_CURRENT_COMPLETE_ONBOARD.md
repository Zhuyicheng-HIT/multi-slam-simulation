# Current Complete Onboard Integration

Baseline: `origin/integration/onboard-three-source-v1`
(`c6ebc8c054c0a10e94301a489ecb180e5f8a16a1`).

Branch: `integration/current-complete-onboard-v1`.

Integrated ownership layers:

- MID360S mount/IMU unit contract and C++ body filter.
- Dynamic Observer v2, fail-open Clean Gateway, epoch reseed and static-map refinement.
- Raw obstacle safety, local avoidance and the single flight command arbiter.
- Relocalization request arbiter, active relocalization controller and risk shadow evaluator.
- Immutable/offline map maintenance and global SE(3) pose graph (offline, disabled by default).

The backend fusion implementation, ExternalNav semantics, one-observation-one-factor,
Z-axis logic, directional LiDAR experiments, bags, databases and generated artifacts
were not imported. Dynamic, active relocalization and offline map launch profiles remain
disabled by default; Clean remains fail-open and Raw LiDAR remains the safety input.

The scheduler now publishes `RelocalizationRequestIntent`; the arbiter is the sole
publisher of `/relocalization/request`. `flight_command_arbiter` is the sole automatic
publisher of `/mavros/setpoint_position/local`.

External dependency: the repository does not pin a Livox driver commit. The required
ROS 2 package is `livox_ros_driver2` 1.0.0, sourced from the official repository at
external commit `13eb05e` in `$HOME/multi-slam-deps/mid360_ws`; its source is not part
of this repository and no sudo install was used.

Validation on 2026-09-02: full `colcon build --symlink-install` completed for all 22
workspace packages. `colcon test` ran 263 tests: 260 passed, 3 failed. The failures are
the inherited safety ownership tests: legacy guided demo nodes still contain direct
MAVROS setpoint literals, and the PR23 launch scripts do not call the newer
`safety_slice_start` helper. These command entrypoints were intentionally left byte-
for-byte unchanged per the PR23 command contract. C++ conversion/body-filter,
relocalization, dynamic, map, backend, and interface tests otherwise built and ran;
`git diff --check` passed. YAML/XML/Python syntax checks passed. Direct pytest outside
the colcon environment is not authoritative here because this server's default Python
interpreter cannot import ROS `rclpy`.

Command contract SHA256 remains unchanged for the four PR23 launch/stop files recorded
before integration. No Gazebo/live replay or hardware acceptance was performed on this
server; NUC validation remains required.

## Server ownership closure

The three original ownership failures were traced to direct setpoint publishers in
`guided_flight.py`, `guided_rectangle_waypoints.py`, and `rectangle_flow_test.py`, plus
an inherited test expectation that the unchanged PR23 shell entrypoints call a
`safety_slice_start` helper which is not present in that command contract. The three
Python nodes now publish `/autonomy/intent/mission/pose`; only
`flight_command_arbiter` publishes automatic `/mavros/setpoint_position/local`.
The shell entrypoints were deliberately not edited, so their recorded SHA256 values
remain unchanged.

After the fix: full build remains 22/22 packages. The full suite remains 263 tests,
with 262 passing and only the one `safety_slice_start` assertion failing. This is a
test/command-contract mismatch, not a second MAVROS publisher. Server-verified items
are build, C++/Python unit tests, offline benchmarks, and static ownership checks;
NUC, MID360, D435i, FCU, real ROS graph, and field execution remain onboard-pending.

## SERVER-SAFETY-BOOTSTRAP-CLOSURE-002 audit

The historical `safety_slice_start` contract (from `06894236`) performs duplicate-
owner detection, launches `uf_safety_supervisor/safety_slice.launch.py` in its own
session, waits for `flight_command_arbiter` and `raw_obstacle_safety_monitor`, and
returns its process-group PID for the caller's TERM/INT/KILL cleanup. The current
PR23 `run_apm_sensor_stack.sh`, `run_rectangle_state_machine.sh`,
`run_s_curve_state_machine.sh`, and `run_pr6_d435i_visual_headless.sh` do not call
this helper; the automatic mission path therefore does not prove an arbiter is
running. The ownership test correctly exposes that gap.

The guided nodes now publish mission intents, but restoring the historical call site
would change the frozen PR23 command-file SHA256 values. No unsafe detached-process
workaround was added. Consequently this server closure remains blocked until the
command-contract owner permits a coordinated SHA update or supplies an equivalent
entrypoint outside the frozen files.

## SERVER-SAFETY-BOOTSTRAP-FINAL-003

The four user-facing commands retain their paths, arguments, and ordering. They now
source the historical `safety_slice_process.sh` contract and call `safety_slice_start`
before automatic guided work. The helper detects duplicate owners, waits for the
arbiter and raw obstacle monitor, and returns a process-group handle for cleanup.
Guided nodes publish only mission intent. A no-graph launch failed closed; a ROS
domain smoke reached safety-slice ready and was explicitly cleaned up.

The previous PR23 `run_apm_sensor_stack.sh` SHA256 was
`1638662c59a1ada8d1c70ec8f2fddcc4f516e2d0def1c7cd780c539f39bf334d`; integrated
command SHAs are captured in the validation log after this one-time safety change.

Final server validation: 22/22 packages build and 263/263 tests pass. Server-verified
coverage is build, unit/integration tests, offline benchmarks, ownership checks, and
safety-slice lifecycle smoke. NUC, MID360, D435i, FCU, real ROS graph, and flight
execution remain onboard-pending.
