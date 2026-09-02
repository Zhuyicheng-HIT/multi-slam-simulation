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
