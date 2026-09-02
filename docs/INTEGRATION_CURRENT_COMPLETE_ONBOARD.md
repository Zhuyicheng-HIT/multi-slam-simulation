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

Validation: 71 Python tests for reliability, global pose graph and map maintenance
passed; YAML/XML/static checks passed; message/interface packages built. Full C++ build
and live replay are blocked on this server because `livox_ros_driver2` is unavailable.
No hardware acceptance was performed.
