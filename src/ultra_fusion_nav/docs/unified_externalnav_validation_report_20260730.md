# Unified ExternalNav Validation Report (2026-07-30)

## Completed

- Parsed the AT+TESTRN NMEA0183 contract for the planned GNSS module: 115200 8N1, RMC/GGA, checksum validation, UTC source time and ROS arrival time kept separately.
- Added a disabled-by-default NMEA GNSS node and a simulation GNSS metadata relay. The supplied RMC sample is intentionally rejected in strict mode because its body computes `*4E`, not the documented `*50`.
- Reserved the D435i interface and a static `base_link -> d435i_link` extrinsic with vision disabled by default. D435i point-cloud generation remains off.
- Added capability-level scheduler output: propagation, horizontal position/velocity/motion, vertical position and yaw tracking. A weak optional factor no longer zeros the full navigation support.
- Updated ExternalNav gate to publish at a fixed 20 Hz, accept `DEGRADED/RISK` when required capabilities remain observable, propagate the latest body-frame twist for at most 0.35 s, inflate covariance during propagation, and stop after source loss.
- Split EKF3 source configuration from the legacy GPS/flow ExternalNav publisher. `ENABLE_EXTERNALNAV_EKF3=1` no longer implies that the old publisher owns `/mavros/odometry/out`.
- Added a unified headless validation launcher with raw-point-cloud, FAST-LIO, unified-odometry and ExternalNav frequency gates.
- WSLg GPU check verified `D3D12 (NVIDIA GeForce RTX 5060 Laptop GPU)` with `Accelerated: yes`; GUI rendering is not used by the long-test launcher.

## Verification

- `colcon build --symlink-install --packages-up-to uf_interfaces uf_reliability uf_sensor_pipeline uf_backend_fusion multi_slam_uav_sim`: passed.
- `colcon test` for the five packages: passed; existing test result summary contains no errors or failures.
- ExternalNav runtime smoke: 10 Hz source -> 19.99 Hz output in `DEGRADED`; after horizontal-motion capability loss, zero additional outputs.
- `git diff --check`, shell syntax and Python compilation: passed.

## Current Blocker

The ordinary Gazebo map does not yet provide a reproducible real-time MID360 stream for the full flight test. The Gazebo transport topic `/mid360/lidar` exists, but the bridged `/sim/mid360/points_raw` was measured at about 2.1 Hz in one clean headless run instead of the configured 10 Hz, and some restarts produced an initial zero-output interval. FAST-LIO therefore sometimes starts with `No point, skip this scan` and the full `/Odometry -> unified backend -> ExternalNav` flight measurement cannot be accepted as valid evidence yet.

This is a simulation scheduling/data-source issue, not evidence that the new scheduler or ExternalNav gate is numerically unstable. No full-flight ATE/RPE result is claimed for this iteration.

## Next Controlled Step

1. Isolate the Gazebo GPU lidar sensor at a lower scan load or a dedicated sensor-only world and make `/sim/mid360/points_raw` sustain at least 5 Hz before starting FAST-LIO.
2. Keep the `CustomMsg`/per-point-time path as the hardware-aligned FAST-LIO path; retain PointCloud2 only as a separate compatibility experiment.
3. Re-run the gated ordinary-map rectangle test and publish the resulting LIO, unified odometry and ExternalNav rate/accuracy report.
