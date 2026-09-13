# Main Validation Progress

This document records validation performed on the merged `main` branch and
separates software evidence from simulation and hardware acceptance. It is a
progress record, not a release claim.

## Baseline

- Date: `2026-09-14`
- Commit: `3f3974c` (`test: rebuild live propagation measurement after anchor retry`)
- Branch: `main`
- ROS: ROS 2 Humble on Ubuntu 22.04.5 in WSL
- Gazebo: `8.15.0`
- CPU/RAM: AMD EPYC 9354, 64 logical CPUs, 62 GiB RAM
- GPU: NVIDIA RTX PRO 6000, WSLg direct rendering enabled
- External versions:
  - ArduPilot: `f9d619e26002d6aaa41643ee99c0ae0ee01e2247`
  - ArduPilot Gazebo: `082a0fe231f6e63bc8d1598f1cba461d9e2ea7f5`
  - Livox ROS Driver 2: `13eb05e4e6dd7a765b934d0c5fd6236676a57b49`
  - FAST-LIO: `a4743b095409588842a5b30ddfa27e29d2f99164`

## Passed

### Static and CI checks

- GitHub Actions CI passed for the previous code commit `873e3ea`.
- `tools/ci_smoke_test.py`: `33 passed`.
- All tracked Python files passed `py_compile`.
- All tracked shell files passed `bash -n`.
- 44 tracked YAML/YML files parsed successfully.
- 41 tracked XML/SDF files parsed successfully.
- `git diff --check` passed.
- No unresolved merge conflict markers in tracked source files.

### Build and unit tests

- Full `colcon build --symlink-install`: 20 packages completed successfully.
- `uf_map_maintenance` and `uf_global_pose_graph` build and tests passed.
- `mid360_sim_bridge_cpp`: 13 C++ tests passed.
- `uf_reliability`: 81 tests passed when run independently.
- After the anchor-retry test fix, `uf_backend_fusion` passed 301 tests.
- The affected-package `colcon test-result` reported 148 tests, 0 errors,
  0 failures, and 0 skipped.

### HybridFusion deterministic benchmark

The three-run benchmark completed successfully at:

`logs/hybridfusion/benchmark_validation_20260914/`

- Initial: 3/3 converged.
- GICP: 3/3 converged.
- Hybrid: 3/3 converged.
- All 9 method runs exited with code 0.
- Summary: `logs/hybridfusion/benchmark_validation_20260914/summary.md`.

This is a deterministic generated-data benchmark and is not evidence of real
sensor accuracy.

## Partially validated

- The first full `colcon test` run found one deterministic
  `live_propagation` anchor-retry test failure. The test double reused a stale
  IMU measurement after an anchor commit. The test was corrected to rebuild a
  measurement for the new anchor and the affected package then passed.
- The complete test suite was not rerun after that test-only correction. The
  affected package and previously passing package results are recorded above;
  a clean full-suite rerun remains recommended.
- Livox, FAST-LIO, ArduPilot SITL and ArduPilot Gazebo dependencies are present
  and discoverable after sourcing their overlays. No real sensor data was used.

## Not yet validated

### Gazebo and ArduPilot runtime

- No clean short rectangle/flight route has completed on this commit.
- Unified sensor topics, `/clock` single ownership, frame IDs, frequencies,
  timestamp monotonicity and packet loss are not yet recorded from a clean run.
- FAST-LIO to unified backend consumption is not yet evidenced by a runtime log.
- Unified odometry writer ownership and 5-10 minute static replay remain open.

The first startup attempt exposed a script invocation/path issue when calling
`src/multi_slam_uav_sim/scripts/run_apm_sensor_stack.sh` directly: it resolved
the workspace install directory as `/home/ld666` and attempted to source a
missing `/home/ld666/setup.bash`. The entrypoint needs a clean installed-workspace
invocation or a path-resolution fix before runtime acceptance.

### Visual and live HybridFusion

- Deterministic offline HybridFusion passed.
- Live RGB-D/CameraInfo/TF pairing, live exporter behavior, and non-interference
  with FAST-LIO/RTAB-Map remain untested.

### Hardware and flight

- Real MID360 static and motion tests: `HARDWARE_DATA_REQUIRED`.
- IMU unit scale, device/ROS time alignment, network loss, driver reconnect,
  external-navigation EKF3 and failsafe behavior: `HARDWARE_DATA_REQUIRED`.
- No real-flight acceptance claim is made.

## Next actions

1. Fix or standardize the `run_apm_sensor_stack.sh` installed-workspace entrypoint.
2. Run a clean headless Gazebo/ArduPilot short rectangle with exactly one
   sensor bridge and save raw logs/topic audits.
3. Run a clean full `colcon test` and archive `colcon test-result --verbose`.
4. Run the unified-odometry static replay and inspect anchor, timestamp,
   queue-discard and native-worker diagnostics.
5. Run live HybridFusion validation if the visual profile is enabled.
6. Perform MID360 and ExternalNav validation only with the real hardware and
   preserve the required sensor, timing, covariance and failsafe evidence.

## Related plan

The detailed checklist is [`VALIDATION_PLAN.md`](VALIDATION_PLAN.md).
