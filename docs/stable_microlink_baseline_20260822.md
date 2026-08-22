# Stable MicoLink Baseline

This baseline is based on `5b3e36a` (`feat: support multiple replay datasets`).
It retains the validated MicoLink optical-flow protocol and the replay viewer,
while excluding the later large-scene relocalization and textured-tunnel
experiments.

## Included

- MicoLink COM17 optical-flow framing, parser, unit conversion, and tests.
- The frozen five-source backend and its low-altitude ExternalNav contract.
- Dataset replay and seekable visualization tools already present at the base
  commit.
- Landing-time observer shutdown and ROS clock-regression handling.
- MAVROS validation plugin lists without the `param` plugin by default.
- One FCU IMU as the simulation IMU source; the optical-flow camera and range
  sensor remain enabled, while the optical-flow and D435i IMUs are removed.
- MID360 simulation reduced to 360 horizontal by 16 vertical samples.

## Excluded

- Large-tunnel texture and straight-route experiments.
- Passive/active relocalization campaign changes.
- RGB-D hard-depth admission and photometric downweighting experiments.
- Directional handoff, dynamic-scene, and other unverified post-baseline
  algorithm experiments.

## Verification

- `colcon build --symlink-install --packages-select multi_slam_uav_sim`
- `colcon test --packages-select multi_slam_uav_sim`: 129 tests passed.
- `colcon test-result --verbose`: 76 tests, 0 errors, 0 failures, 0 skipped.
- Shell syntax, XML parsing, sensor contract, and `git diff --check` passed.

The historical frozen-server reference associated with the earlier checkpoint
was approximately 3.14 cm causal 3D RMSE in the recorded validation log. This
document records provenance only; it does not claim a new flight reproduction.
