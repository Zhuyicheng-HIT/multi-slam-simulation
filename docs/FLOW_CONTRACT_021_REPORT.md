# FLOW-CONTRACT-021 Simulation Optical-Flow Contract

## Scope

This change repairs only the simulation input contract. It does not change the
backend optical-flow thresholds, HXY, GNSS/IMU weights, Dynamic, or visual
estimation algorithms.

Validation used the full FAST-LIO, reliability scheduler, and backend stack in
`large_indoor_tunnel_apm_rgbd_mid360.sdf`. The vehicle flew a 2.0 m by 1.2 m
rectangle at 2.2 m altitude, with four horizontal legs and three commanded
90-degree in-place turns. Truth was used only by the offline flow accuracy
observer.

## Root Causes

1. The long tunnel floor was nearly featureless in each 100 by 100 downward
   camera footprint. Its original transverse bands were 10 m apart. In the
   frozen replay, ground distance was valid for 440 airborne observations but
   quality was zero for all 951 flow messages, so no backend threshold could
   recover the missing measurement.
2. The generator subscribed to the same MID360 IMU twice. `/flow/imu` exposed
   raw pitched mount-frame rates, while `/livox/imu` was source-stamped and
   rotated by the MID360 bridge into `base_link` FLU. Whichever stream covered
   an exposure first could be selected, so the compensation frame contract was
   ambiguous.
3. Invalid ground distance before takeoff and after landing is expected because
   the range sensor is below its minimum working distance. During flight the
   Gazebo `/flow/range` measurement is valid and source-time paired; it must not
   be replaced with Gazebo truth height.

## Fix

- Applied the existing high-contrast checker texture to the tunnel floor
  visual. The floor collision remains the original single plane, so LiDAR
  geometry and the tunnel degeneracy benchmark are unchanged.
- Disabled the direct Gazebo gyro input and made `/livox/imu` the sole generator
  gyro source. It is converted once from ROS body FLU to flow-sensor FRD and is
  integrated over the exact image exposure timestamps. The MTF01P simulation
  bridge independently uses the same `/livox/imu` route for its output packet.
- Kept range input on `/flow/range`, source timestamps unchanged, and all
  backend thresholds unchanged.

## Full-Stack Result

Evidence directory: `logs/flow_contract_021_full3`.

At mission phase `landed`, before the longer post-flight accuracy observer:

| Quantity | Result |
| --- | ---: |
| Flow received by backend | 908 |
| Solver attempts | 873 |
| Factors admitted to solver | 708 |
| Solver admission / attempts | 81.10% |
| Solver admission / received | 77.97% |
| Scheduler-disabled attempts | 154 |
| Quality/distance-disabled attempts | 4 |
| Speed-disabled attempts | 0 |
| Rotation-disabled attempts | 0 |
| Clock-domain mismatches | 0 |
| Gyro integration missing / expired | 0 / 0 |
| Source timestamp regressions | 0 |

The remaining seven attempt/factor differences are early no-sample,
no-valid-observation, or rotation-phase transition exits that the current
backend does not expose as individual counters. During `route_active`, the
reliability scheduler enabled optical flow for 100% of its 132 samples. Most
scheduler rejections above occur before takeoff or during landing, not during
horizontal motion.

### Translation

The offline source-stamp association matched 339 translating observations:

- median quality: 164
- median ground distance: 1.85 m
- recovered scale: 0.988
- displacement RMSE: 0.0011 m
- correlation: 0.997
- expected axis/sign mapping: PASS

### Turns

During the commanded turns, generator diagnostics retained quality 122 to 146
and ground distance 1.84 to 1.86 m while MID360 integrated Z gyro was about
-0.027 to -0.033 rad per exposure. The backend reported zero rotation-gate
factor disables, 712 valid LOS diagnostics, zero invalid LOS diagnostics, and
continued producing body-horizontal factors. This demonstrates that turn
observations are timestamp-associated and admitted rather than mistaken for
translation or discarded by the rotation contract.

## Tests

- `46 passed, 2 deselected` for the optical-flow model, MTF IMU wait,
  MID360 route contract, and world coverage tests. The two deselected tests are
  pre-existing unrelated failures in `test_world_coverage.py` (relief-name
  scope and an undefined `positions` variable).
- `colcon build --packages-select multi_slam_uav_sim --symlink-install`: PASS.
- Empty-string ROS parameter launch parsing smoke test: PASS.
- Full moving/turning simulation: completed flight and landing; flow contract
  PASS. Its strict absolute-position gate is not part of this task and failed
  in the deliberately LiDAR-degenerate tunnel (XY RMSE 1.94 m); no estimator
  tuning was performed.
