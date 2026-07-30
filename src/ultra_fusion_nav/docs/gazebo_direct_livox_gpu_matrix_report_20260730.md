# Direct Livox Simulation Bridge and GPU Matrix

Date: 2026-07-30

## Scope

This change replaces the normal simulation path from a Python PointCloud2
converter with a direct C++ Gazebo `LaserScan` to
`livox_ros_driver2/msg/CustomMsg` adapter. The real MID-360S path remains the
official `livox_ros_driver2` driver, so FAST-LIO receives the same `/livox/lidar`
and `/livox/imu` interfaces in either case.

```text
Gazebo /mid360/lidar -- C++ adapter --> /livox/lidar CustomMsg --> FAST-LIO
FCU /mavros/imu/data_raw -----------> /livox/imu -------------> FAST-LIO

Real MID-360S -- official livox_ros_driver2 --> /livox/* ------> FAST-LIO
```

The C++ adapter publishes `/sim/mid360/ground_truth_odom` only as an evaluation
topic. It is not an input to FAST-LIO, the unified backend, or ExternalNav.

## Validation

`mid360_sim_bridge_cpp` unit tests passed: four scan lines, per-point scan-time
offsets, and reflectivity clamping. The direct adapter smoke test used the
official CustomMsg definition and completed with an average bridge CPU load of
about 3.3 percent in sensor-only profiles. The prior Python converter consumed
about 36 to 46 percent in comparable earlier probes.

The complete GPU matrix selects the intended D3D12 renderer and keeps the
world configuration at `<real_time_factor>1.0`; it does not deliberately dilate
simulation time. Measured RTF below is the effective interval ratio
`delta(simulation time) / delta(wall time)`, not the misleading median of
instantaneous world-stat samples.

## Sensor Matrix

Measurements are in `logs/gpu_matrix_final_sensor/summary.csv`. Rates are
wall-clock arrival rates, rounded to two decimals.

| Profile | AMD 610M RTF | RTX 5060 RTF | AMD LiDAR Hz | RTX LiDAR Hz | Main observation |
| --- | ---: | ---: | ---: | ---: | --- |
| lidar | 0.507 | 0.507 | 5.06 | 5.07 | Renderer selection changes little; Gazebo uses about 70 percent CPU. |
| lidar_flow | 0.468 | 0.482 | 4.68 | 4.82 | Flow reaches 13.80 / 14.57 Hz; Gazebo reaches about 103 / 108 percent CPU. |
| lidar_flow_d435 | 0.423 | 0.444 | 4.23 | 4.44 | D435 bridge is the largest cost: 97.81 / 95.13 percent CPU. |

Both adapters were actually selected:

| Requested adapter | Confirmed renderer |
| --- | --- |
| AMD | `D3D12 (AMD Radeon(TM) 610M)` |
| NVIDIA | `D3D12 (NVIDIA GeForce RTX 5060 Laptop GPU)` |

The NVIDIA adapter is modestly faster in the two profiles with extra sensors,
but neither adapter meets the required effective RTF of 1.0 under WSLg. The
performance limit is not the C++ Livox conversion; it is Gazebo simulation plus
D435 image bridging and the WSLg graphics/runtime stack.

## Online Full-Stack Check

The NVIDIA full profile completed a real GUIDED rectangle flight. Its observed
rates were LiDAR 6.07 Hz, FAST-LIO 6.07 Hz, unified odometry 6.08 Hz, raw flow
14.80 Hz, D435 color/depth 9.31/9.16 Hz, and ExternalNav 20.00 Hz. It also
maintained valid `/mavros/odometry/out` output while the scheduler controlled
factor weight and covariance internally. Its effective RTF was 0.606, therefore
the run is a functional interface test but is not valid for online accuracy
comparison or RTF=1 acceptance.

The AMD full profile started the frontend, backend, and ExternalNav, but after
an arm request ArduCopter lost its MAVROS link and terminated with
`Floating point exception - aborting`. The exact firmware root cause could not
be resolved inside WSL because the generated crash process could not be
attached by the debugger. This profile is marked failed and must not be used
for a performance comparison.

## Rejected Experiments

* Raising the physics maximum step to 2 ms produced RTF 1.0 in a LiDAR-only
  probe, but ArduPilot failed its gyro/main-loop timing checks and later hit the
  same FPE. It is not a valid flight setting.
* Reducing MID360 horizontal rays from 500 to 250 halved the point count but did
  not materially improve RTF, so the model stays at 500 by 40 and 10 Hz.
* Reducing LiDAR to 5 Hz did not establish a stable ExternalNav full stack, so
  the normal model rate remains 10 Hz.

## Decision and Next Gate

The direct C++ adapter is accepted as the normal simulation interface; the
legacy Python PointCloud2 adapter remains an explicit compatibility mode only.
The ExternalNav gate now checks state, frame, timestamp, covariance, and
Scheduler health without dropping a valid fused pose just because one sensor
capability is temporarily low. The scheduler remains responsible for
factor-level weights, disable decisions, and covariance inflation.

Do not tune estimator accuracy from the current live Gazebo runs. First repeat
the NVIDIA full test on native Ubuntu with the NVIDIA driver or a verified GPU
pass-through configuration until the effective RTF reaches 1.0. Then run the
long-route regression and rosbag2 ATE/RPE evaluation. Hardware testing must
launch only the official Livox driver, not `mid360_sim_bridge_cpp`.
