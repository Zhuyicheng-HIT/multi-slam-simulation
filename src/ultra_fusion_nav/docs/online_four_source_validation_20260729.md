# Online Four-Source Validation (Normal Map, 2026-07-29)

## Scope

This milestone validates the online four-source path in the normal Gazebo map:

- FCU `HIGHRES_IMU` through `/mavros/imu/data_raw`;
- MID360 point cloud through patched FAST-LIO native point-to-plane factors;
- direct MTF-01 MAVLink optical flow and range on the companion path;
- GNSS through `/sensors/gnss/fix`.

The D435 point cloud bridge was disabled. The rectangle state machine remained
GPS GUIDED, so this is an estimator-observation validation rather than an
ExternalNav flight-control claim. Gazebo truth and MAVROS local pose were used
only by the drift evaluator, never as estimator inputs.

## Changes Validated

1. The backend worker keeps only the newest native LiDAR frame and runs NumPy
   / BLAS with one numerical worker. This avoids queue growth while preserving
   the latest scan.
2. `uf_lio_adapter` now starts with the unified stack and publishes native
   FAST-LIO residual and Hessian diagnostics to `/lio/diagnostics`.
3. The direct MTF bridge now honors `restamp_output`. In simulation it stamps
   decoded ROS observations in the MAVROS IMU time domain; the original
   MAVLink frame remains available on `/sim/mtf01/mavlink_frame` for offline
   temporal calibration. This fixes the unrelated Gazebo-sim and MAVROS clock
   epochs without changing raw MAVLink payloads.
4. The unified ExternalNav gate requires a fresh scheduler state of `NORMAL`
   or `RECOVERED`. Legacy GPS/flow use remains ungated by default.

## Runtime Evidence

Initial stationary native LiDAR packet:

| Metric | Result |
| --- | ---: |
| matched points | 1,442 |
| point-to-plane residual mean | 0.0150 m |
| Hessian condition number | 42.41 |
| feature repeatability | 0.987 |
| backend solve time | 16-24 ms typical |
| native worker overflow during first flight | 1 frame, discarded in favor of newest |
| backend optimization errors | 0 |

The first 146 s recording exposed the clock-domain defect: all 2,839 flow
observations were invalid because FCU yaw could not be interpolated at the
flow timestamp. After restamping, a 96 s recording of the same route produced:

| Flow / scheduler result | Value |
| --- | ---: |
| direct flow observations | 1,816 |
| valid reliability observations | 380 (20.9%) |
| scheduler states | 765 `DEGRADED`, 181 `RISK`, 9 `RECOVERED` |
| optical-flow factor enabled scheduler updates | 73 / 955 |
| optical-flow factor attempts | 530 |
| accepted optical-flow factors | 49 |
| factor rejections: quality / rotation | 80 / 199 |
| ExternalNav accepted / rejected | 7 / 5,305 |

Ground frames have no valid range and are intentionally rejected. During turns,
the FCU yaw-rate gate downweights or closes the flow factor; this is expected
for the current rotation-compensation policy. The seven ExternalNav packets
were forwarded only during `RECOVERED`; all `DEGRADED` and `RISK` periods were
blocked.

## Standard Drift Evaluation

The evaluator ran for 125.006 s during a 2.0 m x 1.2 m rectangle flight.

| Metric | Result |
| --- | ---: |
| FAST-LIO position RMSE | 0.0190 m |
| maximum position error | 0.0714 m |
| final position error | 0.0021 m |
| yaw RMSE | 0.0940 deg |
| maximum yaw error | 0.5316 deg |
| FAST-LIO yaw / FCU gyro correlation | 0.9152 |
| estimated FCU-IMU lag | 20 ms |
| raw cloud / registered cloud / IMU timestamp regressions | 0 / 0 / 0 |
| cloud voxel-overlap median | 0.768 |
| evaluator result | passed |

Artifacts are intentionally outside the Git tree:

- `/tmp/uf_normal_validation_20260729b/slam_report_restamped.json`
- `/tmp/uf_normal_validation_20260729b/flight_observations_restamped`
- `/tmp/uf_normal_validation_20260729b/rectangle_drift/guided_rectangle_waypoints.log`

## Boundary and Next Work

This proves that LiDAR, IMU, GNSS, and guarded optical-flow factors coexist in
the online manifold window. It does not yet prove that dynamic weighting
improves fused ATE/RPE over a fixed-weight ablation, nor that the resulting
ExternalNav estimate is ready to control a real aircraft. The next focused
iteration is an offline rosbag ablation with matched fault-free and
degradation-injected routes, followed by explicit online temporal/extrinsic
calibration states rather than further threshold tuning.
