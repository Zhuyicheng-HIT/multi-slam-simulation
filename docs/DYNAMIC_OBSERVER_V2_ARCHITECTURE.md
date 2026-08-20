# Dynamic observer v2 architecture

## Runtime boundary

```text
/livox/lidar CustomMsg -----------------+------> FAST-LIO (unchanged)
                                       |
                                       +------> observer v2 side channel
previous FAST-LIO /Odometry ---------->|
/livox/imu ---------------------------->|
                                              -> static/dynamic/unknown clouds
                                              -> statistics/latency diagnostics
```

The node is opt-in and disabled by default. It owns no source topic, publishes
no TF, and cannot select delayed unified-pose deskew. The current scan is
propagated from a strictly non-future FAST-LIO state and raw IMU. This is causal
for a later pre-FAST-LIO gateway because the anchor comes from an earlier scan.

## Visibility state

The implementation is clean-room and combines FreeDOM/DUFOMap principles:

1. accept only measured MID360 returns and their real per-point offsets;
2. ray trace only cells traversed by actual rays; a missing angular sample never
   means free space and no spinning-LiDAR range-image inpainting is used;
3. keep FREE, confirmed STATIC, DYNAMIC candidate, and UNKNOWN evidence apart;
4. require repeated free observations before an occupied contradiction becomes
   a strong dynamic seed;
5. mark a confirmed static surface vacated only when a measured ray traverses
   it, never because it was temporarily occluded or absent;
6. grow labels locally and retain dynamic state for a bounded number of scans so
   slow or recently stopped targets do not immediately become permanent map;
7. protect a currently supported confirmed-static endpoint from neighboring
   vacated evidence;
8. hold far sparse returns UNKNOWN longer with configurable range/dwell values;
9. recover persistent occupancy conservatively to tolerate calibration or pose
   error instead of creating immortal dynamics.

All thresholds are configuration, not attributed to the papers. Truth data is
absent from the node and used only by the benchmark evaluator.

## Deskew contract

Livox `offset_time` is nanoseconds. This matches FAST-LIO's conversion to
milliseconds and then seconds. The causal deskew contract is:

- pose anchor timestamp <= scan begin;
- monotonically increasing IMU samples only;
- no IMU sample newer than the queried point set;
- maximum inter-sample and terminal gaps bounded by `deskew.max_imu_gap_s`;
- terminal zero-order hold allowed only inside that bound;
- prediction horizon bounded independently;
- failure rejects the observer scan and never affects FAST-LIO.

The DYN-INTEGRATION-005 candidate now exposes pose, velocity, calibrated IMU
biases, sequence, reset epoch, and timestamp from the completed Clean FAST-LIO
posterior. Scan i uses only a state strictly preceding it. The observer-mode
node remains available; the production raw path is unchanged.

## Integration-stage gate

DYN-INTEGRATION-005 added a default-off fail-open gateway and completed a frozen
Raw/Clean dual FAST-LIO replay. The synthetic gate is recorded in
`DYN_INTEGRATION_005_CLEAN_GATEWAY.md`. Captured team MID360 data and hardware
scheduling remain mandatory before any source-topic ownership change; the raw
path must remain immediately recoverable.
