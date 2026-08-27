# DUAL-DEGRADE-STABILITY-025

Date: 2026-08-27

Baseline commit: `50038b52c6696400350a056ac256a0a653862bc3`

## Frozen input

The post-FLOW-CONTRACT-021 capture is:

`/home/ld666/projects/dual-degrade-stability-025/frozen_bag_v4`

- World: `large_indoor_tunnel_apm_rgbd_mid360.sdf`
- Route: 2 m by 30 m rectangle, 2.2 m altitude, 0.8 m/s, face edges
- Visual factor: `rgbd_direct`
- Dynamic clean gateway: enabled
- DB3 SHA256: `a354faaf2eb6c9f92734e9c8691556b4ca1ce857e23cfef476b4955b85bd8ce0`
- Metadata SHA256: `3354baed4511f68dc74c0f2b9b36fafa5eca0078acafff0eec1067cfcc8f2af`
- Truth: 1,411 model-pose messages; 30.098 m horizontal extent; monotonic timestamps
- Inputs: 1,356 native LiDAR factors, 28,211 IMU, 679 GNSS fixes,
  2,134 Flow, and 383 Direct batches

The first v3 capture was not scoreable: the C++ MID360 bridge retained the
default `simple_apm_rgbd_mid360` pose topic while Gazebo ran
`large_indoor_tunnel`. Every truth message therefore used the scan
`world_pose` fallback, which was constant zero. The launcher now passes the
selected world and model explicitly. The recorder also now includes
`/vision/rgbd_direct_tracks`.

## Replay contract

All four baseline replays used the same v4 bag and configuration:

- replay rate 1.2, one numeric thread, two executor threads
- 2,048 native-factor QoS and worker queues; latest-only disabled
- 30 s post-replay drain
- `rgbd_direct`, current HXY projector, unchanged 0.15 / 0.25 / 0.001 values
- no IMU disable, no integrity relaxation, no Flow/Visual threshold change

`LiDAR degraded` and `GNSS degraded` are scheduler-mask ablations. The LiDAR
mask removes the complete LiDAR factor and is therefore stricter than real
tunnel translation degeneracy, where rotational and strong-direction
information remains.

## Baseline results

| Condition | XY RMSE | 3D RMSE | Transactions | Rollback | Flow factors | Direct factors |
|---|---:|---:|---:|---:|---:|---:|
| normal | 0.602 m | 0.615 m | 1,257 / 1,259 | 2 late ill-conditioned | 968 / 1,258 | 337 / 342 |
| LiDAR degraded | incomplete | incomplete | 342 / 404 | 62, persistent | 217 / 403 | 20 / 22 |
| GNSS degraded | 5.327 m | 5.612 m | 1,356 / 1,356 | 0 | 1,031 / 1,349 | 373 / 378 |
| LiDAR + GNSS degraded | 4.032 m | 4.452 m | 1,355 / 1,355 | 0 | 1,030 / 1,348 | 373 / 378 |

The LiDAR-degraded accuracy output stopped after 35 s, so its partial RMSE is
not reported as a full-route metric. Its first irreversible rejection was
transaction 342 at 44.7 s. The proposed translation correction was
`[-0.384, -1.279, -1.577] m`; repeated rejection then invalidated the scan
prediction contract and suppressed unified output.

The dual-degraded baseline completed the route without rollback or self-lock,
but failed the required accuracy bound by a wide margin. Flow and Direct being
online is therefore sufficient to remove the old 62 s rollback mode in this
run, but not sufficient for sub-meter full-route accuracy.

## The 60-80 s interval

For the dual-degraded baseline:

- 198 / 198 transactions committed
- Flow was added in 178 transactions and active in 196
- Direct was added in 76 transactions and active in all 198
- IMU was added and active in all 198
- no LiDAR or GNSS factor was active
- XY RMSE was 4.007 m and 3D RMSE was 4.230 m

The old failure interval is no longer an input-adoption outage. It is a
weak-direction history/propagation consistency problem.

## Flow input verification

The frozen bag was scored offline with the existing
`flow_gazebo_accuracy` equations against the repaired Gazebo truth:

- 1,099 aligned moving samples
- expected axis mapping
- scale 0.995
- correlation 0.995
- displacement RMSE 0.0037 m
- 60-80 s scale 1.005 and correlation 0.998
- median quality 163 and median distance 1.85 m

Ground distance, quality, gyro compensation, sign, and metric scale are not
the cause of the remaining drift.

## Prior causal ablation

One diagnostic replay excluded historical visual factors from the marginal
prior while retaining current-window Direct, Flow, GNSS, and IMU. In the
LiDAR-degraded condition it completed 1,356 / 1,356 transactions with zero
rollback and produced 0.365 m XY RMSE and 0.381 m 3D RMSE. This proves that
historical Direct linearization can trigger the lock when GNSS later corrects
the propagated state.

That result was not robust enough to promote. A narrower Direct-only prior
prototype and repeated ordering produced later rollback, including a
dual-degraded first rejection at 67.5 s. That correction was predominantly Z
(`+0.041, +0.208, -2.041 m`), while the prior still contained 409 historical
LiDAR and 560 IMU factors. The prototype was removed because fixing that event
would require changing Z behavior or a broader prior-consistency design.

## Conclusion

This milestone does **not** meet the freeze criteria:

- full dual-degraded progression: PASS on the unmodified baseline
- no persistent rollback/self-lock: PASS on the unmodified dual replay
- XY RMSE below 1 m: FAIL (4.032 m)
- preferred XY RMSE below 0.5 m: FAIL

Do not tune Flow/Visual admission, HXY thresholds, IMU noise, or the integrity
gate. The next experiment should build a deterministic regression around the
first prior/propagation divergence and preserve current-window Direct while
making historical relative visual and weak LiDAR information consistent with
IMU propagation. It must separately protect position and attitude history;
simply deleting Direct from the prior loses accumulated attitude support and
is sensitive to callback ordering.

The localization mainline is not frozen, and switching to offline map
maintenance plus loop closure is premature.
