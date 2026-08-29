# MID360S and D435i hardware geometry contract

This document records the approved hardware geometry contract for the real
MID360S and D435i installation. It deliberately does not alter FAST-LIO's
internal LiDAR-to-IMU calibration.

## Coordinate and transform convention

All transforms use `T_parent_child` and map a point expressed in the child
frame into the parent frame:

```text
p_parent = R_parent_child * p_child + t_parent_child
```

The aircraft body frame is `base_link` in FLU convention: X forward, Y left,
Z up. For this engineering release, the `base_link` origin is **defined** at
the MID360S LiDAR measurement center while retaining the aircraft body-FLU
axes. Consequently `t_body_lidar = [0, 0, 0] m` is a coordinate-system
definition, not a measured mechanical translation. The authoritative LiDAR
frame is `livox_frame`. The camera optical frame must match the runtime
`CameraInfo.header.frame_id`; the calibrated numeric transform remains valid
when that runtime name is substituted explicitly.

## Topic ownership

- `/livox/lidar` is the immutable Livox CustomMsg stream.
- `/livox/imu` is the immutable MID360S internal IMU stream.
- `/sensors/imu` is the body-FLU, SI-normalized backend IMU stream.
- The flight-controller IMU is a separate MAVROS/ArduPilot stream and is not
  the Ultra-Fusion backend IMU.
- No `/livox/lidar_imu` alias is part of the production contract.
- Body removal applies only to a type-preserving copy used by localization or
  mapping. It never mutates or republishes `/livox/lidar` in place.

## MID360S installation rotation

The complete MID360S module is nominally pitched 15 degrees nose-down. With
the transform convention above:

```text
R_body_lidar = R_y(+15 deg)
             = [ 0.965925826289, 0,  0.258819045103,
                 0,              1,  0,
                -0.258819045103, 0,  0.965925826289 ]
q_body_lidar_xyzw = [0, 0.130526192220, 0, 0.991444861374]
```

The translation is exactly zero by the `base_link` origin definition above;
its provenance is `coordinate_definition`, never `measured`. The 15-degree
rotation applies to the complete MID360S module relative to the body-FLU axes.
It is not, and must not overwrite, FAST-LIO's internal LiDAR-to-IMU extrinsic.

The Livox acceleration is in g and the gyro is in rad/s. A stationary,
level airframe should observe approximately `[-sin(15 deg), 0, cos(15 deg)] g`
in the mounted sensor frame and `[0, 0, 9.80665] m/s^2` after conversion and
rotation into `base_link`.

## Camera-to-LiDAR calibration

The source is the four-page formal report
`MID360S_D435i_外参标定结果.docx` (SHA256
`5e6e1401f26eb369b8d7c3132b3a5c6c777f44d1f44d6bce93b938b1dd28160c`).
The authoritative calibration maps `livox_frame` into the D435i color optical
frame:

```text
t_camera_lidar_m = [0.063705566722, -0.170242628857, -0.008687984507]
q_camera_lidar_xyzw =
  [0.429882540535, -0.439190855086, 0.554046154514, 0.561556099442]
```

The calibration report records 3.03 cm overall RMSE and 5.93 cm maximum
single-point residual. It is suitable for engineering initialization, not a
sub-centimetre ground truth.

The body-to-camera transform is derived automatically, without a separately
measured `t_body_camera`:

```text
T_body_camera = T_body_lidar * inverse(T_camera_lidar)
T_body_camera * T_camera_lidar == T_body_lidar
```

The contract checker reports the resulting chain as `DERIVED` and verifies
near-zero SE(3) closure residual. The calibrated camera-to-LiDAR transform is
still a closure constraint; the generic simulation D435i mount publisher stays
off on hardware to avoid a redundant TF parent for RealSense optical frames.

## Self-body envelope

Simulation retains its established legacy AABB behavior. Real hardware uses
one configurable, provisional conservative box in body-FLU:

```text
x: [-0.28, 0.28] m
y: [-0.28, 0.28] m
z: [-0.30, 0.06] m
```

This is an engineering approximation based on the roughly 50 x 50 x 30 cm
airframe envelope plus modest padding. It intentionally does not model arms,
motors, propeller hubs, landing gear or sensor mounts as separate primitives.
It is enabled for the filtered localization/mapping copy, can be disabled with
one parameter (`filter_enabled=false`), and remains fail-open.

On missing/invalid geometry or a runtime filtering exception, the filtered
copy fails open by forwarding the original scan and publishing degraded
diagnostics. Retained Livox points preserve coordinates, `offset_time`,
`reflectivity`, `tag`, and `line` exactly.

## Future refinement policy

Complete CAD is not a feature-enablement blocker. Refine the provisional box
into component geometry only if real logs demonstrate either (a) removal of
nearby environment points, especially walls or ground, or (b) persistent
self-body returns outside the box. Any refinement must preserve immutable raw
`/livox/lidar`, the separate filtered-copy ownership, the one-switch bypass and
fail-open behavior.
