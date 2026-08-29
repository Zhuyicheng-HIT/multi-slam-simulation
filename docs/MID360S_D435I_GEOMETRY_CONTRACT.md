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
Z up. The authoritative LiDAR frame is `livox_frame`. The camera optical frame
must match the runtime `CameraInfo.header.frame_id`; the calibrated numeric
transform remains valid when that runtime name is substituted explicitly.

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

The translation `t_body_lidar` is unmeasured. It must remain absent rather
than being represented by a zero vector. Rotation-only consumers such as IMU
vector normalization may operate; translation-dependent consumers such as
the hardware body mask and body-to-LiDAR TF must remain disabled/fail-open.

The Livox acceleration is in g and the gyro is in rad/s. A stationary,
level airframe should observe approximately `[-sin(15 deg), 0, cos(15 deg)] g`
in the mounted sensor frame and `[0, 0, 9.80665] m/s^2` after conversion and
rotation into `base_link`.

## Camera-to-LiDAR calibration

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

When all body transforms are measured, closure is checked as:

```text
T_body_camera * T_camera_lidar == T_body_lidar
```

Missing body translation produces an `INCOMPLETE` result, never an invented
transform. The calibrated camera-to-LiDAR transform is a closure constraint;
it must not be published as a redundant TF edge when both sensors already
have body-frame parents.

## Self-body envelope

Simulation retains its established legacy AABB behavior. Real hardware uses
an opt-in union of parameterized boxes and cylinders. Every primitive has a
name, body-frame center, orientation, dimensions and non-negative padding.
The real composite filter stays disabled until `t_body_lidar` and CAD-derived
primitives are complete.

On missing/invalid geometry or a runtime filtering exception, the filtered
copy fails open by forwarding the original scan and publishing degraded
diagnostics. Retained Livox points preserve coordinates, `offset_time`,
`reflectivity`, `tag`, and `line` exactly.

## Required measurements

The minimum hardware input is:

1. CAD definition of the `base_link` origin and FLU axes.
2. LiDAR measurement-origin translation `t_body_lidar` in metres.
3. Central body oriented box: center, orientation, length, width, height.
4. Each arm: endpoints or center/direction/length, plus radius.
5. Coaxial motor/hub stacks: center, axis, radius, height.
6. Landing-gear struts and skids: endpoints/centers, axes and dimensions.
7. MID360S, D435i and other persistent visible mounts/housings: oriented-box
   or cylinder dimensions.
8. Per-component manufacturing/flex padding.

The 50 x 50 x 30 cm overall airframe envelope is a sanity bound only and must
not be used as a single removal box.
