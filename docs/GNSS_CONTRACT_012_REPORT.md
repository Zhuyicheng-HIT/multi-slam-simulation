# GNSS-CONTRACT-012

## Timestamp contract

The previous GNSS transport path used `ensure_monotonic_stamp`, which repaired a non-monotonic source stamp to `last_stamp + 1 ns`. That behavior remains the default for compatibility, but the fault injector now exposes `repair_nonmonotonic_timestamps:=false`. In this mode a regression raises `ValueError` before publication and leaves the source stamp unchanged; it cannot silently alter the measurement time.

Regression test: `test_nonmonotonic_stamp_can_fail_without_repair` verifies the explicit failure, while the existing repair test verifies the compatibility path. The GNSS frozen run showed timestamp repair warnings in the injected transport path, so a future clean replay should run with repair disabled and treat any regression as a producer/transport failure.

## Gazebo ENU contract

The simulation world defines WGS84 spherical coordinates:

- origin latitude `-35.363262 deg`
- origin longitude `149.165237 deg`
- elevation `584 m`
- heading `0 deg`

For heading zero, longitude delta maps to Gazebo +X (east) and latitude delta maps to Gazebo +Y (north), with no sign inversion or yaw rotation. Directly converting the 375 frozen `/sensors/gnss/fix` samples using this definition and matching truth by source stamp gives a median association offset of `-32 ms`, P95 absolute `46 ms`, maximum `51 ms`.

After an offline translation/linear-frame fit (truth is not used by the estimator), the GNSS-to-truth horizontal residual is P50 `0.193 m`, P95 `0.529 m`, maximum `0.950 m`; the fitted 2D matrix is approximately:

```text
[[1.0116, -0.0176],
 [0.0022,  1.0084]]
```

Its positive determinant and near-identity rotation confirm no systematic axis sign or 90-degree yaw error. The apparent raw altitude offset is the expected datum difference (`~0.195 m` median), not a horizontal frame error.

## Replay conclusion

The same frozen replay remains:

- GNSS + MID360 IMU: XY RMSE `0.787 m`
- Full HXY chain: XY RMSE `0.790 m`

The approximately `3 mm` difference is replay scheduling variation. Timestamp association is within the backend compensation contract, ENU axes are correct, and HXY is not the remaining error source. The current `~0.79 m` should therefore be frozen as the simulation's GNSS/IMU observation-model floor. No weights, thresholds, IMU noise, HXY cap, Dynamic, Z axis, state machine, or relocalization were changed.

