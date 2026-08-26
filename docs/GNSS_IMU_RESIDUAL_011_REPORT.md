# GNSS-IMU-RESIDUAL-011

## Scope

This audit uses the HXY-DIAG-002 frozen bag (`d842a6d3e19159123644efb8e9ac0d80e46ac2b02360a1ed44b812e84222372b` metadata; `1fdf2c5616670dc9fca7d6ba830ac5b2a4eec07d9cd54deb6f3d6aaf965ebd56` compressed database) and the current HXY + MID360 IMU + Dynamic V2 backend replay artifacts. Truth is used only offline.

## Replay comparison

| mode | XY RMSE | 3D RMSE | endpoint | interpretation |
|---|---:|---:|---:|---|
| GNSS + MID360 IMU | 0.787 m | 0.788 m | 2.09 m | absolute-factor error floor |
| HXY full chain with current weak-mode cap | 0.790 m | 0.790 m | 2.05 m | only 3 mm above GNSS+IMU |

The 3 mm difference is within replay executor/interleaving variation. HXY is no longer the dominant source after the active weak-subspace cap; changing its cap is not justified by this evidence.

## GNSS association and absolute error

The bag contains 375 `/sensors/gnss/fix` samples and 752 truth samples. Nearest truth association gives GNSS header-to-truth timing offset median `-32 ms`, p95 absolute `46 ms`, maximum `51 ms`. Backend GNSS prefit traces report time-compensation age median `33 ms`, p95 `49 ms`, maximum `142 ms`; compensation is applied rather than silently ignored.

The source stream contains non-monotonic timestamp repairs in the formal static run (`fault_injector_gnss: repaired non-monotonic gnss timestamp`). This is a data/transport quality issue to close in the next replay, but it is not evidence of a fixed constant offset large enough to explain 0.79 m.

Direct lat/lon-to-truth comparison requires the simulation's ENU frame transform. After an offline affine frame fit, GNSS horizontal residual is p50 `0.193 m`, p95 `0.529 m`, maximum `0.950 m`; raw uncalibrated comparison is invalid because the bag's GNSS frame and truth frame have different axis/origin conventions. GNSS altitude has a roughly `0.195 m` datum offset, with p95 absolute residual `0.237 m`.

## GNSS innovation and admission

From the frozen replay backend trace and final summary:

- GNSS received `375`, consumed `260`, records `258`, factors `258`.
- Stale samples `22`; scheduler-disabled `2`.
- Hard GNSS NIS rejects `0`.
- XY NIS robust-downweighted samples `96`; XY NIS rejected counter `96`; Z NIS rejects `0`.
- Prefit XY NIS median `0.222`, but p95 `11295` and maximum `30947`.
- Time compensation delta median: X `2 mm`, Y `17 mm`, Z `1 mm`; p95 absolute values X `39 mm`, Y `93 mm`, Z `25 mm`; maximum Y `4.42 m`.

Thus valid GNSS is entering the solver, but a substantial tail of innovations is being robustly weakened. This explains why GNSS+IMU stabilizes the trajectory at approximately `0.787 m` without fully removing the error.

## MID360 IMU propagation

The replay consumed 7,513 IMU samples and formed 497 IMU factors with zero invalid samples, zero pair timeouts, and zero non-monotonic arrivals. Startup initialization passed (`89` samples over `0.881 s`, bias accepted). It performed 16 reintegrations, with 22 deferred, and used the IMU-propagated covariance anchor. There is no evidence of an IMU transport failure or permanent propagation loss. Remaining IMU contribution is therefore ordinary bias/noise/extrinsic propagation error, coupled to the GNSS correction cadence and robust gating.

## Conclusion

The approximately `0.79 m` residual is already close to the current GNSS + MID360 IMU observation floor. The HXY full chain is only about `3 mm` worse than GNSS+IMU, and HXY-PRIOR-007 found historical weak LiDAR suppression changes RMSE by only about `6 mm`. Do not tune GNSS weight, IMU noise, HXY cap, or thresholds from this replay.

No estimator fix is made in this task. The next useful experiment is a clean GNSS timestamp/association replay with timestamp repair disabled as an explicit failure mode, plus a GNSS-frame calibration check against the simulator's actual ENU transform. If that replay preserves the same residual, the remaining 0.79 m should be treated as the current sensor/model floor rather than an HXY defect.

