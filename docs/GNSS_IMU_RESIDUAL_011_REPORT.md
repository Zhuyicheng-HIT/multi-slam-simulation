# GNSS-IMU-RESIDUAL-011

## 范围

本审计使用 HXY-DIAG-002 frozen bag（metadata `d842a6d3e19159123644efb8e9ac0d80e46ac2b02360a1ed44b812e84222372b`，compressed database `1fdf2c5616670dc9fca7d6ba830ac5b2a4eec07d9cd54deb6f3d6aaf965ebd56`）以及当前 HXY + MID360 IMU + Dynamic V2 backend replay artifacts。Truth 仅离线使用。

## Replay 对比

| mode | XY RMSE | 3D RMSE | endpoint | 解释 |
|---|---:|---:|---:|---|
| GNSS + MID360 IMU | 0.787 m | 0.788 m | 2.09 m | absolute-factor error floor |
| HXY full chain with current weak-mode cap | 0.790 m | 0.790 m | 2.05 m | 仅比 GNSS+IMU 高 3 mm |

3 mm 差异在 replay executor/interleaving variation 范围内。启用 weak-subspace cap 后 HXY 不再是主因，没有证据支持修改 cap。

## GNSS association 与绝对误差

Bag 含 375 个 `/sensors/gnss/fix` 与 752 个 truth sample。Nearest-truth association 得到 GNSS header-to-truth timing offset median `-32 ms`、p95 absolute `46 ms`、maximum `51 ms`。Backend GNSS prefit trace 报告 time-compensation age median `33 ms`、p95 `49 ms`、maximum `142 ms`，补偿已应用而非忽略。

Formal static run 的 source stream 含 non-monotonic timestamp repair（`fault_injector_gnss: repaired non-monotonic gnss timestamp`）。这是下一次 replay 要关闭的数据/transport 质量问题，但不足以解释 0.79 m 的固定大偏移。

Lat/lon 与 truth 直接比较必须使用 simulation ENU frame transform。Offline affine frame fit 后 GNSS horizontal residual 为 p50 `0.193 m`、p95 `0.529 m`、maximum `0.950 m`；raw uncalibrated comparison 因 axis/origin convention 不同而无效。GNSS altitude 有约 `0.195 m` datum offset，p95 absolute residual `0.237 m`。

## GNSS innovation 与 admission

冻结 replay backend trace/final summary：GNSS received `375`、consumed `260`、records `258`、factors `258`；stale `22`、scheduler-disabled `2`；hard GNSS NIS reject `0`；XY NIS robust-downweighted `96`、rejected counter `96`、Z NIS reject `0`；prefit XY NIS median `0.222`，p95 `11295`、maximum `30947`；time compensation delta median X/Y/Z 为 `2/17/1 mm`，p95 absolute `39/93/25 mm`，Y maximum `4.42 m`。

有效 GNSS 正进入 solver，但 innovation tail 中相当一部分被 robust weakening。这解释了 GNSS+IMU 将轨迹稳定在约 `0.787 m`，却未完全消除误差。

## MID360 IMU propagation

Replay 消费 7,513 个 IMU sample，形成 497 个 IMU factor；invalid sample、pair timeout、non-monotonic arrival 均为 0。Startup initialization 通过（`89` sample、`0.881 s`，bias accepted）；执行 16 次 reintegration、22 次 deferred，并使用 IMU-propagated covariance anchor。没有 IMU transport failure 或永久 propagation loss 证据，剩余影响是普通 bias/noise/extrinsic propagation error，与 GNSS correction cadence 和 robust gating 耦合。

## 结论

约 `0.79 m` residual 已接近当前 GNSS + MID360 IMU observation floor；HXY full chain 只差约 3 mm，HXY-PRIOR-007 也显示历史 weak LiDAR suppression 仅改变约 6 mm。不要依据本 replay 调整 GNSS weight、IMU noise、HXY cap 或 threshold。

下一步应做 clean GNSS timestamp/association replay（关闭 timestamp repair，并将其作为显式 failure mode），再依据 simulator 实际 ENU transform 做 GNSS-frame calibration check。若 residual 保持不变，应将 0.79 m 视为当前 sensor/model floor，而非 HXY defect。
