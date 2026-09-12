# GNSS-CONTRACT-012

## Timestamp 契约

旧 GNSS transport 使用 `ensure_monotonic_stamp`，将非单调 source stamp 修复为 `last_stamp + 1 ns`。该行为仍是 compatibility default；fault injector 现在提供 `repair_nonmonotonic_timestamps:=false`。此模式下 regression 会在 publication 前抛出 `ValueError`，保持 source stamp 原值，不能静默改变 measurement time。

Regression test `test_nonmonotonic_stamp_can_fail_without_repair` 验证显式失败，现有 repair test 验证兼容路径。GNSS frozen run 在 injected transport 中出现 timestamp repair warning；后续 clean replay 应关闭 repair，把任何 regression 视为 producer/transport failure。

## Gazebo ENU 契约

Simulation world 定义 WGS84 spherical coordinate：origin latitude `-35.363262 deg`、longitude `149.165237 deg`、elevation `584 m`、heading `0 deg`。Heading zero 时，longitude delta 映射 Gazebo +X（east），latitude delta 映射 +Y（north），无 sign inversion 或 yaw rotation。按此定义直接转换 375 个冻结 `/sensors/gnss/fix` sample，并按 source stamp 匹配 truth，median association offset `-32 ms`、P95 absolute `46 ms`、maximum `51 ms`。

Offline translation/linear-frame fit（truth 不进入 estimator）后，GNSS-to-truth horizontal residual 为 P50 `0.193 m`、P95 `0.529 m`、maximum `0.950 m`，拟合 2D matrix：

```text
[[1.0116, -0.0176],
 [0.0022,  1.0084]]
```

Positive determinant 与 near-identity rotation 证明没有 axis sign 或 90-degree yaw 系统错误。Raw altitude offset 约 `0.195 m` 是预期 datum difference，不是 horizontal frame error。

## Replay 结论

- GNSS + MID360 IMU：XY RMSE `0.787 m`
- Full HXY chain：XY RMSE `0.790 m`

约 `3 mm` 差异来自 replay scheduling variation。Timestamp association 满足 backend compensation contract，ENU axes 正确，HXY 不是剩余误差源。因此将当前约 `0.79 m` 冻结为 simulation 的 GNSS/IMU observation-model floor。未修改 weight、threshold、IMU noise、HXY cap、Dynamic、Z axis、state machine 或 relocalization。
