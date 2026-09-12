# HXY-SANITY-005：backend 与 native LiDAR 正确性审计

日期：2026-08-25

## 结果

复现了一个当前版本的正确性缺陷：prediction recovery floor 可能重新接纳已经未通过 FAST-LIO/backend frame-consistency gate 的 LiDAR factor。局部 point-plane rank 完整并不能证明全局 map 对齐。一个带 2 m map jump 的 synthetic full-rank factor 在配置的 recovery weight 和 inflation 下仍使正确锚定的 backend state 移动 1.923 m。

最小修复保持所有未通过 prediction gate 的 factor 禁用。每个新 scan 仍独立评估；innovation 回到 gate 内的 scan 会立即接纳并清除 consecutive-rejection count。这样消除了坏 factor reinjection，不产生 latch，也不改变 threshold、Z policy、fusion state machine、relocalization 或 marginalization。

## 审计状态

| 项目 | 状态 | 证据 |
|---|---|---|
| Backend/FAST-LIO 首帧、origin、reset epoch | PASS | 冻结 bag 首个消费 factor 是 30.723 s 的 scan 131（`camera_init -> body`，reset counter 0）。Backend 直接从 native linearization pose 初始化 pose/origin，velocity 为零，并在同一 15D state 放置首个 prior；合法 residual reduction 后首 pose 仅差 1.5 mm。 |
| Native LiDAR 几何与 normal convention | PASS | raw geometry 重现 exporter residual，误差 `6.62e-12`；`J^T J` 误差 `8.69e-10`；`J^T r` 误差 `2.75e-9`。新的 right-local finite-difference test 误差小于 `2e-8`，孤立 factor 将 residual norm 从 2.425 降至 `3.53e-14`。 |
| Prediction gate 与 recovery | FAIL，已修复 | 修复前 globally shifted factor 在第 3 次 rejection 被接纳并使 state 移动 1.923 m；回归测试在旧行为失败、修复后通过。健康下一 scan 的 recovery test 证明不存在永久 latch。 |
| IMU stationary initialization 与 propagation | PASS | 倾斜静止初始化恢复 accel/gyro bias 至 `1.10e-14`/`1.59e-17`；一秒 propagation 的 position drift `6.34e-15` m、velocity drift `1.32e-14` m/s、residual norm `1.78e-15`。Analytic manifold IMU Jacobian 也通过 finite-difference test。 |
| 首次异常 transaction diagnostics | PASS | 记录 transaction/scan/time、innovation、gate reason、recovery flag、effective weight、solver admission、state commit、optimized state、eigenbasis、active factor 与 marginalization；可在不将 truth 送入 estimator 的情况下定位 transaction 331 首次 reject、transaction 333 首次不安全 recovery。 |

## 首帧与坐标 contract

Native packet 是 FAST-LIO `camera_init` map 中的 absolute unary point-plane factor，而不是 relative odometry factor；state 为 body/IMU pose。LiDAR 点按 `p_map = R_map_body (R_body_lidar p_lidar + t_body_lidar) + t_map_body` 变换。冻结 bag 的 `t_body_lidar = [0.05, 0, 0.10]`，LiDAR-to-body pitch 为 +15 度；从 raw point、extrinsic、map normal 与 map plane point 重算可在数值精度上重现 residual 与 normal equation。

Backend native-factor trigger 忽略独立 FAST-LIO odometry topic。首 state 使用 native factor 的 absolute linearization pose，将 `lio_origin` 设为同一位置，velocity 初始化为零，在可观测时设置 stationary IMU bias，并在精确 15D state 放置首个 prior。Bag 首个有效 packet 为 scan 122，但需等待可观测 IMU interval，因此首个消费 transaction 是 scan 131；这不会创建新的坐标 origin。

支持的 restart contract：协调的 process restart 清空两侧 sequence history；backend relocalization 增加 reset epoch、刷新旧 buffer、丢弃旧 epoch 在途 factor，仅接纳匹配的未来 epoch；legacy 独立 FAST-LIO factor 通过 `map_from_lio` 变换。独立 FAST-LIO process-only hot restart 不属于连续运行支持范围，其 sequence rollback 会被拒绝而非静默接纳。

## Native factor convention

FAST-LIO 导出的列顺序为 map translation、body right-rotation、LiDAR-to-body right-rotation、LiDAR-to-body translation。Backend 验证前六个 label 并固定 extrinsic，15D 顺序为 `position[0:3], right-SO(3)[3:6], velocity[6:9], accel_bias[9:12], gyro_bias[12:15]`。Raw correspondence factor 仅填充 0--5 列。导出的 `H = J^T J` 和 `g = J^T r` 是未加权 geometric normal，measurement variance、robust weight、scheduler weight 与 inflation 由 backend 且仅应用一次。

## Prediction recovery 缺陷

Gate 文档与实现不一致：gate 正确地将 excessive innovation 判为 map/frame consistency fault，但连续三次 reject 后，`recovery_geometry_usable` 覆盖了该 fault；该 flag 只检查 correspondence validity 和局部 6D rank。HXY-KERNEL-004 C 中 transaction 331（scan 461，`t=63.723 s`）首次 hard reject，innovation 1.026 m；随后 transaction 333 起以 effective weight `0.2 / 5 = 0.04` 注入 recovery factor，即使 innovation 从 1.148 m 增至超过 1.6 m，该运行共有 107 个此类 factor。

修复后同一 replay 接收 678 个 native packet，无 queue overflow、supersession 或 latest-only skip，recovery factor 为零。Scan 461 仍是首次 reject，之后不再接纳全局不一致 factor。共同可评分至 65.69 s 的 horizon 上，XY RMSE 从修复前 1.584 m 变为 1.408 m；该比较仅作诊断，因为 callback backlog 限制了修复运行的发布 horizon。

该修复不能解释初始水平 drift：首次 gate reject 发生在 drift 已建立之后；它消除了 transaction 331 后放大故障的真实次级正反馈路径。

## IMU 审计与决定

FCU accelerometer convention 是 body specific force。静止时测量 `R_body_map [0, 0, +9.81] + accel_bias`，propagation 只应用一次 map gravity `[0, 0, -9.81]`。启动 bias 仅在 sample 数/span 足够、平均角速度和 gyro variation 低、平均 force 符合 gravity 且 force variation 低时接受。冻结 replay 接受 0.881 s 内 89 个 sample，估计 accel `[0.00911, 0.00154, 0.00079]` m/s2、gyro `[0.000447, -0.000437, -0.000422]` rad/s 的小 bias，符合 synthetic stationary startup，不表示 origin 或 gravity-sign fault。

坐标、factor、perturbation、state-order 与 IMU primitive 均正确。Unsafe prediction recovery override 是 post-gate error amplification 的真实根因，已修复并加入回归测试，但不是早期 weak-direction drift 的根因。可从修复 commit 继续 HXY-INTERACTION，分析 pre-gate current-window LiDAR/GNSS interaction 和 integrity rollback；不得仅凭局部可观测性恢复被拒绝的 LiDAR factor。未运行 Gazebo，也未 push。
