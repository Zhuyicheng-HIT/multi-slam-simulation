# HXY-INTERACTION-006：事务 331 之前的水平漂移

## 范围与 replay contract

冻结输入为 `/home/ld666/projects/hxy-diag-002/frozen_bag`（metadata SHA256 `d842a6d3e19159123644efb8e9ac0d80e46ac2b02360a1ed44b812e84222372b`，db3.zstd SHA256 `1fdf2c5616670dc9fca7d6ba830ac5b2a4eec07d9cd54deb6f3d6aaf965ebd56`）。运行使用 commit `f82a0d96ae5e1d33b74f0335d10c3fecc7f272c3`、单 numeric thread、queue/QoS depth 1024、`latest-only=false`、rate 0.4、threshold 0.15/0.25 和 weak scale 0.001。Truth 仅用于 offline。

Trace 记录每个 active LiDAR factor 已接纳的 GNSS solver information、按 rotation 条件缓存的 Schur translation information、weak-direction projection 及其比值，不增加或删除 factor，也不改变 normal equation。

## 隔离结果

| replay | XY RMSE | 3D RMSE | endpoint norm | 观察 |
|---|---:|---:|---:|---|
| full, original | 1.538 m | 1.538 m | 6.99 m | rollback chain 后停止 |
| LiDAR + IMU | 55.376 m | 55.571 m | 141.45 m | 无 GNSS factor |
| GNSS + IMU | 0.787 m | 0.788 m | 2.09 m | absolute constraint 提供稳定性 |
| full, rollback off | 1.158 m | 1.158 m | 2.04 m | 首次 drift 仍存在 |
| full, diagnostic | 0.799 m | 0.799 m | 2.05 m | 无 queue loss |
| weak-mode cap | 0.790 m | 0.790 m | 2.05 m | residual 仍存在 |

具体数值会因 ROS executor interleaving 略有变化，但 full 与 rollback-off 运行的前 300 个 transaction 稳定一致。

## 首次因果分叉

首次持续的 5 cm 水平误差出现在 transaction 209（`t=51.513 s`）；首次持续的 0.2 m 误差出现在 transaction 283（`t=58.905 s`）；首次持续的 1 m 误差出现在 transaction 300（`t=60.621 s`）。只要有 sample，GNSS 就会被接纳；在 drift 发生前没有 GNSS NIS、integrity、stale、jump 或 scheduler rejection。

LiDAR weak direction 几乎沿 world Y（`|direction_y| > 0.999`）。修复前，active-window LiDAR weak information 是 GNSS weak information 的中位 5.66 倍（P95 9.58；transaction 200 为 65.73 对 10.45）。因此有效 GNSS correction 虽被接纳，却持续被累积的 weak-direction LiDAR factor 以更大 information 覆盖。LiDAR+IMU 重现同样增长，而 GNSS+IMU 可抑制它。

Transaction 331（`t=63.723 s`，scan 461）发生更晚，是首次 prediction-gate rejection，并非根因。首次原始 rollback 是 transaction 340（`t=64.713 s`），原因为 excessive translation correction。关闭 rollback 仍保留 tx209/283/300 的 drift，只阻止后续放大或截断。

## 最小修复与回归

修复在现有 active weak-subspace episode 内增加单侧 information cap，使用该 transaction 实际接纳的 GNSS information 和缓存的 solver Schur block。Weak-mode LiDAR information 被限制为 1:1 比例；GNSS weight/covariance、factor count、strong mode、Z、state machine、relocalization 和 marginal-prior mathematics 均未改变。该 cap 无法撤销已经 marginalize 的 information。

回归套件有 174 项通过测试，包括 rotated weak-mode 测试，证明 strong-mode scale 始终恰为 1。Prototype 将 active-window weak-mode ratio 降至约 0.10 中位数，但 whole-run RMSE 与 GNSS+IMU 在统计上相同。首个机制已确认；剩余误差超出 current-window-only intervention，主要来自 prior history 和 GNSS/IMU replay residual。

## 决策

根因已确认：首次 drift 时有效 GNSS 没有被拒绝或 rollback，而是在 weak LiDAR subspace 中持续被更大 information 覆盖。Rollback 是 transaction 331 之后的次级放大器。保留 cap 作为 diagnostic repair，但不要将其提升为完整的精度解决方案。下一步应在修改 marginalization mathematics 前，隔离已 marginalize 的 history 以及 absolute-factor timing。
