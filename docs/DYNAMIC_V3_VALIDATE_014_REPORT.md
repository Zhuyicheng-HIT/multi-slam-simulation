# DYNAMIC-V3-VALIDATE-014

## HXY frozen replay

当前 V3 Dynamic branch 在解压的 HXY-DIAG-002 frozen bag 上 replay，使用现有 C++ HXY kernel 与 native-factor contract。处理全部 `678` native factor、提交 `672` 个 state，native queue overflow/discard、IMU pair timeout、worker error、optimization error 均为 0。Causal XY RMSE `0.79306 m`，与约 `0.790 m` 基准相比不是有意义的回归，接近 GNSS+MID360 IMU floor `0.787 m`。

## Static replay 状态

两次 V3 static attempt：完整 Dynamic + visual run 到达 FAST-LIO/backend 后因等待 `/vision/feature_tracks` 超时；关闭 visual frontend/RTAB 的第二次到达 estimator chain，但未满足 PR6 visual readiness contract 即手动终止。此处不宣称完整 60 s V3 drift；仍需用 V3 defaults 完成 clean 60 s run 才能 promotion。

## Feature repeatability zero frame

唯一 zero repeatability sample 位于 LiDAR stamp `17.2 s`（recording elapsed `1.36 s`、wall arrival `29.2 s`），含 `2828` input points、`400` matched points，但 `uncertain_ratio=1.0`、`map_quality=0.0`、`dynamic_ratio=0.0`。这是 startup/map-warmup uncertainty，不是 Dynamic deletion 或 false positive。Static evidence 仍为 median/P5 `100%`、minimum `0%`、`<95%` ratio `1/60 = 1.67%`。

## 困难场景

- `far_sparse_target`：P/R/F1 `0/0/0`，dynamic-unknown ratio `54.8%`；18 m 处目标稀疏且移动不足有效 voxel/neighborhood evidence，是 LiDAR geometry/visibility observability limit，broad deletion 会损害 static preservation。
- `occlusion_appear_disappear`：recall `59.8%`、precision `100%`、contamination `40.2%`；重新出现通常被标记 unknown，observer 对遮挡保持保守。
- `small_fast_target`：recall `68.4%`、precision `99.8%`、dynamic-unknown ratio `30.7%`；目标快速跨 voxel，temporal occupancy history 不足，但 precision 很高。
- `opening_closing_door`：recall `67.6%`、precision `97.9%`、contamination `32.3%`；铰接表面运动部分被 hinge/frame static structure 保护，仍有 dynamic return 留在 map。

## 决策

V3 尚不是 V2 的 drop-in replacement：macro recall 从 `85.77%` 提高到 `86.53%`，precision 与 static preservation 基本不变，但 far-sparse 仍不可观，完整 V3 60 s static replay 未完成。保持 V2 为 release baseline；先修复 visual readiness contract 并重跑 V3 static，再评估 scene-specific far-sparse observability metric，不扩大全局 deletion。
