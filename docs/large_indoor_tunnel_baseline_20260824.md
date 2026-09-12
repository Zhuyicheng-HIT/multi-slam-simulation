# 长室内隧道基线 2026-08-24

长隧道场景现已纳入主 simulation package：
`src/multi_slam_uav_sim/worlds/large_indoor_tunnel_apm_rgbd_mid360.sdf`。
这是一个 96 m 的重复隧道，包含周期肋条、地面条带、彩色标志物、带纹理的人物、
RGB-D、光流和 MID360 传感器。

## 当前运行

当前稳定 backend 在关闭 ExternalNav FCU consumption、使用 `fcu_local` route
feedback 的配置下运行，并在 96 m 隧道内执行短的 8 m 纵向矩形。这样既保留隧道
几何与纵向退化，又使实验可测量。

任务完成起飞、四个路线段、着陆和 disarm。Estimator 结果未通过严格精度门限：

| 指标 | 结果 |
| --- | ---: |
| 3D RMSE | 2.721 m |
| 3D P95 | 5.029 m |
| 3D 最大值 / 终点 | 15.117 m |
| XY RMSE | 2.719 m |
| Z RMSE | 0.095 m |

主要误差是水平 Y 漂移。FAST-LIO diagnostic odometry 比 unified output 更差，
3D RMSE 为 22.153 m。运行还记录到 0.501 s 的最大 unified-odometry gap 和两个
stale stamps。仿真时间推进 132.849 s，wall time 为 397.235 s，有效 RTF 为 0.334。

## 基线上传门限

包含此场景的同一 worktree 在上传前也于普通室内场景运行。任务完成且严格验证通过：

| 指标 | 结果 |
| --- | ---: |
| 3D RMSE | 0.0258 m |
| 3D P95 | 0.0346 m |
| 3D 最大值 | 0.0419 m |
| XY RMSE | 0.0141 m |
| Z RMSE | 0.0216 m |
| 终点误差 | 0.0226 m |

全部五个 source-factor gate 均通过。Unified output 以 10.00 Hz 产生 828 个样本，
最大 gap 为 0.133 s，且没有 stale、duplicate 或回退的 source stamps。

## 方向性证据

稳定运行未执行 directional handoff：

- `axis_information_handoff_enabled=false`；
- LiDAR axis information scale 保持 `1,1,1`；
- axis handoff frames 和各轴 handoff counts 均为零；
- GNSS 使用 scalar reliability/information scale，而不是 XYZ handoff；
- LiDAR prediction-gate recovery 在 322 次 prediction rejection 后产生 320 个 recovery factors，
  这是 all-factor recovery path，不是 per-axis handoff。

尽管如此，LiDAR diagnostic 导出了方向性证据：

```text
profile information XYZ = 12120, 22849, 43942
raw information XYZ     = 60399, 26752, 151849
observability degradation XYZ = 0.848, 0.713, 0.448
condition number = 66.4
normalized Hessian eigenvalues = 0.015, 0.025, 0.060, 0.094, 0.348, 1.0
```

这些数值可作 diagnostic，但当前 backend 不会将其转换为按 source、按 axis 的
information allocation。GNSS 接收 654 次并形成 289 个 factors，scalar effective
information scale 为 0.885，scalar reliability weight 为 0.941。

## 研究方向

下一步应执行受控的 directional-information A/B，而不是新增 global weight。对每个
sensor 和每次 sliding-window update：

1. 将每个 factor Jacobian 投影到公共 body/world XYZ 子空间；
2. 在短的因果历史上累积 per-axis information matrix；
3. 将各 sensor 的 normalized axis information 与该轴的 predicted covariance 和 innovation 比较；
4. 应用带 hysteresis 与 conservation bound 的连续 diagonal information transfer，使健康 GNSS、
   RGB-D 或 optical flow 接管较弱的 LiDAR 轴，同时不改变较强轴；
5. 保留 cross-axis terms，并在弱方向相对 world X/Y/Z 发生旋转时使用 eigenvectors。

首个 acceptance test 应注入一个旋转的弱方向，再注入两个同时的弱方向，并验证 per-axis factor
information、causal error，以及未来 truth 不会带来改进。成功结果必须显示强轴不变、仅弱轴转移、
GNSS innovation 有界且没有 factor duplication。只有完成这些验证后，才应在默认基线中启用 handoff。
