# Ultra-Fusion Robustness V3 验证结果

## 范围与证据 contract

本报告冻结 Performance V2 estimator（`d04f88d422de611454cf9454ffa2cc3a5741dab3`），并从 estimator 外部评估 Robustness V3。未修改 fusion factor、物理 sensor model、integrity threshold、rollback rule、D_V/FRS threshold 或 `/Odometry` fallback。Profile 中的 level 是工程测试点，不是论文声称的极限。

Campaign 共 88 份确定性 replay report：38 个 single-fault、40 个 time/extrinsic-calibration、8 个 double-fault 和 2 个 endurance。所有 replay 使用相同 immutable sensor payload 和冻结 nominal backend trajectory 作为 alignment reference，因此以下 replay ATE/RPE 是 **delta-to-frozen-nominal**，不是 simulator ground-truth 绝对精度。88 次 replay 均为 zero optimization error、zero true integrity rejection、zero transaction rollback、zero `/Odometry` fallback。

## FRS ON/OFF 结果

在具有有效对齐轨迹的配对运行中，FRS ON 在 36 个 case 降低 delta-ATE，在 5 个 case 增加。中位收益 0.001188 m，说明 FRS 总体有保护作用但并非始终更优；LiDAR medium degradation 和 IMU medium bias 是反例，应保留在 regression set。

中位 fault-response time：vision 0.070 s、LiDAR 0.177 s、GNSS 0.009 s、optical flow 0.018 s、IMU 0.109 s。中位 measured recovery time：vision 0.047 s、LiDAR 0.496 s、flow 0.471 s、IMU 0.008 s。GNSS recovery 不能由本 replay 断言，因为冻结 capture 没有 live FCU GNSS innovation metadata，fresh scheduler 正确地将 GNSS 保持为 zero weight。注入的 GNSS jump A/B 验证 isolation，但 detection/recovery 需使用 live FCU innovation evidence 重复。

| Profile | FRS ON delta-ATE | FRS OFF delta-ATE | Completeness | 结论 |
|---|---:|---:|---:|---|
| nominal | 0.002278 m | 0.003623 m | 1.000 | reference |
| visual heavy，6 s dropout | 0.003669 m | 0.004603 m | 1.000 | continuous |
| LiDAR medium，60% correspondence dropout | 0.018342 m | 0.017146 m | 1.000 | continuous；ON 略低精度 |
| GNSS jump heavy，35 m | 0.002307 m | 0.012175 m | 1.000 | FRS 保护最强 |
| flow heavy，25 s outage | 0.002281 m | 0.003625 m | 1.000 | continuous |
| IMU medium，0.05 rad/s、0.30 m/s² bias | 0.005890 m | 0.003646 m | 1.000 | continuous；ON 较低精度 |
| LiDAR heavy，25 s outage | 约 0 m | 0.000068 m | 0.512 | 两种模式均 continuity failure |
| IMU heavy，25 s outage | 约 0 m | 0.000069 m | 0.416 | 两种模式均 continuity failure |

不完整 heavy-outage trajectory 的近零 ATE 不是成功：只有短暂存活 prefix 被对齐。Completeness 和最大 odometry gap 才是 governing criterion。

## 实测边界

以下仅是本 campaign 测试的 pass/fail boundary，并非完整物理极限：

- Vision 在测试的 6 s heavy outage 中保持 continuous。
- Native LiDAR 在 60% correspondence dropout 下保持 continuous；25 s outage 在 route completeness 0.512 时失败。
- GNSS denial 30 s 和 35 m position jump 保持 continuous，但受上述 frozen-innovation 限制。
- Optical-flow outage 25 s 保持 continuous。
- IMU bias 0.05 rad/s + 0.30 m/s² 保持 continuous；25 s outage 在 completeness 0.416 时失败。
- Camera-to-IMU offset 在 +100 ms、−50 ms 通过，这是测试的最大正/负值；实际极限位于未测试点之外或之间，不能推断。
- Native LiDAR timing 是最危险 fault；即使 +2 ms 也产生超过 1 s 的 trajectory gap，±5 ms 和 +20 ms 通常只剩一个可用 native factor。本冻结 replay 未证明存在非零容忍 operating interval。
- D435i extrinsic 在最大测试误差 8° rotation、15 cm translation 下通过。
- MID360 在 3° rotation 下通过；8° 时 completeness 0.77 失败。15 cm translation 通过宽松 continuity criterion，但 delta-ATE 增至 0.0856 m（5 cm 时 0.0285 m），不建议作为 calibration allowance。

## Double fault 与 endurance

Visual+GNSS medium、LiDAR+GNSS medium、IMU+flow medium 在两种 FRS mode 下均保持完整轨迹，optimization/integrity/rollback error 为零。Visual+LiDAR heavy 两种模式均在 completeness 0.512 失败。FRS ON 改善 visual+GNSS，而 LiDAR+GNSS 与 IMU+flow pair 在 FRS OFF 略高精度，因此结果应视为 measured boundary，不能泛化为鲁棒性保证。

Long cyclic visual/GNSS replay 保持完整。FRS ON：delta ATE 0.002825 m，RPE 0.003716 m / 0.015354°，solver median 26.888 ms，CPU 58%，peak RSS 91,516 KiB；FRS OFF：0.003350 m，0.004029 m / 0.019241°，32.211 ms，CPU 62%，93,480 KiB。两者均 zero optimization/integrity/rollback error。

## Online joint-map 压力结果

最终 full-stack run 验证了修正后的 launch contract：`paper_reprojection` active，mapper 消费 `/cloud_registered_filtered`，使用 NativeLidarFactor，未启用 pose fallback。车辆故障前接收 448 LiDAR、516 IMU、516 GNSS、91 optical-flow 和 9 paper visual factor。Source-aware map 含 83,973 voxel：59,303 LiDAR、34,419 RGB-D、9,749 joint-source、49,554 LiDAR-only。RGB coverage 0.164393；RGB-D 提供 24,670 supplementary voxel（0.415999 volume growth）。Geometry conflict、conflict ratio、ghosting proxy、eviction 全为零。

该 full-stack test 为 **FAIL**，不是 stability pass。第二个 turn 中 ArduPilot 报告 `Crash: AngErr=50>30, Accel=0.2<3.0`；车辆在未 LAND 的情况下安全 disarm。Backend 记录 27 个 non-committed transaction 和 27 次 rollback（22 次 excessive translation correction、5 次 excessive accelerometer-bias correction）。Simulation RTF 为 0.478。故障发生在第一段约 5.8 m 后，因此虽 partial map 内部一致，仍未证明长时间 map stability。所有 process 与 port 均已清理。

## 发布决定

Robustness V3 适合作为可复现的 fault-injection/regression candidate，但尚未达到直接 real-hardware flight integration 门槛。阻塞证据包括非零 rollback/full-stack crash、尚未证明 nonzero Native LiDAR time-offset margin，以及缺失 live GNSS innovation/recovery validation。下一安全步骤是 tethered 或 propeller-off hardware bench run，在任何 flight test 前验证 clock discipline、MID360 extrinsic、FCU GNSS innovation metadata 和 transaction integrity。

Machine-readable evidence 位于被忽略的 `logs/tmp` 目录；campaign aggregate 为 `logs/tmp/robustness_v3_campaign_summary.json`，最终 map result 为 `logs/tmp/robustness_v3_joint_map_stress_final6/robustness_joint_map_report.json`。
