# DYNAMIC-MIGRATE-008

## 迁移

已将验证过的 PR15 Dynamic Observer v2 与 Clean Scan Gateway 作为独立 package 迁移到 HXY baseline：`uf_dynamic_interfaces/PreviousFastLioState`、`uf_dynamic_observer`（causal IMU deskew、visibility-aware Dynamic Observer v2、clean admission、fail-open gateway、launch/configuration、benchmark 与 evaluator tests），以及 FAST-LIO wrapper 对 clean input topic 和 previous-state export 的支持。

运行链为：`raw MID360 /livox/lidar -> Dynamic Observer/Clean Gateway -> clean topic -> FAST-LIO -> NativeLidarFactor -> HXY/backend`。Raw `/livox/lidar` 仍供 safety/avoidance 发布，绝不原地 remap。Gateway 使用独立 MID360 `/livox/imu` 与 causal FAST-LIO previous-state export；契约不可用时 fail open，不伪造 clean scan。

## Dynamic benchmark

PR15-compatible benchmark 在 18 个 deterministic scenario、3 个 seed、2 次 repeat 中成功完成：

| 指标 | migrated v2 |
|---|---:|
| micro precision | 99.8439% |
| micro recall | 97.0854% |
| micro F1 | 98.4454% |
| macro precision | 93.5449% |
| macro recall | 85.7748% |
| macro F1 | 88.8424% |
| static preservation | 99.9859% |
| dynamic-as-static contamination | 1.8083% |
| latency P50/P95 | 7.19 / 9.78 ms |

Micro 结果超过旧 PR15 reference（98.25/76.16/85.81），但 macro recall 暴露了 hard scenario（small fast target、opening/closing door、occlusion appear/disappear、far sparse target）。没有加入扩大 recall 的 deletion heuristic。

## 测试与集成状态

`uf_dynamic_observer` 在本地 MID360 overlay 构建通过：21 个 C++ tests、3 个 evaluator-contract tests，共 40 个 CTest/pytest 结果。FAST-LIO wrapper shell syntax 与 clean launch Python 也通过基本检查。

尝试 60 s static Gazebo run 时，stack 到达 `/fusion/unified/diagnostics`，但 sensor supervisor 随后在 `/clock` validity gate 失败，未收集完整 trajectory。因此不宣称该次运行的 10/30/60 s drift statistic；失败属于环境 startup（`ROS /clock did not produce a valid Gazebo simulation timestamp`），不是 Dynamic Observer transaction 或 estimator drift。

没有改变 Z、state machine、relocalization、Dynamic V2 threshold、prediction recovery、scan contract 或 HXY mathematics。
