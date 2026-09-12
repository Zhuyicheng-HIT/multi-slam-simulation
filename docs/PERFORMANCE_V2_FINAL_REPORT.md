# Ultra-Fusion Performance V2 最终报告

## 总体结果

Estimator hot path 明显加快，安全/功能测试全部通过，但完整 V2 performance gate 未达标：6 个匹配场景的 live RTF median 仅提升约 2.0%（目标 20%），time-window rejection 为 9.62%（目标约 5%）。因此本分支是经过验证的 optimization candidate，不是最终冻结的 Performance V2 release。

## Solver、RTF 与资源

| Statistic | V1 | V2 | Delta |
|---|---:|---:|---:|
| six-run solver median | 58.847 ms | 42.399 ms | -27.95% |
| rectangle solver median | 51.973 ms | 33.789 ms | -34.99% |
| S-curve solver median | 59.794 ms | 52.059 ms | -12.94% |
| profiled optimize P50 | 52.867 ms | 40.765 ms | -22.89% |
| profiled optimize P95 | 77.503 ms | 71.980 ms | -7.13% |
| six-run RTF median | 0.460813 | 0.470098 | +2.02% |
| rectangle CPU median | 34.324% | 34.806% | +1.40% |
| S-curve CPU median | 33.447% | 38.788% | +15.97% |
| six-run RAM median | 3.202 GiB | 3.237 GiB | +1.09% |

关闭 profiling 的 rectangle run 达到首选的 solver ≤40 ms；较长的 S-curve 未达到。solver 时间下降但 CPU 未同步下降，是因为 Gazebo、bridge、camera rendering 与完整 flush 的 S-curve 证据主导了整个 WSL 统计。

## Visual health

6 次 production run 中，1112 个 quality-valid candidate 有 789 个被 solver 接受（70.95%），高于 65.8% floor；所有 accepted 都是 finite，无 track rejection。Time-window reject 为 107（9.62%），略差于 V1 合并样本和约 5% 目标。没有放宽 threshold、重写 timestamp、关闭 integrity check 或使用 future state。

Cadence scan 选择 `balanced`：light/balanced/plus 分别接受 61/78/85 个 factor，ATE 为 0.825/0.532/1.137 m，solver 为 34.716/37.550/38.962 ms。Balanced 在 accuracy/information 之间最优，目标不是盲目接受更多 factor。

## Accuracy 与 stability

Rectangle ATE 为 0.120/0.487/0.648 m，translation RPE 为 0.0286/0.0458/0.0417 m，rotation RPE 为 0.1187/0.1304/0.1279 deg；median translation RPE 比 V1 高 34%，需警惕。S-curve ATE 为 0.477/1.446/0.642 m，translation RPE 为 0.0500/0.0579/0.0442 m，rotation RPE 为 0.1117/0.1326/0.1217 deg；median translation RPE 改善 14.6%，r72 是保留的 ATE outlier。6 次运行 median ATE 改善约 11.7%，translation RPE 改善约 2.8%，无全场景系统性回归，但 rectangle warning 阻止无条件 freeze。

Unified odom 首次出现于 rectangle 的 62/62/52 s、S-curve 的 55/57/52 s；未通过改变 observability gate 优化 startup。

## Joint map regression

最终 joint run 生成 114204 个 voxel：104091 LiDAR、22444 RGB-D、12331 joint、10113 supplementary RGB-D。Color coverage 11.85%，occupied-volume growth 9.72%，conflict ratio 与 eviction 均为 0。LiDAR 仍是 geometry authority；map、unified odom 与 5 条 sensor factor path 同时运行，无 optimization error、integrity reject 或 rollback。

## 验证

15 个 ROS package 以 `RelWithDebInfo` 构建；完整 colcon 结果 57 tests、0 error/failure/skipped；backend 158/158、visual 4/4、mapping 6/6 direct tests 通过；D435i active-run lifecycle short test 通过；Python 198、YAML 29、XML 15、shell 53 项 static/syntax check 通过；`git diff --check` 通过，未遗留 live simulation/ROS process 或 listening flight port。

## Freeze 决策

不要将此 commit 标为最终冻结 Performance V2。它适合作为经测试、精确且可回退的 local optimization baseline；freeze 还需要解决 WSL GPU/rendering 环境并重跑匹配 RTF benchmark、在不放宽 0.065 s gate 的前提下将 visual time-window rejection 拉回约 5%，并增加一次 rectangle repeat 以消除 translation-RPE warning。
